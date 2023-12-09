// SPDX-License-Identifier: GPL-2.0
/*
 * key management facility for FS encryption support.
 *
 * Copyright (C) 2015, Google, Inc.
 *
 * This contains encryption key functions.
 *
 * Written by Michael Halcrow, Ildar Muslukhov, and Uday Savagaonkar, 2015.
 */

#include <keys/user-type.h>
#include <linux/scatterlist.h>
#include <linux/ratelimit.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/skcipher.h>
#include "fscrypt_private.h"
#include "fscrypt_ice.h"

static struct crypto_shash *essiv_hash_tfm;

/**
 * derive_key_aes() - Derive a key using AES-128-ECB
 * @deriving_key: Encryption key used for derivation.
 * @source_key:   Source key to which to apply derivation.
 * @derived_raw_key:  Derived raw key.
 *
 * Return: Zero on success; non-zero otherwise.
 */
static int derive_key_aes(const u8 *master_key,
			const u8 nonce[FS_KEY_DERIVATION_NONCE_SIZE],
			u8 *derived_key, unsigned int derived_keysize)
{
	int res = 0;
	struct skcipher_request *req = NULL;
	DECLARE_CRYPTO_WAIT(wait);
	struct scatterlist src_sg, dst_sg;
	struct crypto_skcipher *tfm = crypto_alloc_skcipher("ecb(aes)", 0, 0);

	if (IS_ERR(tfm)) {
		res = PTR_ERR(tfm);
		tfm = NULL;
		goto out;
	}
	crypto_skcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	req = skcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}
	skcipher_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			crypto_req_done, &wait);
	res = crypto_skcipher_setkey(tfm, nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	if (res < 0)
		goto out;

	sg_init_one(&src_sg, master_key, derived_keysize);
	sg_init_one(&dst_sg, derived_key, derived_keysize);
	skcipher_request_set_crypt(req, &src_sg, &dst_sg, derived_keysize,
				NULL);
	res = crypto_wait_req(crypto_skcipher_encrypt(req), &wait);
out:
	skcipher_request_free(req);
	crypto_free_skcipher(tfm);
	return res;
}

static struct fscrypt_mode available_modes[] = {
	[FS_ENCRYPTION_MODE_AES_256_XTS] = {
		.friendly_name = "AES-256-XTS",
		.cipher_str = "xts(aes)",
		.keysize = 64,
		.ivsize = 16,
	},
	[FS_ENCRYPTION_MODE_AES_256_CTS] = {
		.friendly_name = "AES-256-CTS-CBC",
		.cipher_str = "cts(cbc(aes))",
		.keysize = 32,
		.ivsize = 16,
	},
	[FS_ENCRYPTION_MODE_PRIVATE] = {
		.friendly_name = "ice",
		.cipher_str = NULL,
		.keysize = 64,
		.ivsize = 16,
	},
};

static void put_crypt_info(struct fscrypt_info *ci)
{
	struct key *key;

	if (!ci)
		return;

	crypto_free_skcipher(ci->ci_key.tfm);

	key = ci->ci_master_key;
	if (key) {
		struct fscrypt_master_key *mk = key->payload.data[0];

		/*
		 * Remove this inode from the list of inodes that were unlocked
		 * with the master key.
		 *
		 * In addition, if we're removing the last inode from a key that
		 * already had its secret removed, invalidate the key so that it
		 * gets removed from ->s_master_keys.
		 */
		spin_lock(&mk->mk_decrypted_inodes_lock);
		list_del(&ci->ci_master_key_link);
		spin_unlock(&mk->mk_decrypted_inodes_lock);
		if (refcount_dec_and_test(&mk->mk_refcount))
			key_invalidate(key);
		key_put(key);
	}
	memzero_explicit(ci, sizeof(*ci));
	kmem_cache_free(fscrypt_info_cachep, ci);
}

void __exit fscrypt_essiv_cleanup(void)
{
	crypto_free_shash(essiv_hash_tfm);
}

static struct fscrypt_mode *
select_encryption_mode(const union fscrypt_policy *policy,
			const struct inode *inode)
{
	if (S_ISREG(inode->i_mode))
		return &available_modes[fscrypt_policy_contents_mode(policy)];

	if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
		return &available_modes[fscrypt_policy_fnames_mode(policy)];

	WARN_ONCE(1, "fscrypt: filesystem tried to load encryption info for inode %lu, which is not encryptable (file type %d)\n",
		inode->i_ino, (inode->i_mode & S_IFMT));
	return ERR_PTR(-EINVAL);
}

struct crypto_skcipher *fscrypt_allocate_skcipher(struct fscrypt_mode *mode,
                                                  const u8 *raw_key,
                                                  const struct inode *inode)
{
        struct crypto_skcipher *tfm;
        int err;

        tfm = crypto_alloc_skcipher(mode->cipher_str, 0, 0);
        if (IS_ERR(tfm)) {
                if (PTR_ERR(tfm) == -ENOENT) {
                        return ERR_PTR(-ENOPKG);
                }
                return tfm;
        }
        if (unlikely(!mode->logged_impl_name)) {
                /*
                 * fscrypt performance can vary greatly depending on which
                 * crypto algorithm implementation is used.  Help people debug
                 * performance problems by logging the ->cra_driver_name the
                 * first time a mode is used.  Note that multiple threads can
                 * race here, but it doesn't really matter.
                 */
                mode->logged_impl_name = true;
                pr_info("fscrypt: %s using implementation \"%s\"\n",
                        mode->friendly_name,
                        crypto_skcipher_alg(tfm)->base.cra_driver_name);
        }
        crypto_skcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
        err = crypto_skcipher_setkey(tfm, raw_key, mode->keysize);
        if (err)
                goto err_free_tfm;

        return tfm;

err_free_tfm:
        crypto_free_skcipher(tfm);
        return ERR_PTR(err);
}

/*
 * Search the current task's subscribed keyrings for a "logon" key with
 * description prefix:descriptor, and if found acquire a read lock on it and
 * return a pointer to its validated payload in *payload_ret.
 */
static struct key *
find_and_lock_process_key(const char *prefix,
                          const u8 descriptor[FSCRYPT_KEY_DESCRIPTOR_SIZE],
                          unsigned int min_keysize,
                          const struct fscrypt_key **payload_ret)
{
        char *description;
        struct key *key;
        const struct user_key_payload *ukp;
        const struct fscrypt_key *payload;

        description = kasprintf(GFP_NOFS, "%s%*phN", prefix,
                                FSCRYPT_KEY_DESCRIPTOR_SIZE, descriptor);
        if (!description)
                return ERR_PTR(-ENOMEM);

        key = request_key(&key_type_logon, description, NULL);
        kfree(description);
        if (IS_ERR(key))
                return key;

        down_read(&key->sem);
        ukp = user_key_payload_locked(key);

        if (!ukp) /* was the key revoked before we acquired its semaphore? */
                goto invalid;

        payload = (const struct fscrypt_key *)ukp->data;

        if (ukp->datalen != sizeof(struct fscrypt_key) ||
            payload->size < 1 || payload->size > FSCRYPT_MAX_KEY_SIZE) {
                goto invalid;
        }

        if (payload->size < min_keysize) {
                goto invalid;
        }

        *payload_ret = payload;
        return key;
invalid:
        up_read(&key->sem);
        key_put(key);
        return ERR_PTR(-ENOKEY);
}

int fscrypt_prepare_key(struct fscrypt_prepared_key *prep_key,
                        const u8 *raw_key, unsigned int raw_key_size,
                        bool is_hw_wrapped, const struct fscrypt_info *ci)
{
        struct crypto_skcipher *tfm;

        if (WARN_ON(is_hw_wrapped || raw_key_size != ci->ci_mode->keysize))
                return -EINVAL;

        tfm = fscrypt_allocate_skcipher(ci->ci_mode, raw_key, ci->ci_inode);
        if (IS_ERR(tfm))
                return PTR_ERR(tfm);
	 /*
	  * Here we publish ->tfm with a RELEASE barrier so that concurrent
	  * tasks can ACQUIRE it.  Note that this concurrency is only
	  * possible for per-mode keys, not for per-file keys.
	  */
        smp_store_release(&prep_key->tfm, tfm);
        return 0;
}

/* Given a per-file encryption key, set up the file's crypto transform object */
int fscrypt_set_per_file_enc_key(struct fscrypt_info *ci, const u8 *raw_key)
{
        return fscrypt_prepare_key(&ci->ci_key, raw_key, ci->ci_mode->keysize,
                                   false /*is_hw_wrapped*/, ci);
}

static int setup_v1_file_key_private(struct fscrypt_info *ci,
				     const u8 *raw_master_key)
{
	if (!fscrypt_is_ice_capable(ci->ci_inode->i_sb)) {
		return -EINVAL;
	}
	/*
	 * Inline encryption: no key derivation required because IVs are
	 * assigned based on iv_sector.
	 */
	memcpy(ci->ci_raw_key, raw_master_key, FS_AES_256_XTS_KEY_SIZE);
	return 0;
}

static inline bool is_private_mode(const struct fscrypt_mode *mode)
{
	/* Using inline encryption with ICE, rather than the crypto API? */
	return mode->cipher_str == NULL;
}

static int setup_v1_file_key_derived(struct fscrypt_info *ci,
					const u8 *raw_master_key)
{
	u8 *derived_key = NULL;
	int err;

	derived_key = kmalloc(ci->ci_mode->keysize, GFP_NOFS);
	if (!derived_key)
		return -ENOMEM;

	err = derive_key_aes(raw_master_key, ci->ci_nonce,
		derived_key, ci->ci_mode->keysize);
	if (err)
		goto out;

	if (is_private_mode(ci->ci_mode))
		setup_v1_file_key_private(ci, derived_key);
	else fscrypt_set_per_file_enc_key(ci, derived_key);
out:
	kzfree(derived_key);
	return err;
}

int fscrypt_setup_v1_file_key_via_subscribed_keyrings(struct fscrypt_info *ci)
{
	struct key *key;
	const struct fscrypt_key *payload;
	int err;

	key = find_and_lock_process_key("fscrypt:",
					ci->ci_policy.v1.master_key_descriptor,
					ci->ci_mode->keysize, &payload);
	if (key == ERR_PTR(-ENOKEY) && ci->ci_inode->i_sb->s_cop->key_prefix) {
		key = find_and_lock_process_key(ci->ci_inode->i_sb->s_cop->key_prefix,
						ci->ci_policy.v1.master_key_descriptor,
						ci->ci_mode->keysize, &payload);
	}
	if (IS_ERR(key))
		return PTR_ERR(key);

	err = setup_v1_file_key_derived(ci, payload->raw);
	up_read(&key->sem);
	key_put(key);
	return err;
}

/*
 * Find the master key, then set up the inode's actual encryption key.
 *
 * If the master key is found in the filesystem-level keyring, then the
 * corresponding 'struct key' is returned in *master_key_ret with
 * ->mk_secret_sem read-locked.  This is needed to ensure that only one task
 * links the fscrypt_info into ->mk_decrypted_inodes (as multiple tasks may race
 * to create an fscrypt_info for the same inode), and to synchronize the master
 * key being removed with a new inode starting to use it.
 */
static int setup_file_encryption_key(struct fscrypt_info *ci,
					struct key **master_key_ret)
{
	struct key *key;
	struct fscrypt_master_key *mk = NULL;
	struct fscrypt_key_specifier mk_spec;
	int err;

	switch (ci->ci_policy.version) {
		case FSCRYPT_POLICY_V1:
			mk_spec.type = FSCRYPT_KEY_SPEC_TYPE_DESCRIPTOR;
			memcpy(mk_spec.u.descriptor,
				ci->ci_policy.v1.master_key_descriptor,
				FSCRYPT_KEY_DESCRIPTOR_SIZE);
			break;
		default:
			WARN_ON(1);
			return -EINVAL;
	}

	key = fscrypt_find_master_key(ci->ci_inode->i_sb, &mk_spec);
	if (IS_ERR(key)) {
		if (key != ERR_PTR(-ENOKEY) ||
			ci->ci_policy.version != FSCRYPT_POLICY_V1)
				return PTR_ERR(key);

		/*
		 * As a legacy fallback for v1 policies, search for the key in
		 * the current task's subscribed keyrings too.  Don't move this
		 * to before the search of ->s_master_keys, since users
		 * shouldn't be able to override filesystem-level keys.
		 */
		return fscrypt_setup_v1_file_key_via_subscribed_keyrings(ci);
	}

	mk = key->payload.data[0];
	down_read(&mk->mk_secret_sem);

	/* Has the secret been removed (via FS_IOC_REMOVE_ENCRYPTION_KEY)? */
	if (!is_master_key_secret_present(&mk->mk_secret)) {
		err = -ENOKEY;
		goto out_release_key;
	}

	/*
	 * Require that the master key be at least as long as the derived key.
	 * Otherwise, the derived key cannot possibly contain as much entropy as
	 * that required by the encryption mode it will be used for.  For v1
	 * policies it's also required for the KDF to work at all.
	 */
        if (mk->mk_secret.size < ci->ci_mode->keysize) {
	err = -ENOKEY;
		goto out_release_key;
	}

	switch (ci->ci_policy.version) {
	case FSCRYPT_POLICY_V1:
		err = setup_v1_file_key_derived(ci, mk->mk_secret.raw);
		break;
	default:
		WARN_ON(1);
		err = -EINVAL;
		break;
	}

	if (err)
		goto out_release_key;

	*master_key_ret = key;
	return 0;

out_release_key:
	up_read(&mk->mk_secret_sem);
	key_put(key);
	return err;
}
int fscrypt_get_encryption_info(struct inode *inode)
{
	struct fscrypt_info *crypt_info;
	union fscrypt_context ctx;
	struct fscrypt_mode *mode;
	int res;
	struct key *master_key = NULL;

	if (fscrypt_has_encryption_key(inode))
		return 0;

	res = fscrypt_initialize(inode->i_sb->s_cop->flags);
	if (res)
		return res;

	res = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (res < 0) {
		if (!fscrypt_dummy_context_enabled(inode) ||
		    IS_ENCRYPTED(inode))
			return res;
		/* Fake up a context for an unencrypted directory */
		memset(&ctx, 0, sizeof(ctx));
		ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
		ctx.v1.contents_encryption_mode = FS_ENCRYPTION_MODE_AES_256_XTS;
		ctx.v1.filenames_encryption_mode = FS_ENCRYPTION_MODE_AES_256_CTS;
		memset(ctx.v1.master_key_descriptor, 0x42, FS_KEY_DESCRIPTOR_SIZE);
		res = sizeof(ctx.v1);
	}

	crypt_info = kmem_cache_zalloc(fscrypt_info_cachep, GFP_NOFS);
	if (!crypt_info)
		return -ENOMEM;

	crypt_info->ci_inode = inode;

	/* Support 'old' ci_data_mode, used by ICE */
	crypt_info->ci_data_mode = FS_ENCRYPTION_MODE_PRIVATE;

	res = fscrypt_policy_from_context(&crypt_info->ci_policy, &ctx, res);
	if (res) {
		goto out;
	}

	memcpy(crypt_info->ci_nonce, ctx.v1.nonce,
			FS_KEY_DERIVATION_NONCE_SIZE);

	if (!fscrypt_supported_policy(&crypt_info->ci_policy, inode)) {
		res = -EINVAL;
		goto out;
	}

	mode = select_encryption_mode(&crypt_info->ci_policy, inode);
	if (IS_ERR(mode)) {
		res = PTR_ERR(mode);
		goto out;
	}

	WARN_ON(mode->ivsize > FSCRYPT_MAX_IV_SIZE);
	crypt_info->ci_mode = mode;

	res = setup_file_encryption_key(crypt_info, &master_key);
	if (res)
		goto out;

	if (cmpxchg_release(&inode->i_crypt_info, NULL, crypt_info) == NULL) {
		if (master_key) {
			struct fscrypt_master_key *mk =
				master_key->payload.data[0];

			refcount_inc(&mk->mk_refcount);
			crypt_info->ci_master_key = key_get(master_key);
			spin_lock(&mk->mk_decrypted_inodes_lock);
			list_add(&crypt_info->ci_master_key_link,
				&mk->mk_decrypted_inodes);
			spin_unlock(&mk->mk_decrypted_inodes_lock);
		}
		crypt_info = NULL;
	}
	res = 0;
out:
	if (master_key) {
		struct fscrypt_master_key *mk = master_key->payload.data[0];
		up_read(&mk->mk_secret_sem);
		key_put(master_key);
	}
	if (res == -ENOKEY)
		res = 0;
	put_crypt_info(crypt_info);
	return res;
}
EXPORT_SYMBOL(fscrypt_get_encryption_info);

void fscrypt_put_encryption_info(struct inode *inode)
{
	put_crypt_info(inode->i_crypt_info);
	inode->i_crypt_info = NULL;
}
EXPORT_SYMBOL(fscrypt_put_encryption_info);

/**
 * fscrypt_free_inode() - free an inode's fscrypt data requiring RCU delay
 * @inode: an inode being freed
 *
 * Free the inode's cached decrypted symlink target, if any.  Filesystems must
 * call this after an RCU grace period, just before they free the inode.
 */
void fscrypt_free_inode(struct inode *inode)
{
	if (IS_ENCRYPTED(inode) && S_ISLNK(inode->i_mode)) {
		kfree(inode->i_link);
		inode->i_link = NULL;
	}
}
EXPORT_SYMBOL(fscrypt_free_inode);

/**
 * fscrypt_drop_inode() - check whether the inode's master key has been removed
 * @inode: an inode being considered for eviction
 *
 * Filesystems supporting fscrypt must call this from their ->drop_inode()
 * method so that encrypted inodes are evicted as soon as they're no longer in
 * use and their master key has been removed.
 *
 * Return: 1 if fscrypt wants the inode to be evicted now, otherwise 0
 */
int fscrypt_drop_inode(struct inode *inode)
{
	const struct fscrypt_info *ci = READ_ONCE(inode->i_crypt_info);
	const struct fscrypt_master_key *mk;

	/*
	 * If ci is NULL, then the inode doesn't have an encryption key set up
	 * so it's irrelevant.  If ci_master_key is NULL, then the master key
	 * was provided via the legacy mechanism of the process-subscribed
	 * keyrings, so we don't know whether it's been removed or not.
	 */
	if (!ci || !ci->ci_master_key)
		return 0;
	mk = ci->ci_master_key->payload.data[0];

	/*
	 * With proper, non-racy use of FS_IOC_REMOVE_ENCRYPTION_KEY, all inodes
	 * protected by the key were cleaned by sync_filesystem().  But if
	 * userspace is still using the files, inodes can be dirtied between
	 * then and now.  We mustn't lose any writes, so skip dirty inodes here.
	 */
	if (inode->i_state & I_DIRTY_ALL)
		return 0;

	/*
	 * Note: since we aren't holding ->mk_secret_sem, the result here can
	 * immediately become outdated.  But there's no correctness problem with
	 * unnecessarily evicting.  Nor is there a correctness problem with not
	 * evicting while iput() is racing with the key being removed, since
	 * then the thread removing the key will either evict the inode itself
	 * or will correctly detect that it wasn't evicted due to the race.
	 */
	return !is_master_key_secret_present(&mk->mk_secret);
}
EXPORT_SYMBOL_GPL(fscrypt_drop_inode);
