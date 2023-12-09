/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fscrypt_private.h
 *
 * Copyright (C) 2015, Google, Inc.
 *
 * This contains encryption key functions.
 *
 * Written by Michael Halcrow, Ildar Muslukhov, and Uday Savagaonkar, 2015.
 */

#ifndef _FSCRYPT_PRIVATE_H
#define _FSCRYPT_PRIVATE_H

#ifndef __FS_HAS_ENCRYPTION
#define __FS_HAS_ENCRYPTION 1
#endif
#include <linux/fscrypt.h>
#include <crypto/hash.h>
#include <linux/pfk.h>

#define CONST_STRLEN(str)	(sizeof(str) - 1)

#define FS_IV_SIZE			16

#define FSCRYPT_MIN_KEY_SIZE		16

#define FS_AES_128_ECB_KEY_SIZE		16
#define FS_AES_128_CBC_KEY_SIZE		16
#define FS_AES_128_CTS_KEY_SIZE		16
#define FS_AES_256_GCM_KEY_SIZE		32
#define FS_AES_256_CBC_KEY_SIZE		32
#define FS_AES_256_CTS_KEY_SIZE		32
#define FS_AES_256_XTS_KEY_SIZE		64

#define FS_KEY_DERIVATION_NONCE_SIZE		16

struct fscrypt_context_v1 {
	u8 version; /* FSCRYPT_CONTEXT_V1 */
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 master_key_descriptor[FSCRYPT_KEY_DESCRIPTOR_SIZE];
	u8 nonce[FS_KEY_DERIVATION_NONCE_SIZE];
};

struct fscrypt_context_v2 {
	u8 version; /* FSCRYPT_CONTEXT_V2 */
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 __reserved[4];
	u8 master_key_identifier[FSCRYPT_KEY_IDENTIFIER_SIZE];
	u8 nonce[FS_KEY_DERIVATION_NONCE_SIZE];
};
/**
 * Encryption context for inode
 *
 * Protector format:
 *  1 byte: Protector format (1 = this version)
 *  1 byte: File contents encryption mode
 *  1 byte: File names encryption mode
 *  1 byte: Flags
 *  8 bytes: Master Key descriptor
 *  16 bytes: Encryption Key derivation nonce
 */
union fscrypt_context {
	u8 version;
	struct fscrypt_context_v1 v1;
	struct fscrypt_context_v2 v2;
	u8 format;
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
	u8 nonce[FS_KEY_DERIVATION_NONCE_SIZE];
} __packed;

#define FS_ENCRYPTION_CONTEXT_FORMAT_V1		1

/**
 * For encrypted symlinks, the ciphertext length is stored at the beginning
 * of the string in little-endian format.
 */
struct fscrypt_symlink_data {
	__le16 len;
	char encrypted_path[1];
} __packed;

enum ci_mode_info {
	CI_NONE_MODE = 0,
	CI_DATA_MODE,
	CI_FNAME_MODE,
};

#undef fscrypt_policy
union fscrypt_policy {
	u8 version;
	struct fscrypt_policy_v1 v1;
	struct fscrypt_policy_v2 v2;
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
};

/*
 * Return the size expected for the given fscrypt_policy based on its version
 * number, or 0 if the policy version is unrecognized.
 */
static inline int fscrypt_policy_size(const union fscrypt_policy *policy)
{
	switch (policy->version) {
	case FSCRYPT_POLICY_V1:
		return sizeof(policy->v1);
	case FSCRYPT_POLICY_V2:
		return sizeof(policy->v2);
	}
	return 0;
}

/* Return the contents encryption mode of a valid encryption policy */
static inline u8
fscrypt_policy_contents_mode(const union fscrypt_policy *policy)
{
	switch (policy->version) {
	case FSCRYPT_POLICY_V1:
		return policy->v1.contents_encryption_mode;
	case FSCRYPT_POLICY_V2:
		return policy->v2.contents_encryption_mode;
	}
	BUG();
}

/* Return the filenames encryption mode of a valid encryption policy */
static inline u8
fscrypt_policy_fnames_mode(const union fscrypt_policy *policy)
{
	switch (policy->version) {
	case FSCRYPT_POLICY_V1:
		return policy->v1.filenames_encryption_mode;
	case FSCRYPT_POLICY_V2:
	return policy->v2.filenames_encryption_mode;
	}
	BUG();
}

/* Return the flags (FSCRYPT_POLICY_FLAG*) of a valid encryption policy */
static inline u8
fscrypt_policy_flags(const union fscrypt_policy *policy)
{
	switch (policy->version) {
	case FSCRYPT_POLICY_V1:
		return policy->v1.flags;
	case FSCRYPT_POLICY_V2:
		return policy->v2.flags;
	}
	BUG();
}

/**
 * struct fscrypt_prepared_key - a key prepared for actual encryption/decryption
 * @tfm: crypto API transform object
 * @blk_key: key for blk-crypto
 *
 * Normally only one of the fields will be non-NULL.
 */
struct fscrypt_prepared_key {
	struct crypto_skcipher *tfm;
};

/*
 * A pointer to this structure is stored in the file system's in-core
 * representation of an inode.
 */
struct fscrypt_info {
	/* The key in a form prepared for actual encryption/decryption */
	struct fscrypt_prepared_key     ci_key;
	struct fscrypt_mode *ci_mode;
	u8 ci_data_mode;
	u8 ci_flags;
	struct crypto_skcipher *ci_ctfm;
	struct crypto_cipher *ci_essiv_tfm;
	struct key *ci_master_key;
	/*
	 * Link in list of inodes that were unlocked with the master key.
	 * Only used when ->ci_master_key is set.
	 */
	struct list_head ci_master_key_link;
	u8 ci_raw_key[FS_MAX_KEY_SIZE];
	/* Back-pointer to the inode */
	struct inode *ci_inode;
	/* The encryption policy used by this inode */
	union fscrypt_policy ci_policy;
	u8 ci_nonce[FS_KEY_DERIVATION_NONCE_SIZE];
	/* Hashed inode number.  Only set for IV_INO_LBLK_32 */
	u32 ci_hashed_ino;
};

typedef enum {
	FS_DECRYPT = 0,
	FS_ENCRYPT,
} fscrypt_direction_t;

#define FS_CTX_REQUIRES_FREE_ENCRYPT_FL		0x00000001
#define FS_CTX_HAS_BOUNCE_BUFFER_FL		0x00000002

static inline bool fscrypt_valid_enc_modes(u32 contents_mode,
					   u32 filenames_mode)
{
	if (contents_mode == FS_ENCRYPTION_MODE_AES_128_CBC &&
	    filenames_mode == FS_ENCRYPTION_MODE_AES_128_CTS)
		return true;

	if (contents_mode == FS_ENCRYPTION_MODE_AES_256_XTS &&
	    filenames_mode == FS_ENCRYPTION_MODE_AES_256_CTS)
		return true;

	if (contents_mode == FS_ENCRYPTION_MODE_SPECK128_256_XTS &&
	    filenames_mode == FS_ENCRYPTION_MODE_SPECK128_256_CTS)
		return true;

	if (contents_mode == FS_ENCRYPTION_MODE_PRIVATE &&
	    filenames_mode == FS_ENCRYPTION_MODE_AES_256_CTS)
		return true;

	return false;
}

/* crypto.c */
extern struct kmem_cache *fscrypt_info_cachep;
extern int fscrypt_initialize(unsigned int cop_flags);
extern int fscrypt_do_page_crypto(const struct inode *inode,
				  fscrypt_direction_t rw, u64 lblk_num,
				  struct page *src_page,
				  struct page *dest_page,
				  unsigned int len, unsigned int offs,
				  gfp_t gfp_flags);
extern struct page *fscrypt_alloc_bounce_page(gfp_t gfp_flags);

#define FSCRYPT_MAX_IV_SIZE        32
#define HKDF_CONTEXT_KEY_IDENTIFIER     1

union fscrypt_iv {
	struct {
		/* logical block number within the file */
		__le64 lblk_num;

		/* per-file nonce; only set in DIRECT_KEY mode */
		u8 nonce[FS_KEY_DERIVATION_NONCE_SIZE];
	};
	u8 raw[FSCRYPT_MAX_IV_SIZE];
	__le64 dun[FSCRYPT_MAX_IV_SIZE / sizeof(__le64)];
};

void fscrypt_generate_iv(union fscrypt_iv *iv, u64 lblk_num,
                         const struct fscrypt_info *ci);

/* fname.c */
extern int fname_encrypt(struct inode *inode, const struct qstr *iname,
			 u8 *out, unsigned int olen);
extern bool fscrypt_fname_encrypted_size(const struct inode *inode,
					 u32 orig_len, u32 max_len,
					 u32 *encrypted_len_ret);

/* keyring.c */

struct fscrypt_hkdf {
	struct crypto_shash *hmac_tfm;
};

extern int fscrypt_init_hkdf(struct fscrypt_hkdf *hkdf, const u8 *master_key,
                            unsigned int master_key_size);

extern int fscrypt_hkdf_expand(struct fscrypt_hkdf *hkdf, u8 context,
                              const u8 *info, unsigned int infolen,
                              u8 *okm, unsigned int okmlen);

extern void fscrypt_destroy_hkdf(struct fscrypt_hkdf *hkdf);


/*
 * fscrypt_master_key_secret - secret key material of an in-use master key
 */
struct fscrypt_master_key_secret {
	/*
	 * For v2 policy keys: HKDF context keyed by this master key.
	 * For v1 policy keys: not set (hkdf.hmac_tfm == NULL).
	 */
	struct fscrypt_hkdf     hkdf;

	/* Size of the raw key in bytes */
	u32			size;

	/* The raw key */
	u8			raw[FSCRYPT_MAX_KEY_SIZE];

} __randomize_layout;

static inline bool
is_master_key_secret_present(const struct fscrypt_master_key_secret *secret)
{
	/*
	 * The READ_ONCE() is only necessary for fscrypt_drop_inode() and
	 * fscrypt_key_describe().  These run in atomic context, so they can't
	 * take ->mk_secret_sem and thus 'secret' can change concurrently which
	 * would be a data race.  But they only need to know whether the secret
	 * *was* present at the time of check, so READ_ONCE() suffices.
	 */
	return READ_ONCE(secret->size) != 0;
}

/*
 * fscrypt_master_key - an in-use master key
 *
 * This represents a master encryption key which has been added to the
 * filesystem and can be used to "unlock" the encrypted files which were
 * encrypted with it.
 */
struct fscrypt_master_key {
	/* The secret key material */
	struct fscrypt_master_key_secret	mk_secret;

	/* Arbitrary key descriptor which was assigned by userspace */
	struct key              *mk_users;
	struct rw_semaphore                     mk_secret_sem;
	struct fscrypt_key_specifier		mk_spec;
	/*
	 * List of inodes that were unlocked using this key.  This allows the
	 * inodes to be evicted efficiently if the key is removed.
	 */
	struct list_head        mk_decrypted_inodes;
	spinlock_t              mk_decrypted_inodes_lock;
	/*
	 * Length of ->mk_decrypted_inodes, plus one if mk_secret is present.
	 * Once this goes to 0, the master key is removed from ->s_master_keys.
	 * The 'struct fscrypt_master_key' will continue to live as long as the
	 * 'struct key' whose payload it is, but we won't let this reference
	 * count rise again.
	 */
	refcount_t              mk_refcount;
	/* Crypto API transforms for DIRECT_KEY policies, allocated on-demand */
	struct crypto_skcipher  *mk_direct_tfms[__FSCRYPT_MODE_MAX + 1];
	/*
	 * Crypto API transforms for filesystem-layer implementation of
	 * IV_INO_LBLK_64 policies, allocated on-demand.
	 */
	struct crypto_skcipher  *mk_iv_ino_lblk_64_tfms[__FSCRYPT_MODE_MAX + 1];
} __randomize_layout;

static inline const char *master_key_spec_type(
				const struct fscrypt_key_specifier *spec)
{
	switch (spec->type) {
	case FSCRYPT_KEY_SPEC_TYPE_DESCRIPTOR:
		return "descriptor";
	}
	return "[unknown]";
}

static inline int master_key_spec_len(const struct fscrypt_key_specifier *spec)
{
	switch (spec->type) {
	case FSCRYPT_KEY_SPEC_TYPE_DESCRIPTOR:
		return FSCRYPT_KEY_DESCRIPTOR_SIZE;
	}
	return 0;
}

extern struct key *
fscrypt_find_master_key(struct super_block *sb,
			const struct fscrypt_key_specifier *mk_spec);

extern int __init fscrypt_init_keyring(void);

struct fscrypt_mode {
	const char *friendly_name;
	const char *cipher_str;
	int keysize;
	int ivsize;
	bool logged_impl_name;
	bool needs_essiv;
};

/* keyinfo.c */
extern void __exit fscrypt_essiv_cleanup(void);

bool fscrypt_supported_policy(const union fscrypt_policy *policy_u,
                              const struct inode *inode);

extern int fscrypt_policy_from_context(union fscrypt_policy *policy_u,
                                       const union fscrypt_context *ctx_u,
                                       int ctx_size);

#endif /* _FSCRYPT_PRIVATE_H */
