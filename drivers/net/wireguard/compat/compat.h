/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2019 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
 */

#ifndef _WG_COMPAT_H
#define _WG_COMPAT_H

#include <linux/ktime.h>
#include <linux/timekeeping.h>
static inline u64 ktime_get_coarse_boottime_ns(void)
{
	return ktime_to_ns(ktime_mono_to_any(ns_to_ktime(jiffies64_to_nsecs(get_jiffies_64())), TK_OFFS_BOOT));
}

#include <linux/vmalloc.h>
#include <linux/mm.h>
static inline void *__compat_kvcalloc(size_t n, size_t size, gfp_t flags)
{
        return kvmalloc_array(n, size, flags | __GFP_ZERO);
}
#define kvcalloc __compat_kvcalloc

#include <net/genetlink.h>
#define genl_dump_check_consistent(a, b) genl_dump_check_consistent(a, b, &genl_family)

/* Note that all intentional uses of the non-_bh variety need to explicitly
 * undef these, conditionalized on COMPAT_CANNOT_DEPRECIATE_BH_RCU.
 */
#include <linux/rcupdate.h>
static __always_inline void old_synchronize_rcu(void)
{
	synchronize_rcu();
}
static __always_inline void old_call_rcu(void *a, void *b)
{
	call_rcu(a, b);
}
static __always_inline void old_rcu_barrier(void)
{
	rcu_barrier();
}
#ifdef synchronize_rcu
#undef synchronize_rcu
#endif
#ifdef call_rcu
#undef call_rcu
#endif
#ifdef rcu_barrier
#undef rcu_barrier
#endif
#define synchronize_rcu synchronize_rcu_bh
#define call_rcu call_rcu_bh
#define rcu_barrier rcu_barrier_bh
#define COMPAT_CANNOT_DEPRECIATE_BH_RCU

#if defined(__aarch64__)
#define cpu_have_named_feature(name) (elf_hwcap & (HWCAP_ ## name))
#endif

#define blake2s_init zinc_blake2s_init
#define blake2s_init_key zinc_blake2s_init_key
#define blake2s_update zinc_blake2s_update
#define blake2s_final zinc_blake2s_final
#define blake2s_hmac zinc_blake2s_hmac
#define chacha20 zinc_chacha20
#define hchacha20 zinc_hchacha20
#define chacha20poly1305_encrypt zinc_chacha20poly1305_encrypt
#define chacha20poly1305_encrypt_sg_inplace zinc_chacha20poly1305_encrypt_sg_inplace
#define chacha20poly1305_decrypt zinc_chacha20poly1305_decrypt
#define chacha20poly1305_decrypt_sg_inplace zinc_chacha20poly1305_decrypt_sg_inplace
#define xchacha20poly1305_encrypt zinc_xchacha20poly1305_encrypt
#define xchacha20poly1305_decrypt zinc_xchacha20poly1305_decrypt
#define curve25519 zinc_curve25519
#define curve25519_generate_secret zinc_curve25519_generate_secret
#define curve25519_generate_public zinc_curve25519_generate_public
#define poly1305_init zinc_poly1305_init
#define poly1305_update zinc_poly1305_update
#define poly1305_final zinc_poly1305_final
#define blake2s_compress_ssse3 zinc_blake2s_compress_ssse3
#define blake2s_compress_avx512 zinc_blake2s_compress_avx512
#define poly1305_init_arm zinc_poly1305_init_arm
#define poly1305_blocks_arm zinc_poly1305_blocks_arm
#define poly1305_emit_arm zinc_poly1305_emit_arm
#define poly1305_blocks_neon zinc_poly1305_blocks_neon
#define poly1305_emit_neon zinc_poly1305_emit_neon
#define poly1305_init_mips zinc_poly1305_init_mips
#define poly1305_blocks_mips zinc_poly1305_blocks_mips
#define poly1305_emit_mips zinc_poly1305_emit_mips
#define poly1305_init_x86_64 zinc_poly1305_init_x86_64
#define poly1305_blocks_x86_64 zinc_poly1305_blocks_x86_64
#define poly1305_emit_x86_64 zinc_poly1305_emit_x86_64
#define poly1305_emit_avx zinc_poly1305_emit_avx
#define poly1305_blocks_avx zinc_poly1305_blocks_avx
#define poly1305_blocks_avx2 zinc_poly1305_blocks_avx2
#define poly1305_blocks_avx512 zinc_poly1305_blocks_avx512
#define curve25519_neon zinc_curve25519_neon
#define hchacha20_ssse3 zinc_hchacha20_ssse3
#define chacha20_ssse3 zinc_chacha20_ssse3
#define chacha20_avx2 zinc_chacha20_avx2
#define chacha20_avx512 zinc_chacha20_avx512
#define chacha20_avx512vl zinc_chacha20_avx512vl
#define chacha20_mips zinc_chacha20_mips
#define chacha20_arm zinc_chacha20_arm
#define hchacha20_arm zinc_hchacha20_arm
#define chacha20_neon zinc_chacha20_neon

#endif /* _WG_COMPAT_H */
