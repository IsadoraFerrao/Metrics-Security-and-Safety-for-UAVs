/*
 * RELIC is an Efficient LIbrary for Cryptography
 * Copyright (C) 2007-2012 RELIC Authors
 *
 * This file is part of RELIC. RELIC is legal property of its developers,
 * whose names are not listed here. Please refer to the COPYRIGHT file
 * for contact information.
 *
 * RELIC is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * RELIC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RELIC. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @defgroup md Hash functions.
 */

/**
 * @file
 *
 * Interface of the module for computing hash functions.
 *
 * @version $Id: relic_md.h 1124 2012-03-18 12:59:48Z dfaranha $
 * @ingroup md
 */

#ifndef RELIC_MD_H
#define RELIC_MD_H

#include "relic_conf.h"
#include "relic_types.h"

/*============================================================================*/
/* Constant definitions                                                       */
/*============================================================================*/

enum {
	/** Hash length for SHA-1 function. */
	MD_LEN_SHONE = 20,
	/** Hash kength for SHA-224 function. */
	MD_LEN_SH224 = 28,
	/** Hash kength for SHA-256 function. */
	MD_LEN_SH256 = 32,
	/** Hash kength for SHA-384 function. */
	MD_LEN_SH384 = 48,
	/** Hash kength for SHA-512 function. */
	MD_LEN_SH512 = 64
};

/**
 * Length in bytes of default hash function output.
 */
#if MD_MAP == SHONE
#define MD_LEN					MD_LEN_SHONE
#elif MD_MAP == SH224
#define MD_LEN					MD_LEN_SH224
#elif MD_MAP == SH256
#define MD_LEN					MD_LEN_SH256
#elif MD_MAP == SH384
#define MD_LEN					MD_LEN_SH384
#elif MD_MAP == SH512
#define MD_LEN					MD_LEN_SH512
#endif

/*============================================================================*/
/* Macro definitions                                                          */
/*============================================================================*/

/**
 * Maps a byte vector to a fixed-length byte vector using the chosen hash
 * function.
 *
 * @param[out] H				- the digest.
 * @param[in] M					- the message to hash.
 * @param[in] L					- the message length in bytes.
 */
#if MD_MAP == SHONE
#define md_map(H, M, L)			md_map_shone(H, M, L)
#elif MD_MAP == SH224
#define md_map(H, M, L)			md_map_sh224(H, M, L)
#elif MD_MAP == SH256
#define md_map(H, M, L)			md_map_sh256(H, M, L)
#elif MD_MAP == SH384
#define md_map(H, M, L)			md_map_sh384(H, M, L)
#elif MD_MAP == SH512
#define md_map(H, M, L)			md_map_sh512(H, M, L)
#endif

/*============================================================================*/
/* Function prototypes                                                        */
/*============================================================================*/

/**
 * Computes the SHA-1 hash function.
 *
 * @param[out] hash				- the digest.
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_shone(unsigned char *hash, unsigned char *msg, int len);

/**
 * Initializes the hash function context.
 */
void md_map_shone_init(void);

/**
 * Updates the hash function context with more data.
 *
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_shone_update(unsigned char *msg, int len);

/**
 * Finalizes the hash function computation.
 *
 * @param[out] hash				- the digest.
 */
void md_map_shone_final(unsigned char *hash);

/**
 * Returns the internal state of the hash function.
 *
 * @param[out] state			- the internal state.
 */
void md_map_shone_state(unsigned char *state);

/**
 * Computes the SHA-224 hash function.
 *
 * @param[out] hash				- the digest.
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_sh224(unsigned char *hash, unsigned char *msg, int len);

/**
 * Computes the SHA-256 hash function.
 *
 * @param[out] hash				- the digest.
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_sh256(unsigned char *hash, unsigned char *msg, int len);

/**
 * Computes the SHA-384 hash function.
 *
 * @param[out] hash				- the digest.
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_sh384(unsigned char *hash, unsigned char *msg, int len);

/**
 * Computes the SHA-512 hash function.
 *
 * @param[out] hash				- the digest.
 * @param[in] msg				- the message to hash.
 * @param[in] len				- the message length in bytes.
 */
void md_map_sh512(unsigned char *hash, unsigned char *msg, int len);

/**
 * Derives a key from shared secret material through the standardized KDF1
 * function.
 *
 * @param[out] key				- the resulting key.
 * @param[in] key_len			- the intended key length in bytes.
 * @param[in] in				- the shared secret.
 * @param[in] in_len			- the length of the shared secret in bytes.
 */
void md_kdf1(unsigned char *key, int key_len, unsigned char *in, int in_len);

/**
 * Derives a key from shared secret material through the standardized KDF2
 * function.
 *
 * @param[out] key				- the resulting key.
 * @param[in] key_len			- the intended key length in bytes.
 * @param[in] in				- the shared secret.
 * @param[in] in_len			- the length of the shared secret in bytes.
 */
void md_kdf2(unsigned char *key, int key_len, unsigned char *in, int in_len);

/**
 * Derives a mask from shared secret material through the PKCS#1 2.0 MGF1
 * function .
 *
 * @param[out] key				- the resulting mask.
 * @param[in] key_len			- the intended mask length in bytes.
 * @param[in] in				- the shared secret.
 * @param[in] in_len			- the length of the shared secret in bytes.
 */
void md_mgf1(unsigned char *mask, int mask_len, unsigned char *in, int in_len);

#endif /* !RELIC_MD_H */
