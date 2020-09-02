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
 * @defgroup bn Multiple precision integer arithmetic.
 */

/**
 * @file
 *
 * Interface of the module for multiple precision integer arithmetic.
 *
 * @version $Id: relic_bn.h 1124 2012-03-18 12:59:48Z dfaranha $
 * @ingroup bn
 */

#ifndef RELIC_BN_H
#define RELIC_BN_H

#include <string.h>

#include "relic_dv.h"
#include "relic_conf.h"
#include "relic_util.h"
#include "relic_types.h"

/*============================================================================*/
/* Constant definitions                                                       */
/*============================================================================*/

/**
 * Precision in bits of a multiple precision integer.
 *
 * If the library is built with support for dynamic allocation, this constant
 * represents the size in bits of the memory block allocated each time a
 * multiple precision integer must grow. Otherwise, it represents the fixed
 * fixed precision.
 */
#define BN_BITS 	((int)BN_PRECI)

/**
 * Size in bits of a digit.
 */
#define BN_DIGIT	((int)DIGIT)

/**
 * Logarithm of the digit size in base 2.
 */
#define BN_DIG_LOG	((int)DIGIT_LOG)

/**
 * Size in digits of a block sufficient to store the required precision.
 */
#define BN_DIGS		((int)((BN_BITS)/(BN_DIGIT) + (BN_BITS % BN_DIGIT > 0)))

/**
 * Size in bytes of a block sufficient to store the required precision.
 */
#define BN_BYTES 	((int)((BN_BITS)/8 + ((BN_BITS % 8) > 0)))

/**
 * Size in digits of a block sufficient to store a multiple precision integer.
 */
#if BN_MAGNI == DOUBLE
#define BN_SIZE		((int)(2 * BN_DIGS + 2))
#elif BN_MAGNI == CARRY
#define BN_SIZE		((int)(BN_DIGS + 1))
#elif BN_MAGNI == SINGLE
#define BN_SIZE		((int)BN_DIGS)
#endif

/**
 * Positive sign of a multiple precision integer.
 */
#define BN_POS		(0)

/**
 * Negative sign of a multiple precision integer.
 */
#define BN_NEG		(1)

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/**
 * Represents a multiple precision integer.
 *
 * The field dp points to a vector of digits. These digits are organized
 * in little-endian format, that is, the least significant digits are
 * stored in the first positions of the vector.
 */
typedef struct {
	/** The number of digits allocated to this multiple precision integer. */
	int alloc;
	/** The number of digits actually used. */
	int used;
	/** The sign of this multiple precision integer. */
	int sign;
#if ALLOC == DYNAMIC || ALLOC == STATIC
	/** The sequence of contiguous digits that forms this integer. */
	dig_t *dp;
#elif ALLOC == STACK || ALLOC == AUTO
	/** The sequence of contiguous digits that forms this integer. */
	align dig_t dp[BN_SIZE];
#endif
} bn_st;

/**
 * Pointer to a multiple precision integer structure.
 */
#if ALLOC == AUTO
typedef bn_st bn_t[1];
#else
typedef bn_st *bn_t;
#endif

/*============================================================================*/
/* Macro definitions                                                          */
/*============================================================================*/

/**
 * Initializes a multiple precision integer with a null value.
 *
 * @param[out] A			- the multiple precision integer to initialize.
 */
#if ALLOC == AUTO
#define bn_null(A)			/* empty */
#else
#define bn_null(A)			A = NULL;
#endif

/**
 * Calls a function to allocate and initialize a multiple precision integer.
 *
 * @param[in,out] A			- the multiple precision integer to initialize.
 * @throw ERR_NO_MEMORY		- if there is no available memory.
 */
#if ALLOC == DYNAMIC
#define bn_new(A)															\
	A = (bn_t)calloc(1, sizeof(bn_st));										\
	if ((A) == NULL) {														\
		THROW(ERR_NO_MEMORY);												\
	}																		\
	bn_init(A, BN_SIZE);													\

#elif ALLOC == STATIC
#define bn_new(A)															\
	A = (bn_t)alloca(sizeof(bn_st));										\
	if ((A) != NULL) {														\
		(A)->dp = NULL;														\
	}																		\
	bn_init(A, BN_SIZE);													\

#elif ALLOC == AUTO
#define bn_new(A)															\
	bn_init(A, BN_SIZE);													\

#elif ALLOC == STACK
#define bn_new(A)															\
	A = (bn_t)alloca(sizeof(bn_st));										\
	bn_init(A, BN_SIZE);													\

#endif

/**
 * Calls a function to allocate and initialize a multiple precision integer
 * with the required precision in digits.
 *
 * @param[in,out] A			- the multiple precision integer to initialize.
 * @param[in] D				- the precision in digits.
 * @throw ERR_NO_MEMORY		- if there is no available memory.
 * @throw ERR_PRECISION		- if the required precision cannot be represented
 * 							by the library.
 */
#if ALLOC == DYNAMIC
#define bn_new_size(A, D)													\
	A = (bn_t)calloc(1, sizeof(bn_st));										\
	if (A == NULL) {														\
		THROW(ERR_NO_MEMORY);												\
	}																		\
	bn_init(A, D);															\

#elif ALLOC == STATIC
#define bn_new_size(A, D)													\
	A = (bn_t)alloca(sizeof(bn_st));										\
	if (A != NULL) {														\
		(A)->dp = NULL;														\
	}																		\
	bn_init(A, D);															\

#elif ALLOC == AUTO
#define bn_new_size(A, D)													\
	bn_init(A, D);															\

#elif ALLOC == STACK
#define bn_new_size(A, D)													\
	A = (bn_t)alloca(sizeof(bn_st));										\
	bn_init(A, D);															\

#endif

/**
 * Calls a function to clean and free a multiple precision integer.
 *
 * @param[in,out] A			- the multiple precision integer to free.
 */
#if ALLOC == DYNAMIC
#define bn_free(A)															\
	if (A != NULL) {														\
		bn_clean(A);														\
		free(A);															\
		A = NULL;															\
	}

#elif ALLOC == STATIC
#define bn_free(A)															\
	if (A != NULL) {														\
		bn_clean(A);														\
		A = NULL;															\
	}

#elif ALLOC == AUTO
#define bn_free(A)			/* empty */										\

#elif ALLOC == STACK
#define bn_free(A)															\
	A = NULL;																\

#endif

/**
 * Multiples two multiple precision integers. Computes c = a * b.
 *
 * @param[out] C			- the result.
 * @param[in] A				- the first multiple precision integer to multiply.
 * @param[in] B				- the second multiple precision integer to multiply.
 */
#if BN_KARAT > 0
#define bn_mul(C, A, B)		bn_mul_karat(C, A, B)
#elif BN_MUL == BASIC
#define bn_mul(C, A, B)		bn_mul_basic(C, A, B)
#elif BN_MUL == COMBA
#define bn_mul(C, A, B)		bn_mul_comba(C, A, B)
#endif

/**
 * Computes the square of a multiple precision integer. Computes c = a * a.
 *
 * @param[out] C			- the result.
 * @param[in] A				- the multiple precision integer to square.
 */
#if BN_KARAT > 0
#define bn_sqr(C, A)		bn_sqr_karat(C, A)
#elif BN_SQR == BASIC
#define bn_sqr(C, A)		bn_sqr_basic(C, A)
#elif BN_SQR == COMBA
#define bn_sqr(C, A)		bn_sqr_comba(C, A)
#elif BN_SQR == MULTP
#define bn_sqr(C, A)		bn_mul(C, A, A)
#endif

/**
 * Computes the auxiliar value derived from the modulus to be used during
 * modular reduction.
 *
 * @param[out] U			- the result.
 * @param[in] M				- the modulus.
 */
#if BN_MOD == BASIC
#define bn_mod_pre(U, M)	(void)(U), (void)(M)
#elif BN_MOD == BARRT
#define bn_mod_pre(U, M)	bn_mod_pre_barrt(U, M)
#elif BN_MOD == MONTY
#define bn_mod_pre(U, M)	bn_mod_pre_monty(U, M)
#elif BN_MOD == PMERS
#define bn_mod_pre(U, M)	bn_mod_pre_pmers(U, M)
#endif

/**
 * Reduces a multiple precision integer modulo another integer. If the number
 * of arguments is 3, then simple division is used. If the number of arguments
 * is 4, then a modular reduction algorithm is used and the fourth argument
 * is an auxiliary value derived from the modulus. The variant with 4 arguments
 * should be used when several modular reductions are computed with the same
 * modulus. Computes c = a mod m.
 *
 * @param[out] C			- the result.
 * @param[in] A				- the multiple precision integer to reduce.
 * @param[in] ...			- the modulus and an optional argument.
 */
#define bn_mod(C, A, ...)	CAT(bn_mod, OPT(__VA_ARGS__))(C, A, __VA_ARGS__)

/**
 * Reduces a multiple precision integer modulo another integer. This macro
 * should not be called directly. Use bn_mod() with 4 arguments instead.
 *
 * @param[out] C			- the result.
 * @param[in] A				- the the multiple precision integer to reduce.
 * @param[in] M				- the modulus.
 * @param[in] U				- the auxiliar value derived from the modulus.
 */
#if BN_MOD == BASIC
#define bn_mod_imp(C, A, M, U)	bn_mod_basic(C, A, M)
#elif BN_MOD == BARRT
#define bn_mod_imp(C, A, M, U)	bn_mod_barrt(C, A, M, U)
#elif BN_MOD == MONTY
#define bn_mod_imp(C, A, M, U)	bn_mod_monty(C, A, M, U)
#elif BN_MOD == PMERS
#define bn_mod_imp(C, A, M, U)	bn_mod_pmers(C, A, M, U)
#endif

/**
 * Reduces a multiple precision integer modulo a modulus using Montgomery
 * reduction. Computes c = a * u^(-1) (mod m).
 *
 * @param[out] C			- the result.
 * @param[in] A				- the multiple precision integer to reduce.
 * @param[in] M				- the modulus.
 * @param[in] U				- the reciprocal of the modulus.
 */
#if BN_MUL == BASIC
#define bn_mod_monty(C, A, M, U)	bn_mod_monty_basic(C, A, M, U)
#elif BN_MUL == COMBA
#define bn_mod_monty(C, A, M, U)	bn_mod_monty_comba(C, A, M, U)
#endif

/**
 * Exponentiates a multiple precision integer modulo another multiple precision
 * integer. Computes c = a^b mod m. If Montgomery reduction is used, the basis
 * must not be in Montgomery form.
 *
 * @param[out] C			- the result.
 * @param[in] A				- the basis.
 * @param[in] B				- the exponent.
 * @param[in] M				- the modulus.
 */
#if BN_MXP == BASIC
#define bn_mxp(C, A, B, M)	bn_mxp_basic(C, A, B, M)
#elif BN_MXP == SLIDE
#define bn_mxp(C, A, B, M)	bn_mxp_slide(C, A, B, M)
#elif BN_MXP == CONST
#define bn_mxp(C, A, B, M)	bn_mxp_const(C, A, B, M)
#endif

/**
 * Computes the greatest common divisor of two multiple precision integers.
 * Computes c = gcd(a, b).
 *
 * @param[out] C			- the result;
 * @param[in] A				- the first multiple precision integer.
 * @param[in] B				- the second multiple precision integer.
 */
#if BN_GCD == BASIC
#define bn_gcd(C, A, B)		bn_gcd_basic(C, A, B)
#elif BN_GCD == LEHME
#define bn_gcd(C, A, B)		bn_gcd_lehme(C, A, B)
#elif BN_GCD == STEIN
#define bn_gcd(C, A, B)		bn_gcd_stein(C, A, B)
#endif

/**
 * Computes the extended greatest common divisor of two multiple precision
 * integers. This function can be used to compute multiplicative inverses.
 * Computes c = gcd(a, b) and c = a * d + b * e.
 *
 * @param[out] C			- the result;
 * @param[out] D			- the cofactor of the first operand, cannot be NULL.
 * @param[out] E			- the cofactor of the second operand, can be NULL.
 * @param[in] A				- the first multiple precision integer.
 * @param[in] B				- the second multiple precision integer.
 */
#if BN_GCD == BASIC
#define bn_gcd_ext(C, D, E, A, B)		bn_gcd_ext_basic(C, D, E, A, B)
#elif BN_GCD == LEHME
#define bn_gcd_ext(C, D, E, A, B)		bn_gcd_ext_lehme(C, D, E, A, B)
#elif BN_GCD == STEIN
#define bn_gcd_ext(C, D, E, A, B)		bn_gcd_ext_stein(C, D, E, A, B)
#endif

/**
 * Generates a probable prime number.
 *
 * @param[out] A			- the result.
 * @param[in] B				- the length of the number in bits.
 */
#if BN_GEN == BASIC
#define bn_gen_prime(A, B)	bn_gen_prime_basic(A, B)
#elif BN_GEN == SAFEP
#define bn_gen_prime(A, B)	bn_gen_prime_safep(A, B)
#elif BN_GEN == STRON
#define bn_gen_prime(A, B)	bn_gen_prime_stron(A, B)
#endif

/*============================================================================*/
/* Function prototypes                                                        */
/*============================================================================*/

/**
 * Initializes a previously allocated multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer to initialize.
 * @param[in] digits		- the required precision in digits.
 * @throw ERR_NO_MEMORY		- if there is no available memory.
 * @throw ERR_PRECISION		- if the required precision cannot be represented
 * 							by the library.
 */
void bn_init(bn_t a, int digits);

/**
 * Cleans a multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer to free.
 */
void bn_clean(bn_t a);

/**
 * Checks the current precision of a multiple precision integer and optionally
 * expands its precision to a given size in digits.
 *
 * @param[out] a			- the multiple precision integer to expand.
 * @param[in] digits		- the number of digits to expand.
 * @throw ERR_NO_MEMORY		- if there is no available memory.
 * @throw ERR_PRECISION		- if the required precision cannot be represented
 * 							by the library.
 */
void bn_grow(bn_t a, int digits);

/**
 * Adjust the number of valid digits of a multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer to adjust.
 */
void bn_trim(bn_t a);

/**
 * Copies the second argument to the first argument.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to copy.
 */
void bn_copy(bn_t c, bn_t a);

/**
 * Returns the absolute value of a multiple precision integer.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the argument of the absolute function.
 */
void bn_abs(bn_t c, bn_t a);

/**
 * Inverts the sign of a multiple precision integer.
 *
 * @param[out] c			- the result.
 * @param[out] a			- the multiple precision integer to negate.
 */
void bn_neg(bn_t c, bn_t a);

/**
 * Returns the sign of a multiple precision integer.
 *
 * @param[in] a				- the multiple precision integer.
 * @return BN_POS if the argument is positive and BN_NEG otherwise.
 */
int bn_sign(bn_t a);

/**
 * Assigns zero to a multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer to assign.
 */
void bn_zero(bn_t a);

/**
 * Tests if a multiple precision integer is zero or not.
 *
 * @param[in] a				- the multiple precision integer to test.
 * @return 1 if the argument is zero, 0 otherwise.
 */
int bn_is_zero(bn_t a);

/**
 * Tests if a multiple precision integer is even or odd.
 *
 * @param[in] a				- the multiple precision integer to test.
 * @return 1 if the argument is even, 0 otherwise.
 */
int bn_is_even(bn_t a);

/**
 * Returns the number of bits of a multiple precision integer.
 *
 * @param[in] a				- the multiple precision integer.
 * @return number of bits.
 */
int bn_bits(bn_t a);

/**
 * Tests the bit in the given position on a multiple precision integer.
 *
 * @param[in] a				- the multiple precision integer to test.
 * @param[in] bit			- the bit position.
 * @return 0 is the bit is zero, not zero otherwise.
 */
int bn_test_bit(bn_t a, int bit);

/**
 * Reads the bit stored in the given position on a multiple precision integer.
 *
 * @param[in] a				- the multiple precision integer.
 * @param[in] bit			- the bit position to read.
 * @return the bit value.
 */
int bn_get_bit(bn_t a, int bit);

/**
 * Stores a bit in a given position on a multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer.
 * @param[in] bit			- the bit position to store.
 * @param[in] value			- the bit value.
 */
void bn_set_bit(bn_t a, int bit, int value);

/**
 * Returns the Hamming weight of a multiple precision integer.
 *
 * @param[in] a				- the multiple precision integer.
 * @return the number of non-zero bits.
 */
int bn_ham(bn_t a);

/**
 * Reads the first digit in a multiple precision integer.
 *
 * @param[out] digit		- the result.
 * @param[in] a				- the multiple precision integer.
 */
void bn_get_dig(dig_t *digit, bn_t a);

/**
 * Assigns a small positive constant to a multiple precision integer.
 *
 * The constant must fit on a multiple precision digit, or dig_t type using
 * only the number of bits specified on BN_DIGIT.
 *
 * @param[out] a			- the result.
 * @param[in] digit			- the constant to assign.
 */
void bn_set_dig(bn_t a, dig_t digit);

/**
 * Assigns a multiple precision integer to 2^b.
 *
 * @param[out] a			- the result.
 * @param[in] b				- the power of 2 to assign.
 */
void bn_set_2b(bn_t a, int b);

/**
 * Assigns a random value to a multiple precision integer.
 *
 * @param[out] a			- the multiple precision integer to assign.
 * @param[in] sign			- the sign to be assigned (BN_NEG or BN_POS).
 * @param[in] bits			- the number of bits.
 */
void bn_rand(bn_t a, int sign, int bits);

/**
 * Prints a multiple precision integer to standard output.
 *
 * @param[in] a				- the multiple precision integer to print.
 */
void bn_print(bn_t a);

/**
 * Returns the number of digits in radix necessary to store a multiple precision
 * integer. The radix must be included in the interval [2, 64].
 *
 * @param[out] size			- the result.
 * @param[in] a				- the multiple precision integer.
 * @param[in] radix			- the radix.
 * @throw ERR_NO_VALID		- if the radix is invalid.
 */
void bn_size_str(int *size, bn_t a, int radix);

/**
 * Reads a multiple precision integer from a string in a given radix. The radix
 * must be included in the interval [2, 64].
 *
 * @param[out] a			- the result.
 * @param[in] str			- the string.
 * @param[in] len			- the size of the string.
 * @param[in] radix			- the radix.
 * @throw ERR_NO_VALID		- if the radix is invalid.
 */
void bn_read_str(bn_t a, const char *str, int len, int radix);

/**
 * Writes a multiple precision integer to a string in a given radix. The radix
 * must be included in the interval [2, 64].
 *
 * @param[out] str			- the string.
 * @param[in] len			- the buffer capacity.
 * @param[in] a				- the multiple integer to write.
 * @param[in] radix			- the radix.
 * @throw ERR_NO_BUFFER		- if the buffer capacity is insufficient.
 * @throw ERR_NO_VALID		- if the radix is invalid.
 */
void bn_write_str(char *str, int len, bn_t a, int radix);

/**
 * Returns the number of bytes necessary to store a multiple precision integer.
 *
 * @param[out] size			- the result.
 * @param[in] a				- the multiple precision integer.
 */
void bn_size_bin(int *size, bn_t a);

/**
 * Reads a positive multiple precision integer from a byte vector in big-endian
 * format.
 *
 * @param[out] a			- the result.
 * @param[in] bin			- the byte vector.
 * @param[in] little		- the endianness of the data being read.
 */
void bn_read_bin(bn_t a, unsigned char *bin, int len);

/**
 * Writes a positive multiple precision integer to a byte vector in big-endian
 * format.
 *
 * @param[out] bin			- the byte vector.
 * @param[in] len		- the buffer capacity/number of bytes written.
 * @param[in] a				- the multiple integer to write.
 * @throw ERR_NO_BUFFER		- if the buffer capacity is insufficient.
 */
void bn_write_bin(unsigned char *bin, int len, bn_t a);

/**
 * Returns the number of digits necessary to store a multiple precision integer.
 *
 * @param[out] size			- the result.
 * @param[in] a				- the multiple precision integer.
 */
void bn_size_raw(int *size, bn_t a);

/**
 * Reads a positive multiple precision integer from a digit vector.
 *
 * @param[out] a			- the result.
 * @param[in] raw			- the digit vector.
 * @param[in] len			- the size of the string.
 */
void bn_read_raw(bn_t a, dig_t *raw, int len);

/**
 * Writes a positive multiple precision integer to a byte vector.
 *
 * @param[out] raw			- the digit vector.
 * @param[in] len			- the buffer capacity.
 * @param[in] a				- the multiple integer to write.
 * @param[in] sign			- the sign.
 * @throw ERR_NO_BUFFER		- if the buffer capacity is insufficient.
 */
void bn_write_raw(dig_t *raw, int len, bn_t a);

/**
 * Returns the result of an unsigned comparison between two multiple precision
 * integers.
 *
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 * @return CMP_LT if a < b, CMP_EQ if a == b and CMP_GT if a > b.
 */
int bn_cmp_abs(bn_t a, bn_t b);

/**
 * Returns the result of a signed comparison between a multiple precision
 * integer and a digit.
 *
 * @param[in] a				- the multiple precision integer.
 * @param[in] b				- the digit.
 * @return CMP_LT if a < b, CMP_EQ if a == b and CMP_GT if a > b.
 */
int bn_cmp_dig(bn_t a, dig_t b);

/**
 * Returns the result of a signed comparison between two multiple precision
 * integers.
 *
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 * @return CMP_LT if a < b, CMP_EQ if a == b and CMP_GT if a > b.
 */
int bn_cmp(bn_t a, bn_t b);

/**
 * Adds two multiple precision integers. Computes c = a + b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first multiple precision integer to add.
 * @param[in] b				- the second multiple precision integer to add.
 */
void bn_add(bn_t c, bn_t a, bn_t b);

/**
 * Adds a multiple precision integers and a digit. Computes c = a + b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to add.
 * @param[in] b				- the digit to add.
 */
void bn_add_dig(bn_t c, bn_t a, dig_t b);

/**
 * Subtracts a multiple precision integer from another, that is, computes
 * c = a - b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer.
 * @param[in] b				- the multiple precision integer to subtract.
 */
void bn_sub(bn_t c, bn_t a, bn_t b);

/**
 * Subtracts a digit from a multiple precision integer. Computes c = a - b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer.
 * @param[in] b				- the digit to subtract.
 */
void bn_sub_dig(bn_t c, bn_t a, dig_t b);

/**
 * Multiplies a multiple precision integer by a digit. Computes c = a * b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to multiply.
 * @param[in] b				- the digit to multiply.
 */
void bn_mul_dig(bn_t c, bn_t a, dig_t b);

/**
 * Multiplies two multiple precision integers using Schoolbook multiplication.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first multiple precision integer to multiply.
 * @param[in] b				- the second multiple precision integer to multiply.
 */
void bn_mul_basic(bn_t c, bn_t a, bn_t b);

/**
 * Multiplies two multiple precision integers using Comba multiplication.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first multiple precision integer to multiply.
 * @param[in] b				- the second multiple precision integer to multiply.
 */
void bn_mul_comba(bn_t c, bn_t a, bn_t b);

/**
 * Multiplies two multiple precision integers using Karatsuba multiplication.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first multiple precision integer to multiply.
 * @param[in] b				- the second multiple precision integer to multiply.
 */
void bn_mul_karat(bn_t c, bn_t a, bn_t b);

/**
 * Computes the square of a multiple precision integer using Schoolbook
 * squaring.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to square.
 */
void bn_sqr_basic(bn_t c, bn_t a);

/**
 * Computes the square of a multiple precision integer using Comba squaring.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to square.
 */
void bn_sqr_comba(bn_t c, bn_t a);

/**
 * Computes the square of a multiple precision integer using Karatsuba squaring.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to square.
 */
void bn_sqr_karat(bn_t c, bn_t a);

/**
 * Doubles a multiple precision. Computes c = a + a.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to double.
 */
void bn_dbl(bn_t c, bn_t a);

/**
 * Halves a multiple precision. Computes c = floor(a / 2)
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to halve.
 */
void bn_hlv(bn_t c, bn_t a);

/**
 * Shifts a multiple precision number to the left. Computes c = a * 2^bits.
 * c = a * 2^bits.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to shift.
 * @param[in] bits			- the number of bits to shift.
 */
void bn_lsh(bn_t c, bn_t a, int bits);

/**
 * Shifts a multiple precision number to the right. Computes
 * c = floor(a / 2^bits).
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to shift.
 * @param[in] bits			- the number of bits to shift.
 */
void bn_rsh(bn_t c, bn_t a, int bits);

/**
 * Divides a multiple precision integer by another multiple precision integer
 * without computing the remainder. Computes c = floor(a / b).
 *
 * @param[out] c			- the resultint quotient.
 * @param[in] a				- the dividend.
 * @param[in] b				- the divisor.
 * @throw ERR_NO_VALID		- if the divisor is zero.
 */
void bn_div(bn_t c, bn_t a, bn_t b);

/**
 * Divides a multiple precision integer by another multiple precision integer.
 * Computes c = floor(a / b) and d = a mod b.
 *
 * @param[out] c			- the resulting quotient.
 * @param[out] d			- the remainder.
 * @param[in] a				- the dividend.
 * @param[in] b				- the divisor.
 * @throw ERR_NO_VALID		- if the divisor is zero.
 */
void bn_div_rem(bn_t c, bn_t d, bn_t a, bn_t b);

/**
 * Divides a multiple precision integers by a digit without computing the
 * remainder. Computes c = floor(a / b).
 *
 * @param[out] c			- the resulting quotient.
 * @param[out] d			- the remainder.
 * @param[in] a				- the dividend.
 * @param[in] b				- the divisor.
 * @throw ERR_NO_VALID		- if the divisor is zero.
 */
void bn_div_dig(bn_t c, bn_t a, dig_t b);

/**
 * Divides a multiple precision integers by a digit. Computes c = floor(a / b)
 * and d = a mod b.
 *
 * @param[out] c			- the resulting quotient.
 * @param[out] d			- the remainder.
 * @param[in] a				- the dividend.
 * @param[in] b				- the divisor.
 * @throw ERR_NO_VALID		- if the divisor is zero.
 */
void bn_div_rem_dig(bn_t c, dig_t *d, bn_t a, dig_t b);

/**
 * Reduces a multiple precision integer modulo a power of 2. Computes
 * c = a mod 2^b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the dividend.
 * @param[in] b				- the exponent of the divisor.
 */
void bn_mod_2b(bn_t c, bn_t a, int b);

/**
 * Reduces a multiple precision integer modulo a digit. Computes c = a mod b.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the dividend.
 * @param[in] b				- the divisor.
 */
void bn_mod_dig(dig_t *c, bn_t a, dig_t b);

/**
 * Reduces a multiple precision integer modulo a modulus using straightforward
 * division.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to reduce.
 * @param[in] m				- the modulus.
 */
void bn_mod_basic(bn_t c, bn_t a, bn_t m);

/**
 * Computes the reciprocal of the modulus to be used in the Barrett modular
 * reduction algorithm.
 *
 * @param[out] u			- the result.
 * @param[in] m				- the modulus.
 */
void bn_mod_pre_barrt(bn_t u, bn_t m);

/**
 * Reduces a multiple precision integer modulo a modulus using Barrett
 * reduction.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the the multiple precision integer to reduce.
 * @param[in] m				- the modulus.
 * @param[in] u				- the reciprocal of the modulus.
 */
void bn_mod_barrt(bn_t c, bn_t a, bn_t m, bn_t u);

/**
 * Computes the reciprocal of the modulus to be used in the Montgomery reduction
 * algorithm.
 *
 * @param[out] u			- the result.
 * @param[in] m				- the modulus.
 */
void bn_mod_pre_monty(bn_t u, bn_t m);

/**
 * Converts a multiple precision integer to Montgomery form.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to convert.
 * @param[in] m				- the modulus.
 */
void bn_mod_monty_conv(bn_t c, bn_t a, bn_t m);

/**
 * Converts a multiple precision integer from Montgomery form.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to convert.
 * @param[in] m				- the modulus.
 */
void bn_mod_monty_back(bn_t c, bn_t a, bn_t m);

/**
 * Reduces a multiple precision integer modulo a modulus using Montgomery
 * reduction with Schoolbook multiplication.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to reduce.
 * @param[in] m				- the modulus.
 * @param[in] u				- the reciprocal of the modulus.
 */
void bn_mod_monty_basic(bn_t c, bn_t a, bn_t m, bn_t u);

/**
 * Reduces a multiple precision integer modulo a modulus using Montgomery
 * reduction with Comba multiplication.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to reduce.
 * @param[in] m				- the modulus.
 * @param[in] u				- the reciprocal of the modulus.
 */
void bn_mod_monty_comba(bn_t c, bn_t a, bn_t m, bn_t u);

/**
 * Computes u if the modulus has the form 2^b - u.
 *
 * @param[out] u			- the result.
 * @param[in] m				- the modulus.
 */
void bn_mod_pre_pmers(bn_t u, bn_t m);

/**
 * Reduces a multiple precision integer modulo a modulus using Pseudo-Mersenne
 * modular reduction.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the multiple precision integer to reduce.
 * @param[in] m				- the modulus.
 * @param[in] u				- the auxiliar value derived from the modulus.
 */
void bn_mod_pmers(bn_t c, bn_t a, bn_t m, bn_t u);

/**
 * Exponentiates a multiple precision integer modulo a modulus using the Binary
 * method.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the basis.
 * @param[in] b				- the exponent.
 * @param[in] m				- the modulus.
 */
void bn_mxp_basic(bn_t c, bn_t a, bn_t b, bn_t m);

/**
 * Exponentiates a multiple precision integer modulo a modulus using the
 * Sliding window method.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the basis.
 * @param[in] b				- the exponent.
 * @param[in] m				- the modulus.
 */
void bn_mxp_slide(bn_t c, bn_t a, bn_t b, bn_t m);

/**
 * Exponentiates a multiple precision integer modulo a modulus using a
 * constant-time Montgomery powering ladder method.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the basis.
 * @param[in] b				- the exponent.
 * @param[in] m				- the modulus.
 */
void bn_mxp_monty(bn_t c, bn_t a, bn_t b, bn_t m);

/**
 * Computes the greatest common divisor of two multiple precision integers
 * using the standard Euclidean algorithm.
 *
 * @param[out] c			- the result;
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_basic(bn_t c, bn_t a, bn_t b);

/**
 * Computes the greatest common divisor of two multiple precision integers
 * using Lehmer's GCD algorithm.
 *
 * @param[out] c			- the result;
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_lehme(bn_t c, bn_t a, bn_t b);

/**
 * Computes the greatest common divisor of two multiple precision integers
 * using Stein's binary GCD algorithm.
 *
 * @param[out] c			- the result;
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_stein(bn_t c, bn_t a, bn_t b);

/**
 * Computes the greatest common divisor of a multiple precision integer and a
 * digit.
 *
 * @param[out] c			- the result;
 * @param[in] a				- the multiple precision integer.
 * @param[in] b				- the digit.
 */
void bn_gcd_dig(bn_t c, bn_t a, dig_t b);

/**
 * Computes the extended greatest common divisor of two multiple precision
 * integer using the Euclidean algorithm.
 *
 * @param[out] c			- the result.
 * @param[out] d			- the cofactor of the first operand, can be NULL.
 * @param[out] e			- the cofactor of the second operand, can be NULL.
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_ext_basic(bn_t c, bn_t d, bn_t e, bn_t a, bn_t b);

/**
 * Computes the greatest common divisor of two multiple precision integers
 * using Lehmer's algorithm.
 *
 * @param[out] c			- the result;
 * @param[out] d			- the cofactor of the first operand, can be NULL.
 * @param[out] e			- the cofactor of the second operand, can be NULL.
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_ext_lehme(bn_t c, bn_t d, bn_t e, bn_t a, bn_t b);

/**
 * Computes the greatest common divisor of two multiple precision integers
 * using Stein's binary algorithm.
 *
 * @param[out] c			- the result;
 * @param[out] d			- the cofactor of the first operand, can be NULL.
 * @param[out] e			- the cofactor of the second operand, can be NULL.
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_ext_stein(bn_t c, bn_t d, bn_t e, bn_t a, bn_t b);

/**
 * Computes the extended greatest common divisor of two multiple precision
 * halfway through the algorithm returning also two short vectors
 * v1 = (c, d), v2 = (-d, e) useful to decompose an integer k into k0, k1 such
 * that k = k_0 + k_1 * a (mod b).
 *
 * @param[out] c			- the first component of the first vector.
 * @param[out] d			- the second component of the first vector.
 * @param[out] e			- the first component of the second vector.
 * @param[out] f			- the second component of the second vector.
 * @param[in] a				- the first multiple precision integer.
 * @param[in] b				- the second multiple precision integer.
 */
void bn_gcd_ext_mid(bn_t c, bn_t d, bn_t e, bn_t f, bn_t a, bn_t b);

/**
 * Computes the extended greatest common divisor of a multiple precision integer
 * and a digit.
 *
 * @param[out] c			- the result.
 * @param[out] d			- the cofactor of the first operand, can be NULL.
 * @param[out] e			- the cofactor of the second operand, can be NULL.
 * @param[in] a				- the multiple precision integer.
 * @param[in] b				- the digit.
 */
void bn_gcd_ext_dig(bn_t c, bn_t d, bn_t e, bn_t a, dig_t b);

/**
 * Computes the last common multiple of two multiple precision integers.
 * Computes c = lcm(a, b).
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first integer.
 * @param[in] b				- the second integer.
 */
void bn_lcm(bn_t c, bn_t a, bn_t b);

/**
 * Computes the Legendre symbol c = (a|b), b prime.
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first parameter.
 * @param[in] b				- the second parameter.
 */
void bn_smb_leg(bn_t c, bn_t a, bn_t b);

/**
 * Computes the Jacobi symbol c = (a|b).
 *
 * @param[out] c			- the result.
 * @param[in] a				- the first parameter.
 * @param[in] b				- the second parameter.
 */
void bn_smb_jac(bn_t c, bn_t a, bn_t b);

/**
 * Tests if a number is a probable prime.
 *
 * @param[in] a				- the multiple precision integer to test.
 * @return 1 if a is prime, 0 otherwise.
 */
int bn_is_prime(bn_t a);

/**
 * Tests if a number is prime using a series of trial divisions.
 *
 * @param[in] a				- the number to test.
 * @return 1 if a is a probable prime, 0 otherwise.
 */
int bn_is_prime_basic(bn_t a);

/**
 * Tests if a number a > 2 is prime using the Miller-Rabin test with probability
 * 2^(-80) of error.
 *
 * @param[in] a				- the number to test.
 * @return 1 if a is a probable prime, 0 otherwise.
 */
int bn_is_prime_rabin(bn_t a);

/**
 * Tests if a number a > 2 is prime using the Solovay-Strassen test with
 * probability 2^(-80) of error.
 *
 * @param[in] a				- the number to test.
 * @return 1 if a is a probable prime, 0 otherwise.
 */
int bn_is_prime_solov(bn_t a);

/**
 * Generates a probable prime number.
 *
 * @param[out] a			- the result.
 * @param[in] bits			- the length of the number in bits.
 */
void bn_gen_prime_basic(bn_t a, int bits);

/**
 * Generates a probable prime number a with (a - 1)/2 also prime.
 *
 * @param[out] a			- the result.
 * @param[in] bits			- the length of the number in bits.
 */
void bn_gen_prime_safep(bn_t a, int bits);

/**
 * Generates a probable prime number with (a - 1)/2, (a + 1)/2 and
 * ((a - 1)/2 - 1)/2 also prime.
 *
 * @param[out] a			- the result.
 * @param[in] bits			- the length of the number in bits.
 */
void bn_gen_prime_stron(bn_t a, int bits);

/**
 * Tries to factorize an integer using Pollard (p - 1) factoring algorithm.
 * The maximum length of the returned factor is 16 bits.
 *
 * @param[out] c			- the resulting factor.
 * @param[in] a				- the integer to fatorize.
 * @return 1 if a factor is found and stored into c; 0 otherwise.
 */
int bn_factor(bn_t c, bn_t a);

/**
 * Tests if an integer divides other integer.
 *
 * @param[in] c				- the factor.
 * @param[in] a				- the integer.
 * @return 1 if the first integer is a factor; 0 otherwise.
 */
int bn_is_factor(bn_t c, bn_t a);

/**
 * Recodes an integer in window form.
 *
 * @param[out] win			- the recoded integer.
 * @param[out] len			- the number of bytes written.
 * @param[in] k				- the integer to recode.
 * @param[in] w				- the window size in bits.
 */
void bn_rec_win(unsigned char *win, int *len, bn_t k, int w);

/**
 * Recodes an integer in sliding window form.
 *
 * @param[out] win			- the recoded integer.
 * @param[out] len			- the number of bytes written.
 * @param[in] k				- the integer to recode.
 * @param[in] w				- the window size in bits.
 */
void bn_rec_slw(unsigned char *win, int *len, bn_t k, int w);

/**
 * Recodes an integer in width-w Non-Adjacent Form.
 *
 * @param[out] naf			- the recoded integer.
 * @param[out] len			- the number of bytes written.
 * @param[in] k				- the integer to recode.
 * @param[in] w				- the window size in bits.
 */
void bn_rec_naf(signed char *naf, int *len, bn_t k, int w);

/**
 * Recodes an integer in width-w t-NAF.
 *
 * @param[out] tnaf			- the recoded integer.
 * @param[out] len			- the number of bytes written.
 * @param[in] k				- the integer to recode.
 * @param[in] vm			- the V_m curve parameter.
 * @param[in] s0			- the S_0 curve parameter.
 * @param[in] s1			- the S_1 curve parameter.
 * @param[in] u				- the u curve parameter.
 * @param[in] m				- the extension degree of the binary field.
 * @param[in] w				- the window size in bits.
 */
void bn_rec_tnaf(signed char *tnaf, int *len, bn_t k, bn_t vm, bn_t s0, bn_t s1,
		signed char u, int m, int w);

/**
 * Recodes a pair of integers in Joint Sparse Form.
 *
 * @param[out] jsf			- the recoded pair of integers.
 * @param[out] len			- the number of bytes written.
 * @param[in] k				- the first integer to recode.
 * @param[in] l				- the second integer to recode.
 */
void bn_rec_jsf(signed char *jsf, int *len, bn_t k, bn_t l);

/**
 * Recodes an integer in two parts such that k = k0 + phi(k1), where
 * phi is the efficient curve endomorphism.
 *
 * @param[out] k0			- the first part of the result.
 * @param[out] k1			- the second part of the result.
 * @param[in] k				- the integer to recode.
 * @param[in] n				- the group order.
 * @param[in] v1			- the set of parameters v1 for the GLV method.
 * @param[in] v2			- the set of parameters v2 for the GLV method.
 */
void bn_rec_glv(bn_t k0, bn_t k1, bn_t k, bn_t n, bn_t v1[], bn_t v2[]);

#endif /* !RELIC_BN_H */
