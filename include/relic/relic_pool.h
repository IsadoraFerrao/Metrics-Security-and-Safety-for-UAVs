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
 * @file
 *
 * Interface of the memory-management routines for the static memory allocator.
 *
 * @version $Id: relic_pool.h 1124 2012-03-18 12:59:48Z dfaranha $
 * @ingroup relic
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef RELIC_POOL_H
#define RELIC_POOL_H

#include "relic_conf.h"
#include "relic_util.h"

/*============================================================================*/
/* Constant definitions                                                       */
/*============================================================================*/

#if ALLOC == STATIC

/**
 * The size of the static pool of digit vectors.
 */
#ifndef POOL_SIZE
#define POOL_SIZE	(500 * MAX(TESTS, BENCH * BENCH))
#endif

#endif

/*============================================================================*/
/* Function prototypes                                                        */
/*============================================================================*/

#if ALLOC == STATIC

/**
 * Gets a new element from the static pool.
 *
 * @returns the address of a free element in the static pool.
 */
dig_t *pool_get(void);

/**
 * Restores an element to the static pool.
 *
 * @param[in] a			- the address to free.
 */
void pool_put(dig_t *a);

#endif

#endif /* !RELIC_POOL_H */
