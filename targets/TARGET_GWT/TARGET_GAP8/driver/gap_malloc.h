/*
 * Copyright (c) 2017, GreenWaves Technologies, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of GreenWaves Technologies, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _GAP_MALLOC_H_
#define _GAP_MALLOC_H_

#include "cmsis.h"
#include "gap_util.h"

/*!
 * @addtogroup malloc
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct malloc_block_s {
    int                      size;
    struct malloc_block_s *next;
} malloc_chunk_t;


typedef struct malloc_s {
    malloc_chunk_t *first_free;
} malloc_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Get malloc info.
 *
 * @note .
 */
void __malloc_info(malloc_t *a, int *_size, void **first_chunk, int *_nb_chunks);

/*!
 * @brief Print malloc.
 *
 * @note .
 */
void __malloc_dump(malloc_t *a);

/*!
 * @brief Initializes malloc.
 *
 * @note .
 */
void __malloc_init(malloc_t *a, void *_chunk, int size);

/*!
 * @brief Allocate memory.
 *
 * @note .
 * @return Start address of memory
 */
void *__malloc(malloc_t *a, int size);

/*!
 * @brief Free the l1 malloc.
 *
 * @note .
 */
void __attribute__((noinline)) __malloc_free(malloc_t *a, void *_chunk, int size);

/*!
 * @brief Memory Alignment.
 *
 * @note .
 */
void *__malloc_align(malloc_t *a, int size, int align);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_MALLOC_H_*/
