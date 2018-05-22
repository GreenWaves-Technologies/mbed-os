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
#ifndef _GAP_L1_MALLOC_H_
#define _GAP_L1_MALLOC_H_

#include "cmsis.h"
#include "gap_util.h"
#include "gap_malloc.h"


#ifdef FEATURE_CLUSTER

/*!
 * @addtogroup l1_malloc
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* CLUSTER L1 heap */
extern char  __heapsram_start;
extern char  __heapsram_size;

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
 * @brief Initializes the L1 allocator.
 *
 * @note .
 */
void L1_MallocInit();

void *L1_Malloc(int size);

void *L1_MallocAlign(int size, int align);

/*!
 * @brief Free memory allocated by the L1 allocator.
 *
 * @note .
 */
void L1_MallocFree(void *_chunk, int size);


/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /* FEATURE_CLUSTER*/
#endif /*_GAP_L1_MALLOC_H_*/
