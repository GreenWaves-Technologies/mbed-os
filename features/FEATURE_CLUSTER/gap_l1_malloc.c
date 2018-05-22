/*
 * Copyright (C) 2017 ETH Zurich, University of Bologna and GreenWaves Technologies
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Authors: Eric Flamand, GreenWaves Technologies (eric.flamand@greenwaves-technologies.com)
 *          Germain Haugou, ETH (germain.haugou@iis.ee.ethz.ch)
 */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "gap_l1_malloc.h"

#ifdef FEATURE_CLUSTER

malloc_t __l1_malloc_cluster;

void *L1_Malloc(int size)
{
    return __malloc(&__l1_malloc_cluster, size);
}

void L1_MallocFree(void *_chunk, int size)
{
    __malloc_free(&__l1_malloc_cluster, _chunk, size);
}

void *L1_MallocAlign(int size, int align)
{
    return __malloc_align(&__l1_malloc_cluster, size, align);
}

void L1_MallocInit()
{
    __malloc_init(&__l1_malloc_cluster, (void*)&__heapsram_start, (uint32_t)&__heapsram_size);
}

#endif
