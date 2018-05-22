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
#include "gap_fc_malloc.h"

#ifdef FEATURE_CLUSTER

malloc_t __fc_malloc_cluster;

void *FC_Malloc(int size)
{
    return __malloc(&__fc_malloc_cluster, size);
}

void FC_MallocFree(void *_chunk, int size)
{
    __malloc_free(&__fc_malloc_cluster, _chunk, size);
}

void *FC_MallocAlign(int size, int align)
{
    return __malloc_align(&__fc_malloc_cluster, size, align);
}

void FC_MallocInit()
{
    __malloc_init(&__fc_malloc_cluster, (void*)&__heapfcram_start, (uint32_t)&__heapfcram_size);
}

#endif
