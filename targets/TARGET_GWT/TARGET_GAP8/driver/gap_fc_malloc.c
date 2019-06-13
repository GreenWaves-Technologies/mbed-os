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

#include <stdint.h>
#include "gap_fc_malloc.h"

/*******************************************************************************
 * Variables, macros, structures,... definition
 ******************************************************************************/

static malloc_t __fc_malloc; /*!< FC memory allocator. */

/*******************************************************************************
 * Function definition
 ******************************************************************************/

void *FC_Malloc(int32_t size)
{
    return __malloc(&__fc_malloc, size);
}

void FC_MallocFree(void *_chunk, int32_t size)
{
    __malloc_free(&__fc_malloc, _chunk, size);
}

void *FC_MallocAlign(int32_t size, int32_t align)
{
    return __malloc_align(&__fc_malloc, size, align);
}

void FC_MallocInit()
{
    __malloc_init(&__fc_malloc, (void *) &__heapfcram_start, (int32_t) &__heapfcram_size);
}
