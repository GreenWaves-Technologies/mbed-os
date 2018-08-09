/*
 * Copyright (c) 2018, GreenWaves Technologies, Inc.
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

#ifndef _GAP_PERFORMANCE_H_
#define _GAP_PERFORMANCE_H_

#include <assert.h>
#include "cmsis.h"
#include <string.h>
/*!
 * @addtogroup performance
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PERFORMANCE_USING_TIMER_SHIFT            17U                                           /*!< PCER: Timer_CONT Position */
#define PERFORMANCE_USING_TIMER_MASK             (0x1UL << PERFORMANCE_USING_TIMER_SHIFT)      /*!< PCER: Timer CONT Mask */

/*! @brief performance transfer blocking or nonblocking hint */
typedef struct performance_s {
    uint32_t events_mask;     /*!< Events mask */
    uint32_t count[PCER_EVENTS_NUM + 1];  /*!< Count value for all events including one timer value */
} performance_t;


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
 * @brief Start the performance counter.
 *
 * This function will do initialization of performance counter and do the configurations
 * according to specific events, finally enable the counter.
 *
 * @param base The PERFORMANCE channel base pointer.
 * @param mask The logic or of wanted PERFORMANCE counter number bit mask.
 * @note .
 */
void PERFORMANCE_Start(performance_t *base, uint32_t mask);

/*!
 * @brief Stop the performance counter.
 *
 * This function will stop of performance counter and save the data
 * according to specific events, finally disable the counter.
 *
 * @param base The PERFORMANCE channel base pointer.
 * @note .
 */
void PERFORMANCE_Stop(performance_t *base);

/*!
 * @brief Get the performance counter of specific event.
 *
 * This function will return the performance counter data
 * according to specific event
 *
 * @param base  The PERFORMANCE channel base pointer.
 * @param event The PERFORMANCE counter number to read.
 * @note .
 * @return counter value of specific event
 */
uint32_t PERFORMANCE_Get(performance_t *base, uint32_t event);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_PERFORMANCE_H_*/
