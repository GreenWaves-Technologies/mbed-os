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
#include "gap_performance.h"
#include "gap_timer.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t fc_total_cycles = 0;

#ifdef FEATURE_CLUSTER
uint32_t cluster_total_cycles = 0;
#endif

/*!
 * @brief Initializes the performance counter.
 *
 * This function ungates the performance clock and configures the performance peripheral according
 * to the configuration structure.
 *
 * @param base The PERFORMANCE channel base pointer.
 * @note .
 */
static inline void PERFORMANCE_Init(performance_t *base)
{
    memset((void *)base, 0, sizeof(performance_t));
}

/*!
 * @brief Configure the performance counter for specific events.
 *
 * This function ungates the performance clock and configures the performance peripheral according
 * to the configuration structure.
 *
 * @param base The PERFORMANCE channel base pointer.
 * @param mask The logic or of wanted events bit mask.
 * @note .
 */
static inline void PERFORMANCE_Config(performance_t *base, uint32_t mask)
{
    base->events_mask = mask;

    __PCER_Set(mask);
}

/*!
 * @brief Initialize and enable the Cluster Timer.
 *
 * @param timer The Cluster Timer to enable.
 * @note .
 */
static inline void PERFORMANCE_Timer_Start(uint32_t timer)
{
    Timer_Initialize(timer, 0);

    Timer_Enable(timer);
}

void PERFORMANCE_Start(performance_t *base, uint32_t mask)
{
    if (mask == PERFORMANCE_USING_TIMER_MASK) {
        base->events_mask = mask;

        #ifdef FEATURE_CLUSTER
        cluster_total_cycles = 0;
        if( !__is_FC() )
            PERFORMANCE_Timer_Start(TIMER0_CLUSTER);
        else
        #endif
            PERFORMANCE_Timer_Start(TIMER1);
        fc_total_cycles = 0;
    } else {
        PERFORMANCE_Init(base);

        PERFORMANCE_Config(base, mask);

        /* Reset all PCCR to 0 */
        __PCCR31_Set(0);

        /* Enable PCMR */
        __PCMR_Set((1 << PCMR_GLBEN_Pos) | (1 << PCMR_SATU_Pos));
    }
}

/*!
 * @brief Save the performance counter values for specific events.
 *
 * @param base The PERFORMANCE channel base pointer.
 * @note .
 */
static inline void PERFORMANCE_Save(performance_t *base)
{
    uint32_t mask = base->events_mask;

    if (mask == PERFORMANCE_USING_TIMER_MASK) {
        #ifdef FEATURE_CLUSTER
        if( !__is_FC() )
            cluster_total_cycles += Timer_ReadCycle(TIMER0_CLUSTER);
        else
        #endif
            fc_total_cycles += Timer_ReadCycle(TIMER1);
    } else {

        while (mask)
        {
            int event = __FL1(mask);

            mask &= ~(1 << event);

            base->count[event] += __PCCRs_Get(event);
        }
    }
}

void PERFORMANCE_Stop(performance_t *base)
{
    if (base->events_mask == PERFORMANCE_USING_TIMER_MASK) {
        PERFORMANCE_Save(base);

        /* Disable Timer */
        #ifdef FEATURE_CLUSTER
        if( !__is_FC() )
        {
            Timer_Disable(TIMER0_CLUSTER);
        }
        else
        #endif
        {
            Timer_Disable(TIMER1);
        }
    } else {

        /* Disable PCMR */
        __PCMR_Set(0);

        PERFORMANCE_Save(base);
    }
}

uint32_t PERFORMANCE_Get(performance_t *base, uint32_t event)
{
    if (base->events_mask == PERFORMANCE_USING_TIMER_MASK) {
        #ifdef FEATURE_CLUSTER
        if( !__is_FC() )
            return cluster_total_cycles;
        else
        #endif
            return fc_total_cycles;
    }

    return base->count[event];
}
