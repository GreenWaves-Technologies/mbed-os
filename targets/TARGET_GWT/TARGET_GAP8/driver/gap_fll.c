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

#include "gap_fll.h"
#include "gap_pmu.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * Fll control
 * FreqOut = Fref * Mult/2^(Div-1)
 * With Mult on 16 bits and Div on 4 bits
 */

/* Maximum Log2(DCO Frequency) */
#define LOG2_MAXDCO     29
/* Maximum Log2(Clok Divider) */
#define LOG2_MAXDIV     15
/* Log2(FLL_REF_CLK=32768) */
#define LOG2_REFCLK     15
/* Maximum Log2(Multiplier) */
#define LOG2_MAXM       (LOG2_MAXDCO - LOG2_REFCLK)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t flls_frequency[FLL_NUM];

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t FLL_GetMultDivFromFrequency(uint32_t freq, uint32_t *mult, uint32_t *div)
{
    #ifdef DYNAMIC_FREQ
    uint32_t fref = FLL_REF_CLK;
    uint32_t Log2M = __FL1(freq) - __FL1(fref);
    uint32_t D = __MAX(1, (LOG2_MAXM - Log2M)>>1);
    uint32_t M = (freq << D) / fref;
    uint32_t fres;

    fres = (fref * M + (1 << (D - 1))) >> D;   /* Rounding */

    *mult = M;
    *div  = D + 1;
    return fres;
    #else
    uint32_t D = __builtin_pulp_minsi(8, __MAX(1, (8 - (__FL1(freq) - 3 - LOG2_REFCLK))));
    uint32_t M = (freq >> LOG2_REFCLK) * (1 << (D-1));
    *mult = M;
    *div  = D;
    return (((FLL_REF_CLK)*M) / (1 << (D-1)));
    #endif
}

static uint32_t FLL_GetFrequencyFromMultDiv(uint32_t mult, uint32_t div)
{
    /* FreqOut = Fref * Mult/2^(Div-1)    With Mult on 16 bits and Div on 4 bits */
    uint32_t fref = FLL_REF_CLK;
    uint32_t fres = (div == 0) ? (fref * mult) : (fref * mult) >> (div-1);
    return fres;
}

int FLL_SetFrequency(fll_type_t which_fll, uint32_t frequency, int check)
{
    #ifdef FEATURE_CLUSTER
    if ((which_fll == uFLL_CLUSTER) && PMU_ClusterIsOff())
        return -1;
    #else
    if (which_fll == uFLL_CLUSTER)
        return -1;
    #endif

    uint32_t val1, val2;
    uint32_t real_freq, mult, div;

    if (check) {
        uint32_t curr_voltage = DCDC_TO_mV(PMU_State.DCDC_Settings[READ_PMU_REGULATOR_STATE(PMU_State.State)]);

        if (which_fll == uFLL_SOC) {
            if (PMU_SoCMaxFreqAtV(curr_voltage) < (int)frequency)
                return -1;
        } else {
            if (PMU_ClusterMaxFreqAtV(curr_voltage) < (int)frequency)
                return -1;
        }
    }

    real_freq = FLL_GetMultDivFromFrequency(frequency, &mult, &div);

    /* Return to close loop mode and give gain to feedback loop */
    val2 = FLL_CTRL_CONF2_LOOPGAIN(0xB)         |
           FLL_CTRL_CONF2_DEASSERT_CYCLES(0x10) |
           FLL_CTRL_CONF2_ASSERT_CYCLES(0x10)   |
           FLL_CTRL_CONF2_LOCK_TOLERANCE(0x100) |
           FLL_CTRL_CONF2_CONF_CLK_SEL(0x0)     |
           FLL_CTRL_CONF2_OPEN_LOOP(0x0)        |
           FLL_CTRL_CONF2_DITHERING(0x1);

    if (which_fll) {
        FLL_CTRL->CLUSTER_CONF2 = val2;
    } else {
        FLL_CTRL->SOC_CONF2 = val2;
    }

    val1 = FLL_CTRL_CONF1_MODE(1)            |
           FLL_CTRL_CONF1_MULTI_FACTOR(mult) |
           FLL_CTRL_CONF1_CLK_OUT_DIV(div);

    if (which_fll) {
        FLL_CTRL->CLUSTER_CONF1 = val1;
    } else {
        FLL_CTRL->SOC_CONF1 = val1;
    }

    /* Update Frequency */
    flls_frequency[which_fll] = real_freq;
    PMU_State.Frequency[which_fll] = real_freq;

    if (which_fll) {
        FLL_CTRL->CLUSTER_CONF2 = val2;
    } else {
        FLL_CTRL->SOC_CONF2 = val2;
    }

    return real_freq;
}

void FLL_Init(fll_type_t which_fll, uint32_t ret_state)
{
    uint32_t val, val2;
    uint32_t mult, div;

    val = (which_fll) ? FLL_CTRL->CLUSTER_CONF1 : FLL_CTRL->SOC_CONF1;

    if (ret_state || READ_FLL_CTRL_CONF1_MODE(val)) {
        flls_frequency[which_fll] = FLL_GetFrequencyFromMultDiv(READ_FLL_CTRL_CONF1_MULTI_FACTOR(val),
                                                                READ_FLL_CTRL_CONF1_CLK_OUT_DIV(val));

        PMU_State.Frequency[which_fll] = flls_frequency[which_fll];
    } else {
        /* Return to close loop mode and give gain to feedback loop */
        val2 = FLL_CTRL_CONF2_LOOPGAIN(0xB)         |
               FLL_CTRL_CONF2_DEASSERT_CYCLES(0x10) |
               FLL_CTRL_CONF2_ASSERT_CYCLES(0x10)   |
               FLL_CTRL_CONF2_LOCK_TOLERANCE(0x100) |
               FLL_CTRL_CONF2_CONF_CLK_SEL(0x0)     |
               FLL_CTRL_CONF2_OPEN_LOOP(0x0)        |
               FLL_CTRL_CONF2_DITHERING(0x1);

        if (which_fll) {
            FLL_CTRL->CLUSTER_CONF2 = val2;
        } else {
            FLL_CTRL->SOC_CONF2 = val2;
        }

        /* We are in open loop, prime the fll forcing dco input, approx 70 MHz */
        val = (which_fll) ? FLL_CTRL->CLUSTER_INTEGRATOR : FLL_CTRL->SOC_INTEGRATOR;
        /* Set int part to 1*/
        val = FLL_CTRL_INTEGRATOR_INT_PART(332);

        if (which_fll) {
            FLL_CTRL->CLUSTER_INTEGRATOR = val;
        } else {
            FLL_CTRL->SOC_INTEGRATOR = val;
        }

        /* Lock Fll */
        /* Set int part to 1*/
        /* val |= FLL_CTRL_CONF1_OUTPUT_LOCK_EN(1); */
        uint32_t real_freq = FLL_GetMultDivFromFrequency(50000000, &mult, &div);

        val = FLL_CTRL_CONF1_MODE(1)            |
              FLL_CTRL_CONF1_MULTI_FACTOR(mult) |
              FLL_CTRL_CONF1_CLK_OUT_DIV(div);

        if (which_fll) {
            FLL_CTRL->CLUSTER_CONF1 = val;
        } else {
            FLL_CTRL->SOC_CONF1 = val;
        }

        /* Update Frequency */
        flls_frequency[which_fll] = real_freq;
        PMU_State.Frequency[which_fll] = real_freq;

        if (which_fll) {
            FLL_CTRL->CLUSTER_CONF2 = val2;
        } else {
            FLL_CTRL->SOC_CONF2 = val2;
        }
    }
}

void FLL_DeInit(fll_type_t which_fll)
{
    flls_frequency[which_fll] = 0;
}

void FLL_Clear()
{
  for (int i = 0; i < FLL_NUM; i++)
  {
    flls_frequency[i] = 0;
  }
}
