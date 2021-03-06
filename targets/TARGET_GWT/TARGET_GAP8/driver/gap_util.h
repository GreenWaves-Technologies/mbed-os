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

#ifndef _GAP_UTIL_H_
#define _GAP_UTIL_H_

#include "gap_eu.h"

/*!
 * @addtogroup util
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern uint32_t cluster_is_on;

#ifdef FEATURE_CLUSTER
extern uint32_t cluster_status;

/* For PMU events
   bit0 : MSP
   bit1 : ICU_MODE_CHANGED
   bit2 : ICU_OK
   bit3 : ICU_DELAYED
   bit4 : PICL_OK
   bit5 : SCU_OK
*/
extern uint32_t pmu_events_status;
#endif

/*! @brief Type used for all status and error return values. */
typedef int32_t status_t;

/*! @brief Construct a status code value from a group and code number. */
#define MAKE_VERSION(major, minor, bugfix) (((major) << 16) | ((minor) << 8) | (bugfix))
/*! @brief Construct the version number for drivers. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))

/*! @brief Computes the number of elements in an array. */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/*! @brief Status group numbers. */
enum _status_groups
{
    uStatusGroup_Generic = 0,                 /*!< Group number for generic status codes. */
    uStatusGroup_SPI = 1,                     /*!< Group number for SPI status codes. */
    uStatusGroup_UART = 2,                    /*!< Group number for UART status codes. */
    uStatusGroup_I2C = 3,                     /*!< Group number for I2C status codes. */
    uStatusGroup_SAI = 4,                     /*!< Group number for I2S status codes. */
    uStatusGroup_CPI = 5,                     /*!< Group number for CPI status codes. */
    uStatusGroup_RTC = 6,                     /*!< Group number for RTC status codes. */
    uStatusGroup_NOTIFIER = 98,               /*!< Group number for NOTIFIER status codes. */
    uStatusGroup_DebugConsole = 99,           /*!< Group number for debug console status codes. */
    uStatusGroup_ApplicationRangeStart = 100, /*!< Starting number for application groups. */
};

/*! @brief Generic status return codes. */
enum _generic_status
{
    uStatus_Success = MAKE_STATUS(uStatusGroup_Generic, 0),
    uStatus_Fail = MAKE_STATUS(uStatusGroup_Generic, 1),
    uStatus_ReadOnly = MAKE_STATUS(uStatusGroup_Generic, 2),
    uStatus_OutOfRange = MAKE_STATUS(uStatusGroup_Generic, 3),
    uStatus_InvalidArgument = MAKE_STATUS(uStatusGroup_Generic, 4),
    uStatus_Timeout = MAKE_STATUS(uStatusGroup_Generic, 5),
    uStatus_NoTransferInProgress = MAKE_STATUS(uStatusGroup_Generic, 6),
};

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup util_functions
 * @{
 */

static inline uint16_t __core_NUMBERS() {
    return (SOC_CTRL->INFO & SOC_CTRL_INFO_CORE_NB_MASK) >> SOC_CTRL_INFO_CORE_NB_SHIFT;
}

static inline uint16_t __cluster_NUMBERS() {
    return (SOC_CTRL->INFO & SOC_CTRL_INFO_CLUSTER_NB_MASK) >> SOC_CTRL_INFO_CLUSTER_NB_SHIFT;
}

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /*_GAP_UTIL_H_*/
