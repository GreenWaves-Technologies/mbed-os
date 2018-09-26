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
#ifndef _GAP_EFUSE_H_
#define _GAP_EFUSE_H_

#include "cmsis.h"
#include "gap_common.h"

#define GAP_EFUSE_PLT_OTHER   0
#define GAP_EFUSE_PLT_FPGA    1
#define GAP_EFUSE_PLT_RTL     2
#define GAP_EFUSE_PLT_VP      3
#define GAP_EFUSE_PLT_CHIP    4


#define GAP_EFUSE_BOOT_OTHER    0
#define GAP_EFUSE_BOOT_SPI      1
#define GAP_EFUSE_BOOT_JTAG     2
#define GAP_EFUSE_BOOT_ROM      3
#define GAP_EFUSE_BOOT_PRELOAD  4
#define GAP_EFUSE_BOOT_HYPERBUS    5
#define GAP_EFUSE_BOOT_SPIM     6
#define GAP_EFUSE_BOOT_SPIM_QPI 7

/*!
 * @addtogroup efuse
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*
 *sets the values for the 3 diffrent timing intervals that are used by the IP
 * bits[ 9: 0] sets the short interval  default   5 cycles
 * bits[19:10] sets the medium interval default  50 cycles
 * bits[29:20] sets the long interval   default 400 cycles
 */
  static inline void EFUSE_ConfigTimings(uint32_t timervalues) {
    EFUSE_CTRL->CFG = timervalues;
  }

  static inline void EFUSE_StartRead() {
    EFUSE_CTRL->CMD = EFUSE_CTRL_CMD_READ;
  }

  static inline void EFUSE_StartProgram() {
    EFUSE_CTRL->CMD = EFUSE_CTRL_CMD_WRITE;
  }

  static inline void EFUSE_Sleep() {
    EFUSE_CTRL->CMD = EFUSE_CTRL_CMD_SLEEP;
  }

  static inline uint8_t EFUSE_GetInfo() {
    return (uint8_t) EFUSE_REGS->INFO;
  }

  static inline uint8_t EFUSE_GetInfo2() {
    return (uint8_t) EFUSE_REGS->INFO2;
  }

  static inline uint8_t EFUSE_GetPlatform(uint8_t infoValue) {
    return ((infoValue & EFUSE_INFO_PLT_MASK) /*>> EFUSE_INFO_PLT_SHIFT*/);
  }

  static inline uint8_t EFUSE_GetBootmode(uint8_t infoValue) {
    return ((infoValue & EFUSE_INFO_BOOT_MASK) >> EFUSE_INFO_BOOT_SHIFT);
  }

  static inline uint8_t EFUSE_GetEncrypted(uint8_t infoValue) {
    return ((infoValue & EFUSE_INFO_ENCRYPTED_MASK) >> EFUSE_INFO_ENCRYPTED_SHIFT);
  }

  static inline uint8_t EFUSE_GetAESKey(int word) {
    return (uint8_t) EFUSE_REGS->AES_KEY[word];
  }

  static inline uint8_t EFUSE_GetAESIv(int word) {
    return (uint8_t) EFUSE_REGS->AES_IV[word];
  }

  static inline uint8_t EFUSE_wait_GetXtal(uint8_t infoValue) {
    return ((infoValue & EFUSE_INFO_WAIT_XTAL_MASK) >> EFUSE_INFO_WAIT_XTAL_SHIFT);
  }

  static inline uint8_t EFUSE_wait_GetXtalDelta() {
    return ((uint8_t) EFUSE_REGS->WAIT_XTAL_DELTA_LSB) | (((uint8_t) EFUSE_REGS->WAIT_XTAL_DELTA_LSB) << 8);
  }

  static inline uint8_t EFUSE_wait_GetXtalMin() {
    return (uint8_t) EFUSE_REGS->WAIT_XTAL_MIN;
  }

  static inline uint8_t EFUSE_GetWaitXtalMax() {
    return (uint8_t) EFUSE_REGS->WAIT_XTAL_MAX;
  }
/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_EFUSE_H_*/
