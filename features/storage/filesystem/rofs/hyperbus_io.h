/******************************************************************************************
 *
 * Copyright (c) 2018 , GreenWaves Technologies, Inc.
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
 *******************************************************************************************/

#ifndef _GAP_HYPERBUS_IO_H_
#define _GAP_HYPERBUS_IO_H_

#include "stdlib.h"
#include "stdarg.h"
#include "gap_common.h"
#include "pinmap.h"
#include "PeripheralPins.h"

/*
 * HYPERFLASH S26KL256S
 *
 * ERASE SECTOR = 256kB
 * NB_SECTOR = TOTAL / 256kB = 128
 *
 * ADDRESS :
 * 0x00000000 - 0x0001FFFF
 * ----
 * 0xFE000000 - 0x00FFFFFF
 *
 * Line Buffer  = 512B
 * NB_LINE/SECTOR = 256kB / 512B = 512
 *
 * Half-Page    = 16B
 * NB_HALF/SECTOR = 256kB / 16B / 2 = 8192
 *
 * 1 word = 2B
 */

/*!
 * @addtogroup hyperbus_IO
 * @{
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Error codes. */
#define HYPERBUS_IO_OK        0x0
#define HYPERBUS_IO_ERROR     0x1
#define HYPERBUS_IO_BUSY      0x2

#define DEVICE_READY_OFFSET     7

/* Write and read address */
#define SA                      0x0000

typedef struct {
    uint16_t data;
    uint16_t addr;
} cmdSeq;

/* Sector erase sequence */
static GAP_L2_DATA cmdSeq Erase_Seq[6] = {{0xAA, 0x555}, {0x55, 0x2AA}, {0x80, 0x555},
                                          {0xAA, 0x555}, {0x55, 0x2AA}, {0x30, SA}};

/* Configure register0 sequence */
static GAP_L2_DATA cmdSeq VCR_Seq[4]   = {{0xAA, 0x555}, {0x55, 0x2AA}, {0x38, 0x555}, {0x8e0b, 0x0}};

/* Read status register sequence */
static GAP_L2_DATA cmdSeq Reg_Seq      = {0x70, 0x555};

/* Write 512/4 = 128 word to Sector addr 0x4xxx */
static GAP_L2_DATA cmdSeq WP_Seq[3]    = {{0xAA, 0x555}, {0x55, 0x2AA}, {0xA0, 0x555}};

static GAP_L2_DATA uint32_t read_val   = 0;
static GAP_L2_DATA uint32_t write_val  = 0;

static hyperbus_transfer_t masterXfer;

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Initialize Hyperbus used to access HyperFlash or HyperRam.
 *
 * @param device  Type of device to configure : uHYPERBUS_Flash or uHYPERBUS_Ram.
 *
 * @return HYPERBUS_IO_OK if Huperbus initialized, else HYPERBUS_IO_ERROR.
 */
int32_t HYPERBUS_IO_Init( uint8_t device );

/*!
 * @brief Free Hyperbus instance.
 *
 */
void HYPERBUS_IO_Deinit( void );

/*!
 * @brief Erase elements of HyperFlash.
 *
 * @param addr   Address to start erasing.
 *
 * @return HYPERBUS_IO_OK if successfully erase, else HYPERBUS_IO_ERROR.
 */
int32_t HYPERBUS_IO_Erase( uint32_t addr );

/*!
 * @brief Write a buffer to the HYPERBUS Flash.
 *
 * @param addr           Address to write in the device.
 * @param size           Size of the buffer to write, in Bytes.
 * @param buf            Pointer to the buffer to output.
 * @param device         Device accessed.
 * @param device_access  Type of access, register or memory.
 *
 * @return HYPERBUS_IO_OK if successfully erase, else HYPERBUS_IO_ERROR.
 */
int32_t HYPERBUS_IO_Write( uint32_t addr, uint32_t size, void *buf, uint8_t device, uint8_t device_access );

/*!
 * @brief Read a buffer to the HYPERBUS Flash.
 *
 * @param addr           Address to read from the device.
 * @param size           Size of the buffer to read, in Bytes.
 * @param buf            Pointer to the buffer to input.
 * @param device         Device accessed.
 * @param device_access  Type of access, register or memory.
 *
 * @return HYPERBUS_IO_OK if successfully erase, else HYPERBUS_IO_ERROR.
 */
int32_t HYPERBUS_IO_Read( uint32_t addr, uint32_t size, void *buf, uint8_t device, uint8_t device_access );

/*!
 * @brief Synchronise Page Programming/Erasing.
 *
 */
int32_t HYPERBUS_IO_Sync( void );

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /*_GAP_HYPERBUS_IO_H_*/
