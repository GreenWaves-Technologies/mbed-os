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

#ifndef _GAP_MEMCPY_H_
#define _GAP_MEMCPY_H_

#include "GAP8.h"
#include "gap_udma.h"

/*!
 * @addtogroup memcopy
 * @{
 */

/*******************************************************************************
 * Variables, macros, structures,... definitions
 ******************************************************************************/

/*! @brief Memcopy module status. */
typedef enum
{
    uMEMCPY_Idle  = 0x0,  /*!< Memcopy is idle. */
    uMEMCPY_Error = 0x1,  /*!< Error during transfer. */
    uMEMCPY_Busy  = 0x2   /*!< Memcopy is busy with a transfer. */
} memcpy_status_t;

/*! @brief Memcopy type of transfer. */
typedef enum
{
    uMEMCPY_L22L2 = 0x0, /*!< Transfer from L2 to L2. */
    uMEMCPY_FC2L2 = 0x1, /*!< Transfer from FC to L2. */
    uMEMCPY_L22FC = 0x2  /*!< Transfer from L2 to FC. */
} memcpy_sel_t;

/*!
 * @brief Completion callback function pointer type.
 *
 * When an asynchronous is completed, the handler calls this callback function.
 *
 * @param userData  Parameter passed to the callback function by the user.
 */
typedef void (*memcpy_callback_t)(void *useData);

/*!
 * @brief Memcopy handler structure.
 *
 * This structure holds information to handle events from UDMA upon asynchronous transfers completion.
 * When asynchronous transfers are used, this structure should be filled.
 */
typedef struct _memcpy_handle_t
{
    memcpy_status_t state;
    memcpy_callback_t callback;
    void *userData;
} memcpy_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name Memcopy module configuration.
 * @{
 */

/*!
 * @brief Initialize the MEMCPY module.
 *
 * This function intializes the Memcopy module for transfers between FC_TCDM/L2 and L2.
 *
 * @param base         MEMCPY base pointer.
 */
void MEMCPY_Init(MEMCPY_Type *base);

/*!
 * @brief Release the MEMCPY module.
 *
 * @param base         MEMCPY base pointer.
 */
void MEMCPY_Deinit(MEMCPY_Type *base);

/* @} */

/*!
 * @name Synchronous operation(blocking function).
 * @{
 */

/*!
 * @brief Blocking transfer.
 *
 * This function initiates a blocking transfer between FC_TCDM/L2 and L2.
 *
 * @param base         MEMCPY base pointer.
 * @param src_addr     Source buffer.
 * @param dst_addr     Dest buffer.
 * @param size         Size of data to transfer.
 * @param direction    Direction of the transfer.
 *
 * @return Return uStatus_Success if the read operation is successful, an error otherwise.
 */
status_t MEMCPY_BlockingTransfer(MEMCPY_Type *base, uint32_t *src_addr, uint32_t *dst_addr,
                                 uint32_t size, memcpy_sel_t direction);

/* @} */

/*!
 * @name Asynchronous operation(non blocking function).
 * @{
 */

/*!
 * @brief Non blocking transfer.
 *
 * This function is used for non blocking transactions using UDMA.
 * Once the UDMA, called for the transfer operations, is configured, this function returns.
 *
 * @param base         MEMCPY base pointer.
 * @param src_addr     Source buffer.
 * @param dst_addr     Dest buffer.
 * @param size         Size of data to transfer.
 * @param direction    Direction of the transfer.
 * @param handle       Pointer to memcpy_handle_t structure.
 *
 * @return status_t.
 */
status_t MEMCPY_NonBlockingTransfer(MEMCPY_Type *base, uint32_t *src_addr, uint32_t *dst_addr,
                                    uint32_t size, memcpy_sel_t direction, memcpy_handle_t *handle);

/* @} */

/*!
 * @name IRQ Handler.
 * @{
 */

/*!
 * @brief Initialize the MEMCPY IRQ Handler.
 *
 * This function creates a IRQ handler for MEMCPY non blocking operations.
 * The callback function passed to this function is called when transaction is done.
 *
 * @param handle       Pointer to memcpy_handle_t structure.
 * @param callback     Callback function.
 * @param userData     Parameter passed to the callback function.
 */
void MEMCPY_CreateHandler(memcpy_handle_t *handle, memcpy_callback_t callback, void *userData);

/*!
 * @brief MEMCPY IRQ Handler.
 *
 * This function is called when a non blocking transfer is completed.
 * When called, the callback function previously defined is executed.
 *
 * @param arg          Callback function.
 */
void MEMCPY_IRQHandler(void *arg);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_MEMCPY_H_*/
