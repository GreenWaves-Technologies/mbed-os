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
#include "hyperbus_io.h"

static void HYPERBUS_IO_Pin_Init( uint32_t nbArgs, ... )
{
    PORT_Type *const xPort_addrs[] = PORT_BASE_PTRS;
    port_pin_config_t config = { uPORT_PullUpEnable, uPORT_HighDriveStrength, uPORT_MuxAlt3 };
    PinName pin = 0;
    uint32_t port = 0, pin_nb = 0;

    /* pin out the hyperbus pins */
    va_list list;
    va_start( list, nbArgs );
    for( uint32_t i = 0; i< nbArgs; i++)
    {
        pin = va_arg( list, PinName );
        port = GET_GPIO_PORT( pin );
        pin_nb = GET_GPIO_PIN_NUM( pin );
        PORT_SetPinConfig( xPort_addrs[port], pin_nb, &config );
    }
    va_end( list );
    hyperbus_master_config_t  masterConfig;

    HYPERBUS_MasterGetDefaultConfig( &masterConfig );
    HYPERBUS_MasterInit( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterConfig, SystemCoreClock );
}

static void HYPERBUS_IO_Flash_Config( void )
{
    /* Set VCR to 5 delay cycles */
    for ( uint32_t i = 0; i < 4; i++ )
    {
        masterXfer.txData = &VCR_Seq[i].data;
        masterXfer.txDataSize = 2;
        masterXfer.rxData = 0;
        masterXfer.rxDataSize = 0;
        masterXfer.configFlags = 32;
        masterXfer.addr = ( VCR_Seq[i].addr << 1 );
        masterXfer.device = uHYPERBUS_Flash;
        masterXfer.reg_access = uHYPERBUS_Mem_Access;

        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );
    }
}

int32_t HYPERBUS_IO_Init( uint8_t device )
{
    uint32_t latency = 0;

    HYPERBUS_IO_Pin_Init( 13, HYPERBUS_DQ0, HYPERBUS_DQ1, HYPERBUS_DQ2, HYPERBUS_DQ3,
                   HYPERBUS_DQ4, HYPERBUS_DQ5, HYPERBUS_DQ6, HYPERBUS_DQ7,
                   HYPERBUS_CLK, HYPERBUS_CLKN, HYPERBUS_RWDS, HYPERBUS_CSN0, HYPERBUS_CSN1 );
    if( device == uHYPERBUS_Flash )
    {
        latency = 0;
        /* Config memory access timing for TX and RX*/
        HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 0, uHYPERBUS_Flash );
        HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 1, uHYPERBUS_Flash );
        HYPERBUS_IO_Flash_Config();
    }
    else
    {
        latency = 1;
        /* Config memory maximum transfer data length for TX and RX*/
        HYPERBUS_SetMaxLength( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 1, 0x1ff, 0, uHYPERBUS_Ram );
        HYPERBUS_SetMaxLength( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 1, 0x1ff, 1, uHYPERBUS_Ram );
        /* Config memory access timing for TX and RX*/
        HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 0, uHYPERBUS_Ram );
        HYPERBUS_SetTiming( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, 4, 4, 4, latency, 1, uHYPERBUS_Ram );
    }

    return 0;
}

void HYPERBUS_IO_Deinit( void )
{
    HYPERBUS_MasterDeInit( (HYPERBUS_Type *) HYPERBUS_BASE_PTRS );
}

int32_t HYPERBUS_IO_Erase( uint32_t addr )
{
    for ( uint32_t i = 0; i < 5; i++ )
    {
        masterXfer.txData = &Erase_Seq[i].data;
        masterXfer.txDataSize = 2;
        masterXfer.rxData = 0;
        masterXfer.rxDataSize = 0;
        masterXfer.configFlags = 32;
        masterXfer.addr = Erase_Seq[i].addr << 1;
        masterXfer.device = uHYPERBUS_Flash;
        masterXfer.reg_access = uHYPERBUS_Mem_Access;

        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );
    }

    masterXfer.txData = &Erase_Seq[5].data;
    masterXfer.txDataSize = sizeof( uint16_t );
    masterXfer.rxData = 0;
    masterXfer.rxDataSize = 0;
    masterXfer.configFlags = 32;
    masterXfer.addr = addr;
    masterXfer.device = uHYPERBUS_Flash;
    masterXfer.reg_access = uHYPERBUS_Mem_Access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );

    return 0;
}

int32_t HYPERBUS_IO_Write( uint32_t addr, uint32_t size, void *buf, uint8_t device, uint8_t device_access )
{
    if( device == uHYPERBUS_Flash )
    {
        /* Write to Buffer command sequence */
        for( uint32_t i = 0; i < 3; i++ )
        {
            masterXfer.txData = &WP_Seq[i].data;
            masterXfer.txDataSize = 2;
            masterXfer.rxData = 0;
            masterXfer.rxDataSize = 0;
            masterXfer.configFlags = 32;
            masterXfer.addr = WP_Seq[i].addr << 1;
            masterXfer.device = uHYPERBUS_Flash;
            masterXfer.reg_access = uHYPERBUS_Mem_Access;

            HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );
        }
    }
    /* Word Program */
    masterXfer.txData = buf;
    masterXfer.txDataSize = size;
    masterXfer.rxData = 0;
    masterXfer.rxDataSize = 0;
    masterXfer.configFlags = 32;
    masterXfer.addr = addr;
    masterXfer.device = device;
    masterXfer.reg_access = device_access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );
    return 0;
}

int32_t HYPERBUS_IO_Read( uint32_t addr, uint32_t size, void *buf, uint8_t device, uint8_t device_access )
{
    masterXfer.txData = 0;
    masterXfer.txDataSize = 0;
    masterXfer.rxData = buf;
    masterXfer.rxDataSize = size;
    masterXfer.configFlags = 32;
    masterXfer.addr = addr;
    masterXfer.device = device;
    masterXfer.reg_access = device_access;

    HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXfer );

    return 0;
}

int32_t HYPERBUS_IO_Sync( void )
{
    uint16_t reg;
    hyperbus_transfer_t masterXferWrite, masterXferRead;

    write_val = Reg_Seq.data;
    masterXferWrite.txData = ( uint16_t * ) &write_val;
    masterXferWrite.txDataSize = 2;
    masterXferWrite.rxData = 0;
    masterXferWrite.rxDataSize = 0;
    masterXferWrite.configFlags = 32;
    masterXferWrite.addr = Reg_Seq.addr << 1;
    masterXferWrite.device = uHYPERBUS_Flash;
    masterXferWrite.reg_access = uHYPERBUS_Mem_Access;

    masterXferRead.txData = 0;
    masterXferRead.txDataSize = 0;
    masterXferRead.rxData = ( uint16_t * ) &read_val;
    masterXferRead.rxDataSize = 2;
    masterXferRead.configFlags = 32;
    masterXferRead.addr = 0;
    masterXferRead.device = uHYPERBUS_Flash;
    masterXferRead.reg_access = uHYPERBUS_Mem_Access;
    /* Wait the end of process
     * Status Register (SR)
     * bit 4 -> program status bit, 0-success ; 1-failure
     * bit 5 -> erase status bit,   0-success ; 1-failure
     * bit 7 -> device ready bit,   0-busy    ; 1-ready
     */
    do
    {
        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXferWrite );
        HYPERBUS_MasterTransferBlocking( ( HYPERBUS_Type * ) HYPERBUS_BASE_PTRS, &masterXferRead );
        reg = ( ( read_val >> 16 ) & 0xffff );
    } while( !( reg & ( 1 << DEVICE_READY_OFFSET ) ) );

    return 0;
}

