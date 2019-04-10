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
#include <math.h>
#include "mbed_assert.h"

#if DEVICE_QSPI

#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "PeripheralPins.h"
#include "spi_multi_api.h"
#include "qspi_api.h"


/* Array of SPIM peripheral base address. */
static SPIM_Type *const qspi_address[] = QSPIM_BASE_PTRS;

/* SPI master global configuration */
static spi_master_config_t master_config;

qspi_status_t qspi_prepare_command(qspi_t *obj, const qspi_command_t *command, spi_command_sequence_t *seq,
                                   const void *tx_data, size_t tx_size, const void *rx_data, size_t rx_size)
{
    /* _inst_width = QSPI_CFG_BUS_SINGLE; */
    /* _address_width = QSPI_CFG_BUS_SINGLE; */
    /* _address_size = QSPI_CFG_ADDR_SIZE_24; */
    /* _alt_width = QSPI_CFG_BUS_SINGLE; */
    /* _alt_size = QSPI_CFG_ALT_SIZE_8; */
    /* _data_width = QSPI_CFG_BUS_SINGLE; */
    /* _num_dummy_cycles = 0; */
    /* _mode = mode; */
    /* _hz = ONE_MHZ; */
    seq->csn = master_config.whichCsn;

    if(command->instruction.disabled) {
        seq->cmd_bits  = 0;
    } else {
        seq->cmd_bits  = (command->instruction.bus_width + 1) << 3;
        seq->cmd       = command->instruction.value;
    }

    if(command->address.disabled) {
        seq->addr_bits  = 0;
    } else {
        seq->addr_bits  = (command->address.bus_width + 1) << 3;
        seq->addr       = command->address.value;
    }

    if(command->alt.disabled) {
        seq->alter_data  = 0;
    } else {
        seq->alter_data_bits  = (command->alt.bus_width + 1) << 3;
        seq->alter_data       = command->alt.value;
    }

    seq->dummy     = command->dummy_count;

    seq->cmd_mode  = uSPI_Quad;
    seq->addr_mode = uSPI_Quad;
    seq->data_mode = uSPI_Quad;

    if (tx_data) {
        seq->tx_bits   = tx_size * ((command->data.bus_width + 1) << 3);
        seq->tx_buffer = (uint8_t *)tx_data;
    }

    if (rx_data) {
        seq->rx_bits   = rx_size * ((command->data.bus_width + 1) << 3);
        seq->rx_buffer = (uint8_t *)rx_data;
    }

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_init(qspi_t *obj, PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName ssel, uint32_t hz, uint8_t mode)
{
    uint32_t qspi_io0 = pinmap_peripheral(io0, PinMap_SPI_MOSI);
    uint32_t qspi_io1 = pinmap_peripheral(io1, PinMap_SPI_MISO);
    uint32_t qspi_io2 = pinmap_peripheral(io2, PinMap_SPIQ_SDIO2);
    uint32_t qspi_io3 = pinmap_peripheral(io3, PinMap_SPIQ_SDIO3);

    uint32_t qspi_io01 = pinmap_merge(qspi_io0, qspi_io1);
    uint32_t qspi_io23 = pinmap_merge(qspi_io2, qspi_io3);
    uint32_t qspi_io   = pinmap_merge(qspi_io01, qspi_io23);
    uint32_t qspi_cntl = pinmap_merge(sclk, ssel);

    obj->instance = pinmap_merge(qspi_io, qspi_cntl);
    MBED_ASSERT((int)obj->instance != NC);
    MBED_ASSERT(ssel != NC);

    /* pin out the spi pins */
    pinmap_pinout(io0, PinMap_SPI_MOSI);
    pinmap_pinout(io1, PinMap_SPI_MISO);
    pinmap_pinout(io2, PinMap_SPIQ_SDIO2);
    pinmap_pinout(io3, PinMap_SPIQ_SDIO3);
    pinmap_pinout(sclk, PinMap_SPI_SCLK);
    pinmap_pinout(ssel, PinMap_SPI_SSEL);

    /* Get default Master config */
    SPI_MasterGetDefaultConfig(&master_config);

    /* determine the SPI to use */
    if(ssel == SPI0_CSN1)
        master_config.whichCsn = uSPI_csn1;
    else
        master_config.whichCsn = uSPI_csn0;

    /* Master config */
    master_config.cpol = (mode & 0x2) ? uSPI_ClockPolarityActiveLow : uSPI_ClockPolarityActiveHigh;
    master_config.cpha = (mode & 0x1) ? uSPI_ClockPhaseSecondEdge : uSPI_ClockPhaseFirstEdge;

    SPI_MasterInit(qspi_address[obj->instance], &master_config, SystemCoreClock);

    if (hz > SystemCoreClock) {
        return QSPI_STATUS_INVALID_PARAMETER;
    }

    master_config.baudRate = hz;

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_free(qspi_t *obj)
{
    SPI_MasterDeInit(qspi_address[obj->instance]);

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_frequency(qspi_t *obj, int hz)
{
    if ((uint32_t)hz > SystemCoreClock) {
        return QSPI_STATUS_INVALID_PARAMETER;
    }

    uint16_t clk_div = (SystemCoreClock >> 1) / hz;

    /* Configuration only clock frequence */
    master_config.clkDiv = clk_div;
    SPI_MasterFrequencyConfig(qspi_address[obj->instance],  &master_config);

    return QSPI_STATUS_OK;
}

qspi_status_t qspi_write(qspi_t *obj, const qspi_command_t *command, const void *data, size_t *length)
{
    spi_command_sequence_t seq;

    /* Transfer to GAP8 command sequence */
    qspi_status_t status = qspi_prepare_command(obj, command, &seq, data, *length, NULL, 0);

    if (status != QSPI_STATUS_OK) {
        return status;
    }

    int ret = SPI_MasterTransferCommandSequence(qspi_address[obj->instance], &seq);

    if (ret == uStatus_Success ) {
        return QSPI_STATUS_OK;
    } else {
        return QSPI_STATUS_ERROR;
    }
}

qspi_status_t qspi_read(qspi_t *obj, const qspi_command_t *command, void *data, size_t *length)
{
    spi_command_sequence_t seq;

    /* Transfer to GAP8 command sequence */
    qspi_status_t status = qspi_prepare_command(obj, command, &seq, NULL, 0, data, *length);

    if (status != QSPI_STATUS_OK) {
        return status;
    }

    int ret = SPI_MasterTransferCommandSequence(qspi_address[obj->instance], &seq);

    if (ret == uStatus_Success) {
        return QSPI_STATUS_OK;
    } else {
        return QSPI_STATUS_ERROR;
    }
}

qspi_status_t qspi_command_transfer(qspi_t *obj, const qspi_command_t *command, const void *tx_data, size_t tx_size, void *rx_data, size_t rx_size)
{
    spi_command_sequence_t seq;

    /* Transfer to GAP8 command sequence */
    qspi_status_t status = qspi_prepare_command(obj, command, &seq, tx_data, tx_size, rx_data, rx_size);

    if (status != QSPI_STATUS_OK) {
        return status;
    }

    int ret = SPI_MasterTransferCommandSequence(qspi_address[obj->instance], &seq);

    if (ret == uStatus_Success) {
        return QSPI_STATUS_OK;
    } else {
        return QSPI_STATUS_ERROR;
    }
}

#endif

/** @}*/
