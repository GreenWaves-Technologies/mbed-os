/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OSC32KCLK = 0,
} RTCName;

typedef enum {
    UART_0 = 0,
} UARTName;

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_0

typedef enum {
    I2C_0 = 0,
    I2C_1 = 1,
} I2CName;

typedef enum {
    I2S_0 = 0,
    I2S_1 = 1,
} I2SName;

#define TPM_SHIFT   8
typedef enum {
    PWM_0  = 0,
    PWM_1  = 1,
    PWM_2  = 2,
    PWM_3  = 3,
} PWMName;

typedef enum {
    SPI_0 = 0,
    SPI_1 = 1,
} SPIName;

typedef enum {
    SPIS_0 = 0,
} SPISName;

typedef enum {
    HYPERBUS_0 = 0,
} HYPERBUSName;

typedef enum {
    ORCA_0 = 0,
} ORCAName;

typedef enum {
    LVDS_0 = 0,
} LVDSName;

typedef enum {
    CPI_0 = 0,
} CPIName;

#ifdef __cplusplus
}
#endif

#endif
