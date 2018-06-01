/*
 * Copyright (C) 2015 ETH Zurich and University of Bologna
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Authors: Germain Haugou (germain.haugou@gmail.com)
 */
#include <stdarg.h>
#include "gap_common.h"
#include "rtx_core_c.h"
#include "tinyprintf.h"

#ifdef USE_UART
extern uint8_t uart_is_init;
#endif

extern void tfp_format(void *putp, putcf putf, const char *fmt, va_list va);
extern void exit(int code);

debug_struct_t HAL_DEBUG_STRUCT_NAME = GAP_DEBUG_STRUCT_INIT;

#ifdef FEATURE_CLUSTER
GAP_L1_GLOBAL_DATA volatile uint32_t  printf_lock = 0;
#endif

static int _io_lock() {
    /* user code must know if the cluster is on or off, we'll take care of that later on */
    if (__is_U_Mode()) return 0;
    int irq = 0;
#ifdef FEATURE_CLUSTER
    if(!__is_FC()) {
        int needLock = 1;
        if (!cluster_is_on)
            needLock = 0;

        if (needLock) {
            irq = __disable_irq();
            while (GAP_SWMutex_TryLock((uint32_t)&printf_lock)) {
                __restore_irq(irq);
                irq = __disable_irq();
            }
        }
    }
#endif

    return irq;
}

static void _io_unlock(int irq) {
    if (__is_U_Mode()) return;

#ifdef FEATURE_CLUSTER
    if(!__is_FC()) {
        int needLock = 1;
        if (!cluster_is_on)
            needLock = 0;

        if (needLock) {
            GAP_SWMutex_UnLock((uint32_t)&printf_lock);
            __restore_irq(irq);
        }
    }
#endif
}

int putchar(int c);

__attribute__((always_inline))
__STATIC_INLINE char __svcPutChar (char c) {
  SVC_ArgR(0,c);
  SVC_ArgF(putchar);
  SVC_Call0(SVC_In1, SVC_Out1, SVC_CL1);
  return (char) __a0;
}

char osPutChar(char c) {
  if (IsIrqMode() || IsIrqMasked()) {
     return -1 ;
  }
  return __svcPutChar(c);
}

static void tfp_putc(void *data, char c) {
    if (__is_U_Mode()) {
        osPutChar(c);
    } else {
        #ifdef USE_UART
        {
            if (!uart_is_init) {
                uart_config_t config;

                UART_GetDefaultConfig(&config);
                config.baudRate_Bps = 9600;
                config.enableTx = true;
                config.enableRx = true;

                UART_Init(UART, &config, SystemCoreClock);
            }
            UART_WriteByte(UART, (uint8_t)c);
        }
        #endif

        if (DEBUG_GetDebugStruct()->useInternalPrintf) {
            if(__cluster_ID() == FC_CLUSTER_ID) {
                FC_STDOUT->PUTC[__core_ID() << 1] = c;
            }
            #ifdef FEATURE_CLUSTER
            else {
                CLUSTER_STDOUT->PUTC[__core_ID() << 1] = c;
            }
            #endif
        } else {
            DEBUG_Putchar(DEBUG_GetDebugStruct(), c);
        }
    }
}

int GAP_EXPORT printf(const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    /* Only lock the printf if the cluster is up to avoid mixing FC and cluster output */
    int irq = _io_lock();
    tfp_format(NULL, tfp_putc, fmt, va);
    _io_unlock(irq);
    va_end(va);
    return 0;
}

void abort() {
    exit(-1);
}

int GAP_EXPORT puts(const char *s) {
    char c;
    int irq = _io_lock();
    do {
        c = *s;
        if (c == 0) {
            tfp_putc(NULL, '\n');
            break;
        }
        tfp_putc(NULL, c);
        s++;
    } while(1);
    _io_unlock(irq);
    return 0;
}

int putchar(int c) {
    tfp_putc(NULL, c);
    return c;
}
