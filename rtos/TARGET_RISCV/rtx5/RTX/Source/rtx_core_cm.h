/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * -----------------------------------------------------------------------------
 *
 * Project:     CMSIS-RTOS RTX
 * Title:       Cortex-M Core definitions
 *
 * -----------------------------------------------------------------------------
 */
/*
 Modifications copyright (C) 2018 GreenWaves Technologies

 - Delete and Change all CORTEX-M registers access function to support RISC-V registers access
 - Add extern void SysTick_Handler(); extern void PendSV_Handler();
 */

#ifndef RTX_CORE_CM_H_
#define RTX_CORE_CM_H_

#include <cmsis.h>

#ifdef RTE_CMSIS_RTOS2_RTX5_ARMV8M_NS
#define __DOMAIN_NS             1U
#endif

#ifndef __DOMAIN_NS
#define __DOMAIN_NS             0U
#elif ((__DOMAIN_NS          == 1U) && \
      ((__ARM_ARCH_6M__      == 1U) || \
       (__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U)))
#error "Non-secure domain requires ARMv8-M Architecture!"
#endif

#ifndef __EXCLUSIVE_ACCESS
#if   ((__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U) || \
       (__ARM_ARCH_8M_BASE__ == 1U) || \
       (__ARM_ARCH_8M_MAIN__ == 1U))
#define __EXCLUSIVE_ACCESS      1U
#else
#define __EXCLUSIVE_ACCESS      0U
#endif
#endif

#define IS_PRIVILEGED()          ((__get_CPRIV() & CPRIV_PRIV_Msk) != 0U)

#define IS_IRQ_MODE()            ((IS_PRIVILEGED() ? (__get_MCAUSE() & MCAUSE_IRQ_Msk) : \
                                   (__get_UCAUSE() & MCAUSE_IRQ_Msk)) != 0U)

#define IS_IRQ_TICK()            ((IS_PRIVILEGED() ? (__get_MCAUSE() & MCAUSE_EC_Msk) : \
                                   (__get_UCAUSE() & MCAUSE_EC_Msk)) == 10U)

#define IS_USER_IRQ_ENBALE()     ((__get_USTATUS() & MSTATUS_UIE_Msk) != 0U)

#define IS_MACHINE_IRQ_ENABLE()  ((__get_MSTATUS() & MSTATUS_MIE_Msk) != 0U)

#define IS_IRQ_MASKED()          ((__get_BASEPRI() != 0U))

#define IS_USER_IRQ_MASKED()     ((__get_BASEPRI() != 0U))

#define IS_MACHINE_IRQ_MASKED()  ((__get_BASEPRI() != 0U))

#define MSTATUS_INITIAL_VALUE      0x1880U
#define USTATUS_INITIAL_VALUE      0x0U

#define xPSR_INIT(...)          0x01000000U

#if    (__DOMAIN_NS == 1U)
#define STACK_FRAME_INIT        0xBCU
#else
#define STACK_FRAME_INIT        0xFDU
#endif

#if   (__RISCV_ARCH_GAP__   == 1U)

// Stack Frame:
//  - Basic:             S0-S11, A0-A7, (T0), RA, PC, U/MSTATUS
#define STACK_OFFSET_R0(stack_frame)      (12U *4U)

#else
// Stack Frame:
//  - Extended: S16-S31, R4-R11, R0-R3, R12, LR, PC, xPSR, S0-S15, FPSCR
//  - Basic:             R4-R11, R0-R3, R12, LR, PC, xPSR
#if (__FPU_USED == 1U)
#define STACK_OFFSET_R0(stack_frame)                 \
  ((((stack_frame) & 0x10U) == 0U) ? ((16U+8U)*4U) : \
                                          (8U *4U))
#else
#define STACK_OFFSET_R0(stack_frame)      (8U *4U)
#endif

#endif

#define OS_TICK_HANDLER         SysTick_Handler

extern void SysTick_Handler();
extern void PendSV_Handler();

//  ==== Service Calls definitions ====
#if   ((__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U) || \
       (__ARM_ARCH_8M_MAIN__ == 1U))
#define SVC_RegF "r12"
#elif ((__ARM_ARCH_6M__      == 1U) || \
       (__ARM_ARCH_8M_BASE__ == 1U))
#define SVC_RegF "r7"
#elif  (__RISCV_ARCH_GAP__  == 1U)
#define SVC_RegF "t0"
#endif

#define SVC_ArgN(n) \
register uint32_t __a##n __ASM("a"#n)

#define SVC_ArgR(n,a) \
register uint32_t __a##n __ASM("a"#n) = (uint32_t)a

#define SVC_ArgF(f) \
register uint32_t __rf __ASM(SVC_RegF) = (uint32_t)f

#define SVC_In0 "r"(__rf)
#define SVC_In1 "r"(__rf),"r"(__a0)
#define SVC_In2 "r"(__rf),"r"(__a0),"r"(__a1)
#define SVC_In3 "r"(__rf),"r"(__a0),"r"(__a1),"r"(__a2)
#define SVC_In4 "r"(__rf),"r"(__a0),"r"(__a1),"r"(__a2),"r"(__a3)

#define SVC_Out0
#define SVC_Out1 "=r"(__a0)
#define SVC_Out2 "=r"(__a0),"=r"(__a1)

#define SVC_CL0
#define SVC_CL1 "a1"
#define SVC_CL2 "a0","a1"

#define SVC_Call0M  SVC_Call0

#define SVC_Call0(in, out, cl)                                                 \
  __ASM volatile (                                                             \
    "add   t1, x0, x0\n\t"                                                     \
    "ecall\n"                                                                  \
  "1:": out : in : cl)


#define SVC0_0N(f,t)                                                           \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (void) {                                            \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In0, SVC_Out0, SVC_CL2);                                       \
}

#define SVC0_0(f,t)                                                            \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (void) {                                            \
  SVC_ArgN(0);                                                                 \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In0, SVC_Out1, SVC_CL1);                                       \
  return (t) __a0;                                                             \
}

#define SVC0_0M(f,t)                                                           \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (void) {                                            \
  SVC_ArgN(0);                                                                 \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0M(SVC_In0, SVC_Out1, SVC_CL1);                                      \
  return (t) __a0;                                                             \
}

#define SVC0_1N(f,t,t1)                                                        \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1) {                                           \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In1, SVC_Out0, SVC_CL1);                                       \
}

#define SVC0_1(f,t,t1)                                                         \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1) {                                           \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In1, SVC_Out1, SVC_CL1);                                       \
  return (t) __a0;                                                             \
}

#define SVC0_1M(f,t,t1)                                                        \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1) {                                           \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0M(SVC_In1, SVC_Out1, SVC_CL1);                                      \
  return (t) __a0;                                                             \
}

#define SVC0_2(f,t,t1,t2)                                                      \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2) {                                    \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In2, SVC_Out1, SVC_CL0);                                       \
  return (t) __a0;                                                             \
}

#define SVC0_2M(f,t,t1,t2)                                                     \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2) {                                    \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0M(SVC_In2, SVC_Out1, SVC_CL0);                                      \
  return (t) __a0;                                                             \
}

#define SVC0_3(f,t,t1,t2,t3)                                                   \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2, t3 a3) {                             \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgR(2,a3);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In3, SVC_Out1, SVC_CL0);                                       \
  return (t) __a0;                                                             \
}

#define SVC0_3M(f,t,t1,t2,t3)                                                  \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2, t3 a3) {                             \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgR(2,a3);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0M(SVC_In3, SVC_Out1, SVC_CL0);                                      \
  return (t) __a0;                                                             \
}

#define SVC0_4(f,t,t1,t2,t3,t4)                                                \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2, t3 a3, t4 a4) {                      \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgR(2,a3);                                                              \
  SVC_ArgR(3,a4);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0(SVC_In4, SVC_Out1, SVC_CL0);                                       \
  return (t) __a0;                                                             \
}

#define SVC0_4M(f,t,t1,t2,t3,t4)                                               \
__attribute__((always_inline))                                                 \
__STATIC_INLINE t __svc##f (t1 a1, t2 a2, t3 a3, t4 a4) {                      \
  SVC_ArgR(0,a1);                                                              \
  SVC_ArgR(1,a2);                                                              \
  SVC_ArgR(2,a3);                                                              \
  SVC_ArgR(3,a4);                                                              \
  SVC_ArgF(svcRtx##f);                                                         \
  SVC_Call0M(SVC_In4, SVC_Out1, SVC_CL0);                                      \
  return (t) __a0;                                                             \
}




//  ==== Core Peripherals functions ====

/// Initialize SVC and PendSV System Service Calls
__STATIC_INLINE void SVC_Initialize (void) {
    // SysTick and PendSV has the same priority in GAP8
    NVIC_SetVector(SysTick_IRQn, (uint32_t)SysTick_Handler);
    NVIC_SetVector(PendSV_IRQn, (uint32_t)PendSV_Handler);

    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_EnableIRQ(PendSV_IRQn);
}

/// Get Pending SV (Service Call) Flag
/// \return    Pending SV Flag
__STATIC_INLINE uint8_t GetPendSV (void) {
    //return ((uint8_t)((SCB->ICSR & (SCB_ICSR_PENDSVSET_Msk)) >> 24));
    return (EU_CORE_DEMUX->MASK & (1<<PendSV_IRQn));
}

/// Clear Pending SV (Service Call) Flag
__STATIC_INLINE void ClrPendSV (void) {
    //SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    EU_CORE_DEMUX->MASK_IRQ_AND = (1<<PendSV_IRQn);
}

/// Set Pending SV (Service Call) Flag
__STATIC_INLINE void SetPendSV (void) {
    //SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    //rtx_eu_irq_maskSet(1<<PendSV_FC_SW_IRQ);
    EU_SW_EVENTS->TRIGGER_SET[PendSV_IRQn] = 1;
}

/// Set Pending Flags
/// \param[in] flags  Flags to set
__STATIC_INLINE void SetPendFlags (uint8_t flags) {
  //SCB->ICSR = ((uint32_t)flags << 24);
}


//  ==== Exclusive Access Operation ====

#if (__EXCLUSIVE_ACCESS == 1U)

/// Atomic Access Operation: Write (8-bit)
/// \param[in]  mem             Memory address
/// \param[in]  val             Value to write
/// \return                     Previous value
__STATIC_INLINE uint8_t atomic_wr8 (uint8_t *mem, uint8_t val) {
  register uint8_t  ret;

  __ASM volatile (
  "1:\n\t"
    "lb %[ret],(%[mem])\n\t"
    "sb %[val],(%[mem])\n"
  "2:"
  : [ret] "=&l" (ret)
  : [mem] "l"   (mem),
    [val] "l"   (val)
  : "memory"
  );

  return ret;
}


/// Atomic Access Operation: Set bits (32-bit)
/// \param[in]  mem             Memory address
/// \param[in]  bits            Bit mask
/// \return                     New value
__STATIC_INLINE uint32_t atomic_set32 (uint32_t *mem, uint32_t bits) {

  register uint32_t val;

  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw    %[val],(%[mem])\n\t"
    "or    %[ret],%[val],%[bits]\n\t"
    "sw    %[ret],(%[mem])\n"
  "2:"
  : [ret]  "=&l" (ret),
    [val]  "=&l" (val)
  : [mem]  "l"   (mem),
    [bits] "l"   (bits)
  : "memory"
  );

  return ret;
}


/// Atomic Access Operation: Clear bits (32-bit)
/// \param[in]  mem             Memory address
/// \param[in]  bits            Bit mask
/// \return                     Previous value
__STATIC_INLINE uint32_t atomic_clr32 (uint32_t *mem, uint32_t bits) {

  register uint32_t val;
  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw %[ret],(%[mem])\n\t"
    "xori   %[val],%[bits], 0xffffffff\n\t"
    "and    %[val],%[ret],%[val]\n\t"
    "sw    %[val],(%[mem])\n"
  "2:"
  : [ret]  "=&l" (ret),
    [val]  "=&l" (val)
  : [mem]  "l"   (mem),
    [bits] "l"   (bits)
  : "memory"
  );

  return ret;
}


/// Atomic Access Operation: Check if all specified bits (32-bit) are active and clear them
/// \param[in]  mem             Memory address
/// \param[in]  bits            Bit mask
/// \return                     Active bits before clearing or 0 if not active
__STATIC_INLINE uint32_t atomic_chk32_all (uint32_t *mem, uint32_t bits) {
  register uint32_t val;
  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw    %[ret],(%[mem])\n\t"
    "and   %[val],%[ret],%[bits]\n\t"
    "beq   %[val],%[bits], 2f\n\t"
    "li    %[ret],#0\n\t"
    "j     3f\n"
  "2:\n\t"
    "xori   %[val],%[bits], 0xffffffff\n\t"
    "and    %[val],%[ret],%[val]\n\t"
    "sw     %[val],(%[mem])\n"
  "3:"
  : [ret]  "=&l" (ret),
    [val]  "=&l" (val)
  : [mem]  "l"   (mem),
    [bits] "l"   (bits)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Check if any specified bits (32-bit) are active and clear them
/// \param[in]  mem             Memory address
/// \param[in]  bits            Bit mask
/// \return                     Active bits before clearing or 0 if not active
__STATIC_INLINE uint32_t atomic_chk32_any (uint32_t *mem, uint32_t bits) {
  register uint32_t val;
  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw    %[ret],[%[mem]]\n\t"
    "and   %[val],%[ret],%[bits]\n\t"
    "bne   %[val], x0, 2f\n\t"
    "li  %[ret],#0\n\t"
    "j     3f\n"
  "2:\n\t"
    "xori   %[val],%[bits], 0xffffffff\n\t"
    "and    %[val],%[ret],%[val]\n\t"
    "sw    %[val],(%[mem])\n"
  "3:"
  : [ret]  "=&l" (ret),
    [val]  "=&l" (val)
  : [mem]  "l"   (mem),
    [bits] "l"   (bits)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Increment (32-bit)
/// \param[in]  mem             Memory address
/// \return                     Previous value
__STATIC_INLINE uint32_t atomic_inc32 (uint32_t *mem) {
  register uint32_t val;
  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw %[ret],(%[mem])\n\t"
    "addi  %[val],%[ret],0x1\n\t"
    "sw    %[val],(%[mem])\n"
  "2:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val)
  : [mem] "l"   (mem)
  : "cc", "memory"
  );

  return ret;
}

/// atomic Access Operation: Increment (32-bit) if Less Than
/// \param[in]  mem             Memory address
/// \param[in]  max             Maximum value
/// \return                     Previous value
__STATIC_INLINE uint32_t atomic_inc32_lt (uint32_t *mem, uint32_t max) {
  register uint32_t val, res;
  register uint32_t ret;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n\t"
#endif
  "1:\n\t"
    "ldrex %[ret],[%[mem]]\n\t"
    "cmp   %[max],%[ret]\n\t"
    "bhi    2f\n\t"
    "clrex\n\t"
    "b      3f\n"
  "2:\n\t"
    "adds  %[val],%[ret],#1\n\t"
    "strex %[res],%[val],[%[mem]]\n\t"
    "cbz   %[res],3f\n\t"
    "b     1b\n"
  "3:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val),
    [res] "=&l" (res)
  : [mem] "l"   (mem),
    [max] "l"   (max)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Increment (16-bit) if Less Than
/// \param[in]  mem             Memory address
/// \param[in]  max             Maximum value
/// \return                     Previous value
__STATIC_INLINE uint16_t atomic_inc16_lt (uint16_t *mem, uint16_t max) {
  register uint32_t val;
  register uint16_t ret;

  __ASM volatile (
  "1:\n\t"
    "lh     %[ret],(%[mem])\n\t"
    "bge    %[max],%[ret], 2f\n\t"
    "j      3f\n"
  "2:\n\t"
    "addi   %[val],%[ret], 0x1\n\t"
    "sh     %[val],(%[mem])\n\t"
    "j      1b\n"
  "3:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val)
  : [mem] "l"   (mem),
    [max] "l"   (max)
  : "cc", "memory"
  );

  return ret;
}

/// Atomic Access Operation: Increment (16-bit) and clear on Limit
/// \param[in]  mem             Memory address
/// \param[in]  max             Maximum value
/// \return                     Previous value
__STATIC_INLINE uint16_t atomic_inc16_lim (uint16_t *mem, uint16_t lim) {
  register uint32_t val;
  register uint16_t ret;

  __ASM volatile (
  "1:\n\t"
    "lh     %[ret],(%[mem])\n\t"
    "addi   %[val],%[ret],0x1\n\t"
    "bge    %[val],%[lim], 2f\n\t"
    "li     %[val], 0x0\n"
  "2:\n\t"
    "sh     %[val],(%[mem])\n"
  "3:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val)
  : [mem] "l"   (mem),
    [lim] "l"   (lim)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Decrement (32-bit) if Not Zero
/// \param[in]  mem             Memory address
/// \return                     Previous value
__STATIC_INLINE uint32_t atomic_dec32_nz (uint32_t *mem) {

  register uint32_t val;

  register uint32_t ret;

  __ASM volatile (
  "1:\n\t"
    "lw    %[ret],(%[mem])\n\t"
    "bne   %[ret], x0, 2f\n\t"
    "j     3f\n"
  "2:\n\t"
    "addi  %[val],%[ret], -1\n\t"
    "sh    %[val],(%[mem])\n"
  "3:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val)
  : [mem] "l"   (mem)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Decrement (16-bit) if Not Zero
/// \param[in]  mem             Memory address
/// \return                     Previous value
__STATIC_INLINE uint16_t atomic_dec16_nz (uint16_t *mem) {

  register uint32_t val;

  register uint16_t ret;

  __ASM volatile (
  "1:\n\t"
    "lh     %[ret],(%[mem])\n\t"
    "bne    %[ret], x0, 2f\n\t"
    "j      3f\n"
  "2:\n\t"
    "addi   %[val],%[ret], -1\n\t"
    "sw     %[val],(%[mem])\n"
  "3:"
  : [ret] "=&l" (ret),
    [val] "=&l" (val)
  : [mem] "l"   (mem)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Link Get
/// \param[in]  root            Root address
/// \return                     Link
__STATIC_INLINE void *atomic_link_get (void **root) {
  register uint32_t val, res;
  register void    *ret;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n\t"
#endif
  "1:\n\t"
    "ldrex %[ret],[%[root]]\n\t"
    "cbnz  %[ret],2f\n\t"
    "clrex\n\t"
    "b     3f\n"
  "2:\n\t"
    "ldr   %[val],[%[ret]]\n\t"
    "strex %[res],%[val],[%[root]]\n\t"
    "cbz   %[res],3f\n\t"
    "b     1b\n"
  "3:"
  : [ret]  "=&l" (ret),
    [val]  "=&l" (val),
    [res]  "=&l" (res)
  : [root] "l"   (root)
  : "cc", "memory"
  );

  return ret;
}


/// Atomic Access Operation: Link Put
/// \param[in]  root            Root address
/// \param[in]  lnk             Link
__STATIC_INLINE void atomic_link_put (void **root, void *link) {
  register uint32_t val1, val2, res;

  __ASM volatile (
#ifndef __ICCARM__
  ".syntax unified\n\t"
#endif
  "1:\n\t"
    "ldr   %[val1],[%[root]]\n\t"
    "str   %[val1],[%[link]]\n\t"
    "dmb\n\t"
    "ldrex %[val1],[%[root]]\n\t"
    "ldr   %[val2],[%[link]]\n\t"
    "cmp   %[val2],%[val1]\n\t"
    "bne   1b\n\t"
    "strex %[res],%[link],[%[root]]\n\t"
    "cbz   %[res],2f\n\t"
    "b     1b\n"
  "2:"
  : [val1] "=&l" (val1),
    [val2] "=&l" (val2),
    [res]  "=&l" (res)
  : [root] "l"   (root),
    [link] "l"   (link)
  : "cc", "memory"
  );
}

#endif  // (__EXCLUSIVE_ACCESS == 1U)

#endif  // RTX_CORE_CM_H_
