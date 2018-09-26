        .equ     I_T_RUN_OFS, 20
        .equ     TCB_SP_OFS,  56
        .equ     TCB_SF_OFS,  34

.section .text

  .global Ecall_Handler
#ifndef MBED_CONF_RTOS_PRESENT
Ecall_Handler:
        mret
#else
Ecall_Handler:
        addi sp, sp, -18*4
        sw   ra,  15*4(sp)                   // Save ra here
        jal  Exception_handler

        jalr t0  // Call service function

        sw   a0, 0*4(sp)      // Store function return values
        sw   a1, 1*4(sp)      // Store function return values


  .global SVC_Context
SVC_Context:
        la  t2, osRtxInfo+I_T_RUN_OFS     // Load address of osRtxInfo.run
        lw  t1, (t2)                      // Load osRtxInfo.thread.run: curr & next
        lw  t2, 4(t2)
        beq t1, t2, SVC_Exit
        beq t1, x0, SVC_ContextSwitch
                                          // Check if thread switch is required
                                          // Exit when threads are the same
                                          // Branch if running thread is not deleted

SVC_ContextSave:
        addi  a0,  sp, -12*4

        sw    s0,   0*4(a0)
        sw    s1,   1*4(a0)
        sw    s2,   2*4(a0)
        sw    s3,   3*4(a0)
        sw    s4,   4*4(a0)
        sw    s5,   5*4(a0)
        sw    s6,   6*4(a0)
        sw    s7,   7*4(a0)
        sw    s8,   8*4(a0)
        sw    s9,   9*4(a0)
        sw    s10, 10*4(a0)
        sw    s11, 11*4(a0)

        sw    a0,  TCB_SP_OFS(t1) // Store SP
        sb    t0,  TCB_SF_OFS(t1) // Store stack frame information


SVC_ContextSwitch:
        sw  t2, (t0)                       // osRtxInfo.thread.run: curr = next

SVC_ContextRestore:
        lb    t0,  TCB_SF_OFS(t2) // Load stack frame information
        lw    a0,  TCB_SP_OFS(t2) // Load SP

        lw    s0,   0*4(a0)
        lw    s1,   1*4(a0)
        lw    s2,   2*4(a0)
        lw    s3,   3*4(a0)
        lw    s4,   4*4(a0)
        lw    s5,   5*4(a0)
        lw    s6,   6*4(a0)
        lw    s7,   7*4(a0)
        lw    s8,   8*4(a0)
        lw    s9,   9*4(a0)
        lw    s10, 10*4(a0)
        lw    s11, 11*4(a0)

        addi  sp,  a0, 12*4
        sw    sp, %tiny(userStack)(x0)

        .global SVC_Exit
SVC_Exit:
        lw     a0,  16*4(sp)    //mepc
        csrw   0x341, a0

        //lw    a0,  17*4(sp) // mstatus
        //csrw   0x300, a0
        lw    a0,  0*4(sp)
        lw    a1,  1*4(sp)
        lw    a2,  2*4(sp)
        lw    a3,  3*4(sp)
        lw    a4,  4*4(sp)
        lw    a5,  5*4(sp)
        lw    a6,  6*4(sp)
        lw    a7,  7*4(sp)
        lw    t0,  8*4(sp)
        lw    t1,  9*4(sp)
        lw    t2,  10*4(sp)
        lw    t3,  11*4(sp)
        lw    t4,  12*4(sp)
        lw    t5,  13*4(sp)
        lw    t6,  14*4(sp)

        lw    ra, %tiny(currentStack)(x0)
        bne   ra, x0, .LC1

        lw    ra,  15*4(sp)
        addi  sp, sp, 18*4
        sw    sp, %tiny(kernelStack)(x0) // Return from Machine mode
        mret
.LC1:
        lw    ra,  15*4(sp)
        addi  sp, sp, 18*4
        sw    sp, %tiny(userStack)(x0)   // Return from User mode
        mret
#endif

       .global Exception_handler
Exception_handler:                       // exception handler should called for each exception and interruption
        sw    a0,  0*4(sp)

        lw    a0, %tiny(currentStack)(x0)
        bne   a0, x0, .LC2
        sw    sp, %tiny(kernelStack)(x0) // Use kernel SP
.LC2:
        sw    sp, %tiny(userStack)(x0)   // Use User SP

        lw    a0,  0*4(sp)
        sw    a1,  1*4(sp)
        sw    a2,  2*4(sp)
        sw    a3,  3*4(sp)
        sw    a4,  4*4(sp)
        sw    a5,  5*4(sp)
        sw    a6,  6*4(sp)
        sw    a7,  7*4(sp)

        sw    t0,  8*4(sp)
        sw    t1,  9*4(sp)
        sw    t2,  10*4(sp)
        sw    t3,  11*4(sp)
        sw    t4,  12*4(sp)
        sw    t5,  13*4(sp)
        sw    t6,  14*4(sp)
        // sw    ra,  15*4(sp)

        csrr  t2,  0x341
        csrr  t3,  0x342
        srli  t3,  t3, 31
        bnez  t3,  .LC3
        addi  t2,  t2, 4   // Exception mepc + 4
.LC3:
        sw    t2,  16*4(sp) // mepc
        //csrr  t2,  0x300
        //sw    t2,  17*4(sp) // mstatus
        ret


#ifdef MBED_CONF_RTOS_PRESENT
       .global PendSV_Handler
PendSV_Handler:
        addi sp, sp, -18*4
        sw   ra, 15*4(sp)                   // Save ra here
        jal  Exception_handler

        jal  osRtxPendSV_Handler           // Call osRtxPendSV_Handler
        j    SVC_Context



       .global SysTick_Handler
SysTick_Handler:
        addi sp, sp, -18*4
        sw   ra, 15*4(sp)                   // Save ra here
        jal  Exception_handler

#ifdef MBED_CONF_RTOS_PRESENT
        la   t2, us_ticker_overflow_delta  // us ticker increment
        lw   t1, (t2)
        addi t1, t1, 1
        sw   t1, (t2)
#endif
        jal  osRtxTick_Handler           // Call osRtxTick_Handler

        j    SVC_Context
#endif
