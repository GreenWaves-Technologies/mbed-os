/* mbed Microcontroller Library
 * Copyright (c) 2006-2012 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MBED_RTX_CONF_H
#define MBED_RTX_CONF_H

#include "mbed_rtx.h"

/** Any access to RTX5 specific data structures used in common code should be wrapped in ifdef MBED_OS_BACKEND_RTX5 */
#define MBED_OS_BACKEND_RTX5

#if defined(MBED_CONF_APP_THREAD_STACK_SIZE)
#define OS_STACK_SIZE               MBED_CONF_APP_THREAD_STACK_SIZE
#else
#define OS_STACK_SIZE               MBED_CONF_RTOS_THREAD_STACK_SIZE
#endif

#ifdef MBED_CONF_APP_TIMER_THREAD_STACK_SIZE
#define OS_TIMER_THREAD_STACK_SIZE  MBED_CONF_APP_TIMER_THREAD_STACK_SIZE
#else
#define OS_TIMER_THREAD_STACK_SIZE  MBED_CONF_RTOS_TIMER_THREAD_STACK_SIZE
#endif

// Increase the idle thread stack size when tickless is enabled
#if defined(MBED_TICKLESS) && defined(LPTICKER_DELAY_TICKS) && (LPTICKER_DELAY_TICKS > 0)
#define EXTRA_IDLE_STACK MBED_CONF_RTOS_IDLE_THREAD_STACK_SIZE_TICKLESS_EXTRA
#else
#define EXTRA_IDLE_STACK 0
#endif
#ifdef MBED_CONF_APP_IDLE_THREAD_STACK_SIZE
#define OS_IDLE_THREAD_STACK_SIZE   (MBED_CONF_APP_IDLE_THREAD_STACK_SIZE + EXTRA_IDLE_STACK)
#else
#define OS_IDLE_THREAD_STACK_SIZE   (MBED_CONF_RTOS_IDLE_THREAD_STACK_SIZE + EXTRA_IDLE_STACK)
#endif

#define OS_DYNAMIC_MEM_SIZE         0

#if defined(OS_TICK_FREQ) && (OS_TICK_FREQ != 1000)
#error "OS Tickrate must be 1000 for system timing"
#endif

#if !defined(OS_STACK_WATERMARK) && (defined(MBED_STACK_STATS_ENABLED) || defined(MBED_ALL_STATS_ENABLED))
#define OS_STACK_WATERMARK          1
#endif

#if !defined(OS_STACK_WATERMARK) && defined(MBED_THREAD_STATS_ENABLED)
#define OS_STACK_WATERMARK          1
#endif


#define OS_IDLE_THREAD_TZ_MOD_ID     1
#define OS_TIMER_THREAD_TZ_MOD_ID    1


// Don't adopt default multi-thread support for ARM/ARMC6 toolchains from RTX code base.
// Provide Mbed-specific instead.
#define RTX_NO_MULTITHREAD_CLIB
// LIBSPACE default value set for ARMCC
#define OS_THREAD_LIBSPACE_NUM      4

#define OS_IDLE_THREAD_NAME         "idle_thread"
#define OS_TIMER_THREAD_NAME        "timer_thread"

/* Enable only the evr events we use in Mbed-OS to save flash space. */
//Following events are used by Mbed-OS, DO NOT disable them
//#define EVR_RTX_KERNEL_ERROR_DISABLE
//#define EVR_RTX_THREAD_ERROR_DISABLE
//#define EVR_RTX_THREAD_EXIT_DISABLE
//#define EVR_RTX_THREAD_TERMINATE_DISABLE
//#define EVR_RTX_TIMER_ERROR_DISABLE
//#define EVR_RTX_EVENT_FLAGS_ERROR_DISABLE
//#define EVR_RTX_MUTEX_ERROR_DISABLE
//#define EVR_RTX_SEMAPHORE_ERROR_DISABLE
//#define EVR_RTX_MEMORY_POOL_ERROR_DISABLE
//#define EVR_RTX_MESSAGE_QUEUE_ERROR_DISABLE

//Following events are NOT used by Mbed-OS, you may enable them if needed for debug purposes
#define EVR_RTX_MEMORY_INIT_DISABLE
#define EVR_RTX_MEMORY_ALLOC_DISABLE
#define EVR_RTX_MEMORY_FREE_DISABLE
#define EVR_RTX_MEMORY_BLOCK_INIT_DISABLE
#define EVR_RTX_MEMORY_BLOCK_ALLOC_DISABLE
#define EVR_RTX_MEMORY_BLOCK_FREE_DISABLE
#define EVR_RTX_KERNEL_INITIALIZE_DISABLE
#define EVR_RTX_KERNEL_INITIALIZED_DISABLE
#define EVR_RTX_KERNEL_GET_INFO_DISABLE
#define EVR_RTX_KERNEL_INFO_RETRIEVED_DISABLE
#define EVR_RTX_KERNEL_GET_STATE_DISABLE
#define EVR_RTX_KERNEL_START_DISABLE
#define EVR_RTX_KERNEL_STARTED_DISABLE
#define EVR_RTX_KERNEL_LOCK_DISABLE
#define EVR_RTX_KERNEL_LOCKED_DISABLE
#define EVR_RTX_KERNEL_UNLOCK_DISABLE
#define EVR_RTX_KERNEL_UNLOCKED_DISABLE
#define EVR_RTX_KERNEL_RESTORE_LOCK_DISABLE
#define EVR_RTX_KERNEL_LOCK_RESTORED_DISABLE
#define EVR_RTX_KERNEL_SUSPEND_DISABLE
#define EVR_RTX_KERNEL_SUSPENDED_DISABLE
#define EVR_RTX_KERNEL_RESUME_DISABLE
#define EVR_RTX_KERNEL_RESUMED_DISABLE
#define EVR_RTX_KERNEL_GET_TICK_COUNT_DISABLE
#define EVR_RTX_KERNEL_GET_TICK_FREQ_DISABLE
#define EVR_RTX_KERNEL_GET_SYS_TIMER_COUNT_DISABLE
#define EVR_RTX_KERNEL_GET_SYS_TIMER_FREQ_DISABLE
#define EVR_RTX_THREAD_NEW_DISABLE
#define EVR_RTX_THREAD_CREATED_DISABLE
#define EVR_RTX_THREAD_GET_NAME_DISABLE
#define EVR_RTX_THREAD_GET_ID_DISABLE
#define EVR_RTX_THREAD_GET_STATE_DISABLE
#define EVR_RTX_THREAD_GET_STACK_SIZE_DISABLE
#define EVR_RTX_THREAD_GET_STACK_SPACE_DISABLE
#define EVR_RTX_THREAD_SET_PRIORITY_DISABLE
#define EVR_RTX_THREAD_GET_PRIORITY_DISABLE
#define EVR_RTX_THREAD_YIELD_DISABLE
#define EVR_RTX_THREAD_SUSPEND_DISABLE
#define EVR_RTX_THREAD_SUSPENDED_DISABLE
#define EVR_RTX_THREAD_RESUME_DISABLE
#define EVR_RTX_THREAD_RESUMED_DISABLE
#define EVR_RTX_THREAD_DETACH_DISABLE
#define EVR_RTX_THREAD_DETACHED_DISABLE
#define EVR_RTX_THREAD_JOIN_DISABLE
#define EVR_RTX_THREAD_JOIN_PENDING_DISABLE
#define EVR_RTX_THREAD_JOINED_DISABLE
#define EVR_RTX_THREAD_BLOCKED_DISABLE
#define EVR_RTX_THREAD_UNBLOCKED_DISABLE
#define EVR_RTX_THREAD_PREEMPTED_DISABLE
#define EVR_RTX_THREAD_SWITCHED_DISABLE
#define EVR_RTX_THREAD_DESTROYED_DISABLE
#define EVR_RTX_THREAD_GET_COUNT_DISABLE
#define EVR_RTX_THREAD_ENUMERATE_DISABLE
#define EVR_RTX_THREAD_FLAGS_SET_DISABLE
#define EVR_RTX_THREAD_FLAGS_SET_DONE_DISABLE
#define EVR_RTX_THREAD_FLAGS_CLEAR_DISABLE
#define EVR_RTX_THREAD_FLAGS_CLEAR_DONE_DISABLE
#define EVR_RTX_THREAD_FLAGS_GET_DISABLE
#define EVR_RTX_THREAD_FLAGS_WAIT_DISABLE
#define EVR_RTX_THREAD_FLAGS_WAIT_PENDING_DISABLE
#define EVR_RTX_THREAD_FLAGS_WAIT_TIMEOUT_DISABLE
#define EVR_RTX_THREAD_FLAGS_WAIT_COMPLETED_DISABLE
#define EVR_RTX_THREAD_FLAGS_WAIT_NOT_COMPLETED_DISABLE
#define EVR_RTX_THREAD_DELAY_DISABLE
#define EVR_RTX_THREAD_DELAY_UNTIL_DISABLE
#define EVR_RTX_THREAD_DELAY_COMPLETED_DISABLE
#define EVR_RTX_TIMER_CALLBACK_DISABLE
#define EVR_RTX_TIMER_NEW_DISABLE
#define EVR_RTX_TIMER_CREATED_DISABLE
#define EVR_RTX_TIMER_GET_NAME_DISABLE
#define EVR_RTX_TIMER_START_DISABLE
#define EVR_RTX_TIMER_STARTED_DISABLE
#define EVR_RTX_TIMER_STOP_DISABLE
#define EVR_RTX_TIMER_STOPPED_DISABLE
#define EVR_RTX_TIMER_IS_RUNNING_DISABLE
#define EVR_RTX_TIMER_DELETE_DISABLE
#define EVR_RTX_TIMER_DESTROYED_DISABLE
#define EVR_RTX_EVENT_FLAGS_NEW_DISABLE
#define EVR_RTX_EVENT_FLAGS_CREATED_DISABLE
#define EVR_RTX_EVENT_FLAGS_GET_NAME_DISABLE
#define EVR_RTX_EVENT_FLAGS_SET_DISABLE
#define EVR_RTX_EVENT_FLAGS_SET_DONE_DISABLE
#define EVR_RTX_EVENT_FLAGS_CLEAR_DISABLE
#define EVR_RTX_EVENT_FLAGS_CLEAR_DONE_DISABLE
#define EVR_RTX_EVENT_FLAGS_GET_DISABLE
#define EVR_RTX_EVENT_FLAGS_WAIT_DISABLE
#define EVR_RTX_EVENT_FLAGS_WAIT_PENDING_DISABLE
#define EVR_RTX_EVENT_FLAGS_WAIT_TIMEOUT_DISABLE
#define EVR_RTX_EVENT_FLAGS_WAIT_COMPLETED_DISABLE
#define EVR_RTX_EVENT_FLAGS_WAIT_NOT_COMPLETED_DISABLE
#define EVR_RTX_EVENT_FLAGS_DELETE_DISABLE
#define EVR_RTX_EVENT_FLAGS_DESTROYED_DISABLE
#define EVR_RTX_MUTEX_NEW_DISABLE
#define EVR_RTX_MUTEX_CREATED_DISABLE
#define EVR_RTX_MUTEX_GET_NAME_DISABLE
#define EVR_RTX_MUTEX_ACQUIRE_DISABLE
#define EVR_RTX_MUTEX_ACQUIRE_PENDING_DISABLE
#define EVR_RTX_MUTEX_ACQUIRE_TIMEOUT_DISABLE
#define EVR_RTX_MUTEX_ACQUIRED_DISABLE
#define EVR_RTX_MUTEX_NOT_ACQUIRED_DISABLE
#define EVR_RTX_MUTEX_RELEASE_DISABLE
#define EVR_RTX_MUTEX_RELEASED_DISABLE
#define EVR_RTX_MUTEX_GET_OWNER_DISABLE
#define EVR_RTX_MUTEX_DELETE_DISABLE
#define EVR_RTX_MUTEX_DESTROYED_DISABLE
#define EVR_RTX_SEMAPHORE_NEW_DISABLE
#define EVR_RTX_SEMAPHORE_CREATED_DISABLE
#define EVR_RTX_SEMAPHORE_GET_NAME_DISABLE
#define EVR_RTX_SEMAPHORE_ACQUIRE_DISABLE
#define EVR_RTX_SEMAPHORE_ACQUIRE_PENDING_DISABLE
#define EVR_RTX_SEMAPHORE_ACQUIRE_TIMEOUT_DISABLE
#define EVR_RTX_SEMAPHORE_ACQUIRED_DISABLE
#define EVR_RTX_SEMAPHORE_NOT_ACQUIRED_DISABLE
#define EVR_RTX_SEMAPHORE_RELEASE_DISABLE
#define EVR_RTX_SEMAPHORE_RELEASED_DISABLE
#define EVR_RTX_SEMAPHORE_GET_COUNT_DISABLE
#define EVR_RTX_SEMAPHORE_DELETE_DISABLE
#define EVR_RTX_SEMAPHORE_DESTROYED_DISABLE
#define EVR_RTX_MEMORY_POOL_NEW_DISABLE
#define EVR_RTX_MEMORY_POOL_CREATED_DISABLE
#define EVR_RTX_MEMORY_POOL_GET_NAME_DISABLE
#define EVR_RTX_MEMORY_POOL_ALLOC_DISABLE
#define EVR_RTX_MEMORY_POOL_ALLOC_PENDING_DISABLE
#define EVR_RTX_MEMORY_POOL_ALLOC_TIMEOUT_DISABLE
#define EVR_RTX_MEMORY_POOL_ALLOCATED_DISABLE
#define EVR_RTX_MEMORY_POOL_ALLOC_FAILED_DISABLE
#define EVR_RTX_MEMORY_POOL_FREE_DISABLE
#define EVR_RTX_MEMORY_POOL_DEALLOCATED_DISABLE
#define EVR_RTX_MEMORY_POOL_FREE_FAILED_DISABLE
#define EVR_RTX_MEMORY_POOL_GET_CAPACITY_DISABLE
#define EVR_RTX_MEMORY_POOL_GET_BLOCK_SZIE_DISABLE
#define EVR_RTX_MEMORY_POOL_GET_COUNT_DISABLE
#define EVR_RTX_MEMORY_POOL_GET_SPACE_DISABLE
#define EVR_RTX_MEMORY_POOL_DELETE_DISABLE
#define EVR_RTX_MEMORY_POOL_DESTROYED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_NEW_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_CREATED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_NAME_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_PUT_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_PUT_PENDING_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_PUT_TIMEOUT_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_INSERT_PENDING_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_INSERTED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_NOT_INSERTED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_PENDING_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_TIMEOUT_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_RETRIEVED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_NOT_RETRIEVED_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_CAPACITY_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_MSG_SIZE_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_COUNT_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_GET_SPACE_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_RESET_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_RESET_DONE_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_DELETE_DISABLE
#define EVR_RTX_MESSAGE_QUEUE_DESTROYED_DISABLE

#endif /* MBED_RTX_CONF_H */
