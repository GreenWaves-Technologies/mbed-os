/*
 * Copyright (c) 2018, GreenWaves Technologies, Inc.
 * Copyright (c) 2015 ETH Zurich and University of Bologna
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

#ifndef _GAP_DEBUG_H_
#define _GAP_DEBUG_H_

#include <stdint.h>
#include "gap_util.h"

#define HAL_DEBUG_STRUCT_NAME Debug_Struct
#define HAL_DEBUG_STRUCT_NAME_STR "Debug_Struct"

#define GAP_USE_DEBUG_STRUCT 1
/* #define GAP_USE_NEW_REQLOOP  1 */
#define PRINTF_BUF_SIZE 128

#ifdef GAP_USE_DEBUG_STRUCT
typedef enum {
  BRIDGE_REQ_CONNECT = 0,
  BRIDGE_REQ_DISCONNECT = 1,
  BRIDGE_REQ_OPEN = 2,
  BRIDGE_REQ_READ = 3,
  BRIDGE_REQ_WRITE = 4,
  BRIDGE_REQ_CLOSE = 5,
  BRIDGE_REQ_FB_OPEN = 6,
  BRIDGE_REQ_FB_UPDATE = 7,
#ifdef GAP_USE_NEW_REQLOOP
  BRIDGE_REQ_TARGET_STATUS_SYNC = 8,
  BRIDGE_REQ_FIRST_USER = 9
#else
  BRIDGE_REQ_FIRST_USER = 8
#endif
} bridge_req_e;

typedef struct _bridge_req_s {
  uint32_t next;
  uint32_t size;
  uint32_t type;
  uint32_t done;
  uint32_t popped;
  uint32_t padding;
  union {
    struct {
      uint32_t name_len;
      uint32_t name;
      uint32_t flags;
      uint32_t mode;
      uint32_t retval;
    } open;
    struct {
      uint32_t file;
      uint32_t retval;
    } close;
    struct {
      uint32_t file;
      uint32_t ptr;
      uint32_t len;
      uint32_t retval;
    } read;
    struct {
      uint32_t file;
      uint32_t ptr;
      uint32_t len;
      uint32_t retval;
    } write;
    struct {
      uint64_t screen;
      uint32_t name_len;
      uint32_t name;
      uint32_t width;
      uint32_t height;
      uint32_t format;
    } fb_open;
    struct {
      uint64_t screen;
      uint32_t addr;
      uint32_t posx;
      uint32_t posy;
      uint32_t width;
      uint32_t height;
    } fb_update;
#ifdef GAP_USE_NEW_REQLOOP
    struct {
    } target_status_sync;
#endif
  };
} bridge_req_t;

#ifdef GAP_USE_NEW_REQLOOP
typedef struct {
  volatile int32_t available;
} target_state_t;

typedef struct {
  volatile int32_t connected;
} bridge_state_t;
#endif

/* This structure can be used to interact with the host loader */
typedef struct _debug_struct {
#ifdef GAP_USE_NEW_REQLOOP
    target_state_t target;

    bridge_state_t bridge;
#endif
    /* Used by external debug bridge to get exit status when using the board */
    uint32_t exitStatus;

    /* Printf */
    uint32_t useInternalPrintf;
    uint32_t putcharPending;
    uint32_t putcharCurrent;
    uint8_t putcharBuffer[PRINTF_BUF_SIZE];

    /* Debug step, used for showing progress to host loader */
    uint32_t debugStep;
    uint32_t debugStepPending;

    // Requests
    uint32_t firstReq;
    uint32_t lastReq;
    uint32_t firstBridgeReq;

    uint32_t notifReqAddr;
    uint32_t notifReqValue;

#ifndef GAP_USE_NEW_REQLOOP
    uint32_t bridgeConnected;
#endif

} debug_struct_t;
#endif

extern debug_struct_t HAL_DEBUG_STRUCT_NAME;

static inline debug_struct_t* DEBUG_GetDebugStruct()
{
#ifdef GAP_USE_DEBUG_STRUCT
  return &HAL_DEBUG_STRUCT_NAME;
#else
  return (void *)0;
#endif
}

#ifdef GAP_USE_NEW_REQLOOP
#define GAP_DEBUG_STRUCT_INIT { {0}, {0}, 0, 1, 0 ,0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0}
#else
#define GAP_DEBUG_STRUCT_INIT {0, 1, 0 ,0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

static inline int BRIDGE_isConnected(debug_struct_t *bridge) {
#ifdef GAP_USE_NEW_REQLOOP
  return *(volatile uint32_t *)&bridge->bridge.connected;
#else
  return *(volatile uint32_t *)&bridge->bridgeConnected;
#endif
}

static inline void BRIDGE_Connect(bridge_req_t *req)
{
  req->type = BRIDGE_REQ_CONNECT;
}

static inline void BRIDGE_Disconnect(bridge_req_t *req)
{
  req->type = BRIDGE_REQ_DISCONNECT;
}

static inline void BRIDGE_open(bridge_req_t *req, int name_len, const char* name, int flags, int mode)
{
  req->type = BRIDGE_REQ_OPEN;
  req->open.name_len = name_len;
  req->open.name = (uint32_t)(long)name;
  req->open.flags = flags;
  req->open.mode = mode;
}

static inline void BRIDGE_close(bridge_req_t *req, int file)
{
  req->type = BRIDGE_REQ_CLOSE;
  req->close.file = file;
}

static inline void BRIDGE_read(bridge_req_t *req, int file, void* ptr, int len)
{
  req->type = BRIDGE_REQ_READ;
  req->read.file = file;
  req->read.ptr = (uint32_t)(long)ptr;
  req->read.len = len;
}

static inline void BRIDGE_write(bridge_req_t *req, int file, void* ptr, int len)
{
  req->type = BRIDGE_REQ_WRITE;
  req->write.file = file;
  req->write.ptr = (uint32_t)(long)ptr;
  req->write.len = len;
}

static inline void DEBUG_FlushPrintf(debug_struct_t *debugStruct) {
  while(*(volatile uint32_t *)&debugStruct->putcharPending);
}

static inline void DEBUG_Exit(debug_struct_t *debugStruct, int status) {
  *(volatile uint32_t *)&debugStruct->exitStatus = 0x80000000 | status;
}

static inline void DEBUG_Putchar(debug_struct_t *debugStruct, char c) {
  DEBUG_FlushPrintf(debugStruct);
  *(volatile uint8_t *)&(debugStruct->putcharBuffer[debugStruct->putcharCurrent++]) = c;
  if (*(volatile uint32_t *)&debugStruct->putcharCurrent == PRINTF_BUF_SIZE || c == '\n') {
    *(volatile uint32_t *)&debugStruct->putcharPending = debugStruct->putcharCurrent;
    *(volatile uint32_t *)&debugStruct->putcharCurrent = 0;
  }
}

static inline void DEBUG_Step(debug_struct_t *debugStruct, uint32_t value) {
  *(volatile uint32_t *)&debugStruct->debugStep = value;
  *(volatile uint32_t *)&debugStruct->debugStepPending = 1;
  while (*(volatile uint32_t *)&debugStruct->debugStepPending);
}

static inline void DEBUG_Reset(debug_struct_t *debugStruct) {
  *(volatile uint32_t *)&debugStruct->exitStatus = 0x80000000 | 0x40000000;
}

#endif
