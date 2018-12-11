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

#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "gap_bridge.h"
#include "gap_handler_wrapper.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
bridge_req_t request;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* handler wrapper  */
Handler_Wrapper_Light(BRIDGE_IRQHandler);

/*******************************************************************************
 * Code
 ******************************************************************************/
static inline bridge_t *BRIDGE_Get()
{
    return DEBUG_GetDebugStruct();
}

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

static inline void BRIDGE_Open(bridge_req_t *req, int name_len, const char* name, int flags, int mode)
{
    req->type = BRIDGE_REQ_OPEN;
    req->open.name_len = name_len;
    req->open.name = (uint32_t)(long)name;
    req->open.flags = flags;
    req->open.mode = mode;
}

static inline void BRIDGE_Close(bridge_req_t *req, int file)
{
    req->type = BRIDGE_REQ_CLOSE;
    req->close.file = file;
}

static inline void BRIDGE_Read(bridge_req_t *req, int file, void* ptr, int len)
{
    req->type = BRIDGE_REQ_READ;
    req->read.file = file;
    req->read.ptr = (uint32_t)(long)ptr;
    req->read.len = len;
}

static inline void BRIDGE_Write(bridge_req_t *req, int file, void* ptr, int len)
{
    req->type = BRIDGE_REQ_WRITE;
    req->write.file = file;
    req->write.ptr = (uint32_t)(long)ptr;
    req->write.len = len;
}


static inline void BRIDGE_FBOpen(bridge_req_t *req, int name_len, const char* name, int width, int height, int format)
{
    req->type = BRIDGE_REQ_FB_OPEN;
    req->fb_open.name_len = name_len;
    req->fb_open.name = (uint32_t)(long)name;
    req->fb_open.width = width;
    req->fb_open.height = height;
    req->fb_open.format = format;
}

static inline void BRIDGE_FBUpdate(bridge_req_t *req, uint64_t fb, unsigned int addr, int posx, int posy, int width, int height)
{
    req->type = BRIDGE_REQ_FB_UPDATE;
    req->fb_update.screen = fb;
    req->fb_update.addr = addr;
    req->fb_update.posx = posx;
    req->fb_update.posy = posy;
    req->fb_update.width = width;
    req->fb_update.height = height;
}

static inline void BRIDGE_TargetStatusSync(bridge_req_t *req)
{
    req->type = BRIDGE_REQ_TARGET_STATUS_SYNC;
}


// This function should be called everytime the bridge may have new requests
// to handle, so that its pointer to ready requests is updated
static void BRIDGE_CheckBridgeReq()
{
    bridge_t *bridge = BRIDGE_Get();

    // first_bridge_req is owned by the runtime only when it is NULL, otherwise
    // it is owned by the bridge which is modifying it everytime a request
    // is handled
    if (bridge->firstBridgeReq == 0)
    {
        // We are owning the pointer, check if we can make it point to ready
        // requests
        bridge_req_t *req = (bridge_req_t *)bridge->firstReq;

        while (req && req->popped)
        {
            req = (bridge_req_t *)req->next;
        }

        if (req)
        {
            bridge->firstBridgeReq = (uint32_t )req;
        }
    }
}

// This function can be called by the API layer to commit a new request to
// the bridge
static void BRIDGE_PostReq(bridge_req_t *req, void *event)
{
    bridge_t *bridge = BRIDGE_Get();

    req->next = 0;
    req->done = 0;
    req->popped = 0;
    req->size = sizeof(bridge_req_t);
    /* req->event = event; */

    if (bridge->firstReq)
        ((bridge_req_t *)bridge->lastReq)->next = (uint32_t)req;
    else
        bridge->firstReq = (uint32_t)req;

    bridge->lastReq = (uint32_t)req;
    req->next = 0;

    BRIDGE_CheckBridgeReq();
}

static void BRIDGE_HandleNotify(bridge_t* bridge)
{
    // Go through all the requests and handles the ones which are done
    bridge_req_t *req = (bridge_req_t *)bridge->firstReq;

    while (req && req->done)
    {
        bridge_req_t *next = (bridge_req_t *)req->next;
        bridge->firstReq = (uint32_t)next;
    }

    // Then check if we must update the bridge queue
    BRIDGE_CheckBridgeReq();
}

void BRIDGE_BlockWait()
{
    /* Disable IRQ */
    int irq_en = NVIC_GetEnableIRQ(FC_SW_NOTIFY_BRIDGE_EVENT);
    NVIC_DisableIRQ(FC_SW_NOTIFY_BRIDGE_EVENT);

    int event = 0;
    do {
        event = EU_EVT_MaskWaitAndClr(1 << FC_SW_NOTIFY_BRIDGE_EVENT);
    } while (!(event & (1 << FC_SW_NOTIFY_BRIDGE_EVENT)));

    /* Restore IRQ */
    if (irq_en)
        NVIC_EnableIRQ(FC_SW_NOTIFY_BRIDGE_EVENT);
}

void BRIDGE_EventInit()
{
    bridge_t* bridge = BRIDGE_Get();

    /* Tell debug bridge which address to trigger software event */
    bridge->notifyReqAddr  = (uint32_t)(&FC_EU_SW_EVENTS->TRIGGER_SET[FC_SW_NOTIFY_BRIDGE_EVENT]);
    bridge->notifyReqValue = 1;

    /* Bind software event handle */
    NVIC_SetVector(FC_SW_NOTIFY_BRIDGE_EVENT, (uint32_t)__handler_wrapper_light_BRIDGE_IRQHandler);
    /* Activate interrupt handler for soc event */
    NVIC_EnableIRQ(FC_SW_NOTIFY_BRIDGE_EVENT);
}

int BRIDGE_EventConnect(int wait_bridge, void *event)
{
    bridge_t *bridge = BRIDGE_Get();

    if (!wait_bridge && !BRIDGE_isConnected(bridge)) {
        return -1;
    }

    request.next = 0;

    BRIDGE_Connect(&request);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return 0;
}

void BRIDGE_EventDisconnect(void *event)
{
    request.next = 0;

    BRIDGE_Disconnect(&request);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();
}

int BRIDGE_EventOpen(const char* name, int flags, int mode, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_Open(&request, strlen(name), name, flags, mode);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return request.open.retval;
}

int BRIDGE_EventOpenWait(void* event)
{
    BRIDGE_BlockWait();

    return request.open.retval;
}

int BRIDGE_EventClose(int file, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_Close(&request, file);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return request.close.retval;
}

int BRIDGE_EventCloseWait(void *event)
{
    BRIDGE_BlockWait();

    return request.close.retval;
}

int BRIDGE_EventRead(int file, void* ptr, int len, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_Read(&request, file, ptr, len);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return request.read.retval;
}

int BRIDGE_EventReadWait(void *event)
{
    BRIDGE_BlockWait();

    return request.read.retval;
}

int BRIDGE_EventWrite(int file, void* ptr, int len, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_Write(&request, file, ptr, len);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return request.write.retval;
}

int BRIDGE_EventWriteWait(void *event)
{
    BRIDGE_BlockWait();

    return request.write.retval;
}

uint64_t BRIDGE_EventFBOpen(const char* name, int width, int height, bridge_fb_format_e format, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_FBOpen(&request, strlen(name), name, width, height, format);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();

    return request.fb_open.screen;
}

uint64_t BRIDGE_EventFBOpenWait(void *event)
{
    BRIDGE_BlockWait();

    return request.fb_open.screen;
}

void BRIDGE_EventFBUpdate(uint64_t fb, int addr, int posx, int posy, int width, int height, void *event)
{
    memset((void *)&request, 0, sizeof(bridge_req_t));

    BRIDGE_FBUpdate(&request, fb, addr, posx, posy, width, height);

    BRIDGE_PostReq(&request, NULL);

    BRIDGE_BlockWait();
}

void BRIDGE_EventTargetStatusSync(void *event)
{
    bridge_t *bridge = BRIDGE_Get();

    if ( BRIDGE_isConnected(bridge) ) {
        memset((void *)&request, 0, sizeof(bridge_req_t));

        BRIDGE_TargetStatusSync(&request);

        BRIDGE_PostReq(&request, NULL);

        BRIDGE_BlockWait();
    }
}

void BRIDGE_IRQHandler(void)
{
}
