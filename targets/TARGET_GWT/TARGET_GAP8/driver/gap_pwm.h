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

#ifndef _GAP_PWM_H_
#define _GAP_PWM_H_

#include <assert.h>
#include "gap_util.h"
#include "gap_soc_eu.h"

/*!
 * @addtogroup PWM
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Threshold values for each channels. */
typedef enum
{
    uPWM_TH_CHANNEL0 = 0x00,
    uPWM_TH_CHANNEL1 = 0x01,
    uPWM_TH_CHANNEL2 = 0x02,
    uPWM_TH_CHANNEL3 = 0x03
} pwm_channel_t;

/*! @brief PWM commands. */
typedef enum
{
    uPWM_CMD_START  = 0x01,
    uPWM_CMD_STOP   = 0x02,
    uPWM_CMD_UPDATE = 0x04,
    uPWM_CMD_RESET  = 0x08,
    uPWM_CMD_ARM    = 0x10
} pwm_cmd_t;

/*! @brief Event trigger mode. */
typedef enum
{
    uPWM_EACH_CLK_CYCLE     = 0x00,
    uPWM_IN_SOURCE_0        = 0x01,
    uPWM_IN_SOURCE_1        = 0x02,
    uPWM_IN_RISING_EDGE     = 0x03,
    uPWM_IN_FALLING_EDGE    = 0x04,
    uPWM_IN_RISE_FALL_EDGE  = 0x05,
    uPWM_IN_RISE_EDGE_ARMED = 0x06,
    uPWM_IN_FALL_EDGE_ARMED = 0x07
} pwm_trigger_mode_t;

/*! @brief PWM's clock source. */
typedef enum
{
    uPWM_FLL     = 0x00,
    uPWM_REF_32K = 0x01
} pwm_clk_src_t;

/*! @brief Counter reset method when threshold reached. */
typedef enum
{
    uPWM_MOUNTAIN = 0x00, /* /\/\/\ */
    uPWM_SAW      = 0x01  /* /|/|/| */
} pwm_updown_mode_t;

/*! @brief Action when threshold match. */
typedef enum
{
    uPWM_SET          = 0x00,
    uPWM_TOGGLE_CLEAR = 0x01,
    uPWM_SET_CLEAR    = 0x02,
    uPWM_TOGGLE       = 0x03,
    uPWM_CLEAR        = 0x04,
    uPWM_TOGGLE_SET   = 0x05,
    uPWM_CLEAR_SET    = 0x06
} pwm_channel_mode_t;

/*! @brief Event output select. */
typedef enum
{
    uPWM_EVENT_SEL0 = 0x00,
    uPWM_EVENT_SEL1 = 0x01,
    uPWM_EVENT_SEL2 = 0x02,
    uPWM_EVENT_SEL3 = 0x03
} pwm_event_sel_t;

/*! @brief Options to configure a Timer channel's PWM signal */
typedef struct _pwm_signal_param
{
    pwm_channel_t chnum;         /*!< The channel number.*/

    uint8_t dutyCyclePercent;      /*!< PWM pulse width, value should be between 0 to 100
                                     0 = inactive signal(0% duty cycle)...
                                     100 = always active signal (100% duty cycle).*/
} pwm_signal_param_t;

/*! @brief Options to configure a PWM signal */
/*
 * PWM timer configuration
 * Arguments:
 *      - inputsrc: Select counting source
 *                      0-31 gpio
 *                      32-35 out of timer 0
 *                      ...
 *                      44-47 out of timer 3
 *      - mode: count rules
 *                      uPWM_EACH_CLK_CYCLE         0x0 //event at each clock cycle
 *                      uPWM_IN_SOURCE_0            0x1 //event if input signal is 0
 *                      uPWM_IN_SOURCE_1            0x2 //event if input signal is 1
 *                      uPWM_IN_RISING_EDGE         0x3 //event on input signal rising edge
 *                      uPWM_IN_FALLING_EDGE        0x4 //event on input signal falling edge
 *                      uPWM_IN_RISE_FALL_EDGE      0x5 //event on input signal rising or falling edge
 *                      uPWM_IN_RISE_EDGE_ARMED     0x6 //event on input signal rising edge when armed
 *                      uPWM_IN_FALL_EDGE_ARMED     0x7 //event on input signal falling edge when armed
 *      - clksel:
 *                      uPWM_FLL                    0x00
 *                      uPWM_REF_32K                0x01
 *      - updownsel: action when reaching the TH:
 *                      uPWM_MOUNTAIN :  /\/\/\ (NOSAW)
 *                      uPWM_SAW      :  /|/|/| (SAW)
 *      - prescale
 */
typedef struct _pwm_config_t
{
    uint8_t inputsrc;
    pwm_trigger_mode_t mode;
    pwm_clk_src_t clksel;
    pwm_updown_mode_t updownsel;
    uint8_t prescale;
    pwm_event_sel_t evtSel;
} pwm_config_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Get instance number of an PWM.
 *
 * @param base PWM peripheral base address.
 */
uint32_t PWM_GetInstance(PWM_Type *base);

/*!
 * @brief Init an PWM.
 *
 * @param base          PWM instance.
 * @param master_config Pointer to the struct holding PWM's config.
 *
 */
void PWM_Init(PWM_Type *base, pwm_config_t *master_config);

/*!
 * @brief De-Init an PWM.
 *
 * @param base          PWM instance.
 */
void PWM_Deinit(PWM_Type *base);

/*!
 * @brief Get PWM's configuration.
 *
 * @param master_config Pointer to the struct holding PWM's config.
 */
void PWM_GetDefaultConfig(pwm_config_t *master_config);

/*!
 * @brief Send a command to start a module.
 *
 * @param base    PWM instance.
 */
void PWM_StartTimer(PWM_Type *base);

/*!
 * @brief Send a command to stop a module.
 *
 * @param base    PWM instance.
 */
void PWM_StopTimer(PWM_Type *base);

/*!
 * @brief @brief Configures the PWM signal parameters
 *
 * @param base    PWM instance.
 * @param ch_params   Array of PWM channel parameters to configure the channel(s)
 * @param ch_num      Number of channels to configure; This should be the size of the array passed in
 * @param pwmFreq_Hz  PWM signal frequency in Hz
 * @param clkSrc     clock source
 *
 * @return uStatus_Success if the PWM setup was successful
 *         uStatus_Error on failure
 */
status_t PWM_SetupPwm(PWM_Type *base, const pwm_signal_param_t *ch_params,
                      uint8_t ch_num, uint32_t pwmFreq_Hz, pwm_clk_src_t clkSrc);

/*!
 * @brief Configure threshold value of a module.
 *
 * @param base      PWM instance.
 * @param threshold Threshold value to set.
 */
void PWM_SetThreshold(PWM_Type *base, uint32_t threshold);

/*!
 * @brief Init Channel.
 *
 * @param base      PWM instance.
 * @param channel   Number of the channel to config.
 * @param threshold Channel's threshold value.
 * @param mode      Channel's mode.
 */
void PWM_ChannelConfig(PWM_Type *base, pwm_channel_t channel, uint16_t threshold, uint8_t mode);

/*!
 * @brief Set PWM's output event.
 *
 * @param base    PWM instance.
 * @param channel The number of channel.
 * @param evtSel  The number of event sel.
 */
void PWM_SetOutputEvent(PWM_Type *base, pwm_channel_t channel, pwm_event_sel_t evtSel);

/*!
 * @brief Clear PWM's output event.
 *
 * @param base    PWM instance.
 * @param channel The number of channel.
 * @param evtSel  The number of event sel.
 */
void PWM_ClearOutputEvent(PWM_Type *base, pwm_channel_t channel, pwm_event_sel_t evtSel);

/*!
 * @brief PWM IRQ Handler Binding
 *
 * @param base    PWM instance.
 * @param irq     PWM IRQ handler function pointer to binding
 * @param arg     Argument of callbcak function
 */
void PWM_IRQHandlerBind(PWM_Type *base, uint32_t irq, void *arg);

/*!
 * @brief PWM's event handler.
 *
 * @param evtSel The number of event output sel.
 */
void PWM_IRQHandler(uint32_t evtSel);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_PWM_H_*/
