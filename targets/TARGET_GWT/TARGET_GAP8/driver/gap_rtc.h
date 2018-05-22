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

#ifndef _GAP_RTC_H_
#define _GAP_RTC_H_

#include "gap_udma.h"

/*!
 * @addtogroup rtc_driver
 * @{
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
#define GAP_RTC_DRIVER_VERSION (MAKE_VERSION(1, 0, 0)) /*!< Version 1.0.0 */
/*@}*/

/*! @brief List of RTC interrupts */
typedef enum _rtc_interrupt_enable
{
    uRTC_AlarmInterruptEnable = RTC_IMR_ALARM_MASK,  /*!< Alarm interrupt.*/
    uRTC_TimerInterruptEnable = RTC_IMR_TIMER_MASK,  /*!< Timer interrupt.*/
    uRTC_CalibrationInterruptEnable = RTC_IMR_CALIBRATION_MASK  /*!< Calibration interrupt.*/
} rtc_interrupt_enable_t;

/*! @brief List of RTC flags */
typedef enum _rtc_status_flags
{
    uRTC_AlarmFlag = RTC_IFR_ALARM_MASK,  /*!< Alarm flag */
    uRTC_TimerFlag = RTC_IFR_TIMER_MASK, /*!< Timer flag */
    uRTC_CalibrationFlag = RTC_IFR_CALIBRATION_MASK  /*!< Calibration flag*/
} rtc_status_flags_t;

/*! @brief  RTC status return codes. */
enum _rtc_status
{
    uStatus_RTC_TimeInvalid  = MAKE_STATUS(uStatusGroup_RTC, 0),            /*!< RTC time invalid. */
    uStatus_RTC_TimeOverflow = MAKE_STATUS(uStatusGroup_RTC, 1),            /*!< RTC time overflow. */
    uStatus_RTC_Alarm        = MAKE_STATUS(uStatusGroup_RTC, 2)             /*!< RTC time alarm */
};

/*! @brief RTC Clock defaut frequency.*/
#define RTC_CLK_FRE_DEFAUT     32768

/*! @brief This is used to make RTC works in which mode. */
typedef enum {
  MODE_CALENDAR = 0,   /*!< Calendar mode.*/
  MODE_ALARM,          /*!< Alarm mode, work with calendar mode.*/
  MODE_COUNTDOWN,      /*!< Count down mode.*/
  MODE_CALIBR          /*!< RTC Calibration.*/
} rtc_mode_t;

/*! @brief  This is used to set the alarm repeat mode */
typedef enum{
  RPT_SEC  = 3,          /*!< Repeat per second.*/
  RPT_MIN  = 4,          /*!< Repeat per minute*/
  RPT_HOUR = 5,          /*!< Repeat per hour*/
  RPT_DAY  = 6,          /*!< Repeat per day*/
  RPT_MON  = 7,          /*!< Repeat per month*/
  RPT_YEAR = 8           /*!< Repeat per year*/
} rt_alarm_rpt_mode_t;

/*! @brief Structure is used to hold the date and time */
typedef struct _rtc_datetime_bcd
{
    uint32_t time;  /*!< Time coded in Bcd, for example: 0x00124508 = 12H:45M:08S */
    uint32_t date;  /*!< Date coded in Bcd, for example: 0x00171228 = 2017/12/28 */
} rtc_datetime_bcd_t;

/*! @brief Structure is used to hold the date and time */
typedef struct _rtc_datetime
{
    uint16_t year;  /*!< Range from 2000 to 2099.*/
    uint8_t month;  /*!< Range from 1 to 12.*/
    uint8_t day;    /*!< Range from 1 to 31 (depending on month).*/
    uint8_t hour;   /*!< Range from 0 to 23.*/
    uint8_t minute; /*!< Range from 0 to 59.*/
    uint8_t second; /*!< Range from 0 to 59.*/
} rtc_datetime_t;

/*! @brief RTC  configuration structure.*/
typedef struct _rtc_config
{
    uint8_t mode;
    uint32_t clkDiv;     /*!< Baud rate configuration of RTC peripheral. */
} rtc_config_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name Initialization and deinitialization
 * @{
 */
/*!
 * @brief Initializes the RTC .
 *
 * This function initializes the RTC configuration. This is an example use case.
 *  @code
 *   rtc_config_t  Config;
 *   Config.mode = 100000;
 *   Config.clkDiv = 100000;
 *   RTC_Init(base, &Config);
 *  @endcode
 *
 * @param base RTC peripheral address.
 * @param Config Pointer to the structure rtc_config_t.
 */
void RTC_Init(RTC_APB_Type *base, const rtc_config_t *Config);

/*!
 * @brief De-initializes the RTC peripheral. Call this API to gate the RTC clock.
 * The RTC module can't work unless the RTC_Init is called.
 * @param base RTC base pointer
 */
void RTC_Deinit(RTC_APB_Type *base);

/*!
 * @brief Fills in the RTC config struct with the default settings.
 *
 * The default values are as follows.
 * @code
 *    Config->mode   = MODE_CALENDAR;
 *    Config->clkDiv = 2;
 * @endcode
 * @param Config Pointer to the user's RTC configuration structure.
 */
void RTC_GetDefaultConfig(rtc_config_t *Config);



/*!
 * @brief Convert datetime to second
 *
 * @param datetime Pointer to the structure where the date and time details are stored.
 */
uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime);

/* @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @name Current Time & Alarm
 * @{
 */

/*!
 * @brief Sets the RTC date and time according to the given time structure.
 *
 * The RTC counter must be stopped prior to calling this function because writes to the RTC
 * seconds register fail if the RTC counter is running.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 *
 *
 * @return uStatus_Success: Success in setting the time and starting the RTC
 *         uStatus_InvalidArgument: Error because the datetime format is incorrect
 */
status_t RTC_SetCalendar(RTC_APB_Type *base, const rtc_datetime_t *datetime);

/*!
 * @brief Gets the RTC time and stores it in the given time structure.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 */
void RTC_GetCalendar(RTC_APB_Type *base, rtc_datetime_t *datetime);

/*!
 * @brief Sets the RTC alarm time.
 *
 * The function checks whether the specified alarm time is greater than the present
 * time. If not, the function does not set the alarm and returns an error.
 *
 * @param base      RTC peripheral base address
 * @param alarmTime Pointer to the structure where the alarm time is stored.
 *
 * @return uStatus_Success: success in setting the RTC alarm
 *         uStatus_InvalidArgument: Error because the alarm datetime format is incorrect
 *         uStatus_Fail: Error because the alarm time has already passed
 */
status_t RTC_SetAlarm(RTC_APB_Type *base, const rtc_datetime_t *alarmTime);

/*!
 * @brief Returns the RTC alarm time.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to the structure where the alarm date and time details are stored.
 */
void RTC_GetAlarm(RTC_APB_Type *base, rtc_datetime_t *datetime);

/*!
 * @brief Sets the RTC Countedown time.
 *
 * The function checks whether the specified alarm time is greater than the present
 * time. If not, the function does not set the alarm and returns an error.
 *
 * @param base      RTC peripheral base address
 * @param count     The initial counter value.
 *
 */
void RTC_SetCountDown(RTC_APB_Type *base, const uint32_t count);

/*!
 * @brief Returns the RTC Countedown time.
 *
 * @param base     RTC peripheral base address
 *
 * @return Counter value
 */
uint32_t RTC_GetCountDown(RTC_APB_Type *base);

/*! @}*/

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enables the selected RTC interrupts.
 *
 * @param base RTC peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::rtc_interrupt_enable_t
 */
void RTC_EnableInterrupts(RTC_APB_Type *base, uint32_t mask);

/*!
 * @brief Disables the selected RTC interrupts.
 *
 * @param base RTC peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::rtc_interrupt_enable_t
 */
void RTC_DisableInterrupts(RTC_APB_Type *base, uint32_t mask);

/*!
 * @brief Gets the enabled RTC interrupts.
 *
 * @param base RTC peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::rtc_interrupt_enable_t
 */
uint32_t RTC_GetEnabledInterrupts(RTC_APB_Type *base);

/*!
 * @brief RTC APB IRQ Handler
 *
 */
void RTC_APB_IRQHandler();

/*!
 * @brief RTC IRQ Handler
 *
 */
void RTC_IRQHandler();

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Gets the RTC status flags.
 *
 * @param base RTC peripheral base address
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::rtc_status_flags_t
 */
uint32_t RTC_GetStatusFlags(RTC_APB_Type *base);

/*!
 * @brief  Clears the RTC status flags.
 *
 * @param base RTC peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::rtc_status_flags_t
 */
void RTC_ClearStatusFlags(RTC_APB_Type *base, uint32_t mask);

/*! @}*/

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the RTC calendar time counter.
 *
 * @param base RTC peripheral base address
 */
void RTC_StartCalendar(RTC_APB_Type *base);

/*!
 * @brief Stops the RTC calendar time counter.
 *
 * @param base RTC peripheral base address
 */
void RTC_StopCalendar(RTC_APB_Type *base);

/*!
 * @brief Starts the RTC alarm time counter.
 *
 * @param base RTC peripheral base address
 * @param repeatmode RTC alarm repeat mode
 */
void RTC_StartAlarm(RTC_APB_Type *base, rt_alarm_rpt_mode_t repeatmode);

/*!
 * @brief Stops the RTC alarm time counter.
 *
 * @param base RTC peripheral base address
 */
void RTC_StopAlarm(RTC_APB_Type *base);

/*!
 * @brief Starts the RTC countdown time counter.
 *
 * @param base RTC peripheral base address
 * @param repeat_en RTC countdown timer repeat enable
 */
void RTC_StartCountDown(RTC_APB_Type *base, uint8_t repeat_en);

/*!
 * @brief Stops the RTC countdown time counter.
 *
 * @param base RTC peripheral base address
 */
void RTC_StopCountDown(RTC_APB_Type *base);

/*!
 * @brief RTC calibration.
 *
 * @param base RTC peripheral base address
 */
void RTC_Calibration(RTC_APB_Type *base);

/*!
 * @brief Get RTC Status.
 *
 * @param base RTC peripheral base address
 */
uint32_t RTC_GetStatus(RTC_APB_Type *base);

/*! @}*/

/*!
 * @brief Check if RTC is enbale
 *
 * @param base RTC peripheral base address
 */
int RTC_IsEnable(RTC_APB_Type *base);

/*!
 * @brief Performs a software reset on the RTC module.
 *
 * This resets all RTC registers except for the SWR bit and the RTC_WAR and RTC_RAR
 * registers. The SWR bit is cleared by software explicitly clearing it.
 *
 * @param base RTC peripheral base address
 */
void RTC_Reset(RTC_APB_Type *base);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_GAP_RTC_H_*/
