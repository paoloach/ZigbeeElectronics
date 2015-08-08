/*********************************************************************
  Filename:       hal_timer.c
  Revised:        $Date: 2009-12-22 12:38:28 -0800 (Tue, 22 Dec 2009) $
  Revision:       $Revision: 21387 $

  Description:   This file contains the interface to the Timer Service.


  Copyright 2006-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <intrinsics.h>
#include "hal_types.h"
#include "hal_timer.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"


/*********************************************************************
 * MACROS
 */
#define HAL_TIMER_INIT() { TBCCTL0 = 0 ; TBCTL = 0; TBCCR0 = 0; }


/*********************************************************************
 * CONSTANTS
 */

/* Timer clock pre-scaler definition for TimerB - MSP430 */
#define HAL_TIMERB_16_TC_DIV1   ID_0
#define HAL_TIMERB_16_TC_DIV2   ID_1
#define HAL_TIMERB_16_TC_DIV4   ID_2
#define HAL_TIMERB_16_TC_DIV8   ID_3

/* Prescale settings */
#define HAL_TIMER_B_16_PRESCALE      HAL_TIMERB_16_TC_DIV1
#define HAL_TIMER_B_16_PRESCALE_VAL  1

/* Clock settings, for MSP430s that have 6MHz crystal installed,
 * #define HAL_TIMER_FLL_ADJUST_KHZ HAL_TIMER_MHZ - this will turn off the FLL adjustment.
 * The integer casts prevent floating point library from being linked in.
 */
#define HAL_TIMER_MHZ             (uint8)(HAL_CPU_CLOCK_MHZ)         /* 6MHz for some boards */
#define HAL_TIMER_FLL_ADJUST_KHZ (uint32)(HAL_CPU_CLOCK_MHZ * 1000)  /* 6291KHZ */

/* Timer re-map */
#define HAL_TIMER_A   HAL_TIMER_1  /* Used by MAC */
#define HAL_TIMER_B   HAL_TIMER_3


/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  bool configured;
  bool intEnable;
  uint8 opMode;
  uint8 channel;
  uint8 channelMode;
  uint8 prescale;
  uint8 prescaleVal;
  uint8 clock;
  halTimerCBack_t callBackFunc;
}halTimerSettings_t;

/***************************************************************************************************
 *                                             GLOBAL VARIABLES
 ***************************************************************************************************/
static halTimerSettings_t timerRecord[HAL_TIMER_MAX];

/*********************************************************************
 * FUNCTIONS - Local
 */
static uint8 halTimerSetCount (uint8 timerId, uint32 timePerTick);
static uint8 halTimerSetPrescale (uint8 timerId, uint8 prescale);
static uint8 halTimerSetOpMode (uint8 timerId, uint8 opMode);
static void  halTimerSendCallBack (uint8 timerId, uint8 channel, uint8 channelMode);

/***************************************************************************************************
 *                                              FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalTimerInit
 *
 * @brief   Initialize Timer Service
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void HalTimerInit (void)
{
  /* Setup prescale & clock for timerB */
  timerRecord[HAL_TIMER_B].prescale    = HAL_TIMER_B_16_PRESCALE;
  timerRecord[HAL_TIMER_B].clock       = HAL_TIMER_MHZ; /* MHz */
  timerRecord[HAL_TIMER_B].prescaleVal = HAL_TIMER_B_16_PRESCALE_VAL;

  HAL_TIMER_INIT();
}

/***************************************************************************************************
 * @fn      HalTimerConfig
 *
 * @brief   Configure the Timer Serivce
 *
 * @param   timerId - Id of the timer
 *          opMode  - Operation mode
 *          channel - Channel where the counter operates on
 *          channelMode - Mode of that channel
 *          prescale - Prescale of the clock
 *          cBack - Pointer to the callback function
 *
 * @return  Status of the configuration
 ***************************************************************************************************/
uint8 HalTimerConfig ( uint8 timerId, uint8 opMode, uint8 channel, uint8 channelMode,  bool intEnable, halTimerCBack_t cBack )
{
  if ((opMode & HAL_TIMER_MODE_MASK) && (timerId < HAL_TIMER_MAX) && (channelMode & HAL_TIMER_CHANNEL_MASK) && (channel & HAL_TIMER_CHANNEL_MASK))
  {
    if (timerId == HAL_TIMER_B)
    {
      timerRecord[timerId].configured    = TRUE;
      timerRecord[timerId].opMode        = opMode;
      timerRecord[timerId].channel       = channel;      /* Ignored for MSP430 */
      timerRecord[timerId].channelMode   = channelMode;  /* Support only 1 Mode - Capture-Compare */
      timerRecord[timerId].intEnable     = intEnable;
      timerRecord[timerId].callBackFunc  = cBack;
    }
  }
  else
  {
    return HAL_TIMER_PARAMS_ERROR;
  }
  return HAL_TIMER_OK;
}


/***************************************************************************************************
 * @fn      HalTimerStart
 *
 * @brief   Start the Timer Service
 *
 * @param   timerId      - ID of the timer
 *          timerPerTick - number of micro sec per tick, (ticks x prescale) / clock = usec/tick
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerStart ( uint8 timerId, uint32 timePerTick)
{
  if (timerRecord[timerId].configured)
  {
    halTimerSetCount        (timerId, timePerTick);
    halTimerSetPrescale     (timerId, timerRecord[timerId].prescale);
    halTimerSetOpMode       (timerId, timerRecord[timerId].opMode);
    HalTimerInterruptEnable (timerId, timerRecord[timerId].channelMode, timerRecord[timerId].intEnable);
  }
  else
  {
    return HAL_TIMER_NOT_CONFIGURED;
  }
  return HAL_TIMER_OK;
}

/***************************************************************************************************
 * @fn      HalTimerTick
 *
 * @brief   Check the counter for expired counter.
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
 void HalTimerTick ( void )
 {
    if (!timerRecord[HAL_TIMER_B].intEnable)
    {
      /* Counter3 - OC Mode */
      if (TBCCTL0 & CCIFG)
      {
        /* Clear CIF flag manually for polling */
        TBCCTL0 &= (CCIFG ^ 0xFFFF);
        /* Send callback */
        halTimerSendCallBack ( HAL_TIMER_B, HAL_TIMER_CHANNEL_A, HAL_TIMER_CH_MODE_OUTPUT_COMPARE);
      }
    }
}

/***************************************************************************************************
 * @fn      HalTimerStop
 *
 * @brief   Stop the Timer Service
 *
 * @param   timerId - ID of the timer
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerStop ( uint8 timerId )
{
  uint8 status = HAL_TIMER_OK;
  switch (timerId)
  {
    case HAL_TIMER_B:
      TBCTL |= MC_0;
      break;
    default:
      status = HAL_TIMER_INVALID_ID;
      break;
  }
  return (status);
}

/***************************************************************************************************
 * @fn      halTimerSetCount
 *
 * @brief   Stop the Timer Service
 *
 * @param   timerId - ID of the timer
 *          timerPerTick - Number micro sec per ticks
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 halTimerSetCount ( uint8 timerId, uint32 timePerTick )
{
  uint32 fll_adjust;
  uint16 count;

  /* FLL adjust. CPU clock may be generatd from 32KHz and may not be exactly 6 MHz.
   * first, calculate the difference to the base 6MHz in KHz.
   */
  fll_adjust = HAL_TIMER_FLL_ADJUST_KHZ - timerRecord[timerId].clock * 1000;

  /* then, add the base 6MHz to get the true clock speed in KHz */
  fll_adjust += (HAL_TIMER_MHZ * 1000);

  /* Load count = ((sec/tick) x clock) / prescale */
  count = (uint16)((timePerTick * fll_adjust / 1000) / timerRecord[timerId].prescaleVal);

  switch (timerId)
  {
    case HAL_TIMER_B:
      TBR     = 0;         /* Set counter */
      TBCTL  |= TBSSEL_2;  /* Set Clock type - SMCLK */
      TBCCR0  = count-1;   /* Set count */
      break;
    default:
      return HAL_TIMER_INVALID_ID;
  }
  return HAL_TIMER_OK;
}

/***************************************************************************************************
 * @fn      halTimerSetPrescale
 *
 * @brief   Stop the Timer Service
 *
 * @param   timerId - ID of the timer
 *          prescale - Prescale of the clock
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 halTimerSetPrescale ( uint8 timerId, uint8 prescale )
{
  switch (timerId)
  {
    case HAL_TIMER_B:
      TBCTL |= prescale;  /* Load prescalar */
      break;
    default:
      return HAL_TIMER_INVALID_ID;
  }

  return HAL_TIMER_OK;
}

/***************************************************************************************************
 * @fn      halTimerSetOpMode
 *
 * @brief   Setup operate modes
 *
 * @param   timerId - ID of the timer
 *          opMode - operation mode of the timer
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 halTimerSetOpMode ( uint8 timerId, uint8 opMode )
{
  uint8 status = HAL_TIMER_OK;

  switch (opMode)
  {
    case HAL_TIMER_MODE_NORMAL:
    case HAL_TIMER_MODE_CTC:
      if (timerId == HAL_TIMER_B)
      {
        TBCTL |= MC_1;  /* UP mode */
      }
      break;

    default:
      status = HAL_TIMER_INVALID_OP_MODE;
      break;
  }

  return (status);
}

/***************************************************************************************************
 * @fn      HalTimerInterruptEnable
 *
 * @brief   Enable or disable interrupt and set channel mode
 *
 * @param   timerId - ID of the timer
 *          channelMode - channel mode
 *          enable - TRUE or FALSE
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerInterruptEnable ( uint8 timerId, uint8 channelMode, bool enable )
{
  uint8 status = HAL_TIMER_OK;

  switch (channelMode)
  {

    case HAL_TIMER_CH_MODE_OUTPUT_COMPARE:

      /* Set to Compare mode */
      TBCCTL0 &= (CAP ^ 0xFFFF);

      if (enable)
      {
        /* Enable interrupt */
        if (timerId == HAL_TIMER_B)
          TBCCTL0 |= CCIE;
      }
      else
      {
        /* Disable interrupt */
        if (timerId == HAL_TIMER_B)
          TBCCTL0  &= (CCIE ^ 0xFFFF);
      }
      break;

    default:
      status = HAL_TIMER_INVALID_CH_MODE;
    break;
  }

  return (status);
}

/***************************************************************************************************
 * @fn      halTimerSendCallBack
 *
 * @brief   Send Callback back to the caller
 *
 * @param   timerId - ID of the timer
 *          channel - channel where the interrupt occurs
 *          channelMode - channel mode
 *
 *
 * @return  None
 ***************************************************************************************************/
void halTimerSendCallBack ( uint8 timerId, uint8 channel, uint8 channelMode )
{
  if (timerRecord[timerId].callBackFunc)
  {
    (timerRecord[timerId].callBackFunc) (timerId, channel, channelMode);
  }
}


/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/*
 *   Interrupt Service for the timerB, channel 0, output compare mode
 */
INTERRUPT_TIMERB_OC_CC0()
{
#if (defined HAL_TIMER) && (HAL_TIMER == TRUE)

#ifdef POWER_SAVING
  __low_power_mode_off_on_exit();
#endif
  if (timerRecord[HAL_TIMER_B].intEnable)
  {
    halTimerSendCallBack ( HAL_TIMER_B, HAL_TIMER_CHANNEL_A, HAL_TIMER_CH_MODE_OUTPUT_COMPARE);
  }

#endif /* (defined HAL_TIMER) && (HAL_TIMER == TRUE) */
}

/*
 *   Interrupt Service for the timerB, channel 1-6, output compare mode
 */
INTERRUPT_TIMERB_OC_CC1_6()
{
  /* Not doing anything for now */
}

/***************************************************************************************************
***************************************************************************************************/

