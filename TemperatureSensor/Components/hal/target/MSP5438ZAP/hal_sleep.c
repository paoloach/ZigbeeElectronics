/**************************************************************************************************
  Filename:       hal_sleep.c
  Revised:        $Date: 2013-12-10 07:42:48 -0800 (Tue, 10 Dec 2013) $
  Revision:       $Revision: 36527 $

  Description:    This module contains the HAL power management procedures for the MSP430.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <intrinsics.h>
#include "hal_types.h"
#include "hal_mcu.h"
#include "hal_board.h"
#include "hal_sleep.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_drivers.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* POWER CONSERVATION DEFINITIONS
 * Sleep mode definitions (enabled with POWER_SAVING compile option)
 */
#define MSP430_AM             0        /* All clocks are active */
#define MSP430_LPM3           1        /* CPU is disabled; MCLK, FLL+ loop control, and DCOCLK are disabled;
                                        * DCO's dc-generator is disabled; ACLK remains active */
#define MSP430_LPM4           2        /* In addition to LPM3, Crystal oscillator is stopped */

/* HAL power management mode is set according to the power management state. The default
 * setting is HAL_SLEEP_OFF. The actual value is tailored to different HW platform. Both
 * HAL_SLEEP_TIMER and HAL_SLEEP_DEEP selections will:
 *   1. turn off the system clock, and
 *   2. halt the MCU.
 * HAL_SLEEP_TIMER can be woken up by sleep timer interrupt, I/O interrupt and reset.
 * HAL_SLEEP_DEEP can be woken up by I/O interrupt and reset.
 */
#define HAL_SLEEP_OFF         MSP430_AM
#define HAL_SLEEP_TIMER       MSP430_LPM3
#define HAL_SLEEP_DEEP        MSP430_LPM4

/* minimum time to sleep */
#define MIN_SLEEP_TIME        14

/* MAX_SLEEP_COUNT calculation:
 * 16-bi MAC timer maximum sleep duration = 0xFFFF/(32.768KHz/8) = 16 seconds
 */
#define MAX_SLEEP_COUNT                  0xFFFF              /* maximum sleep count allowed by H/W */

/* set MSP430 power mode */
/* sleep with wakeup timer */
#define HAL_SLEEP_TIMER_SLEEP()  __low_power_mode_1()
/* Cannot turn off interrupt for timer sleep */
#define HAL_SLEEP_DISABLE_INTERRUPTS()

/* sleep without timers */
//#define HAL_SLEEP_TIMER_SLEEP()  __low_power_mode_3()
//#define HAL_SLEEP_DISABLE_INTERRUPTS()      HAL_DISABLE_INTERRUPTS()

#define HAL_SLEEP_SET_POWER_MODE(mode)      st( if ( mode == MSP430_LPM3 ) { HAL_SLEEP_TIMER_SLEEP(); }   \
                                                else if ( mode == MSP430_LPM4 ) { __low_power_mode_4(); } \
                                              )

/* sleep timer interrupt control */
#define HAL_SLEEP_TIMER_ENABLE_INT()        MAC_RADIO_TIMER_WAKE_UP()
#define HAL_SLEEP_TIMER_DISABLE_INT()       MAC_RADIO_TIMER_SLEEP()

/* convert msec to 320 usec units with round */
#define HAL_SLEEP_MS_TO_320US(ms)           (((((uint32) (ms)) * 100) + 31) / 32)

/* conver 320 usec units to msec with round */
#define HAL_SLEEP_320US_TO_MS(u320)         (((((uint32) (u320)) * 8) + 24) / 25)

/* ------------------------------------------------------------------------------------------------
 *                                        Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* HAL power management mode is set according to the power management state.
 */
static uint8 halPwrMgtMode = HAL_SLEEP_OFF;

/* stores the sleep timer count upon entering sleep */
static uint16 halSleepTimerStart;

/* stores the sleep timer compare count upon entering sleep */
static uint16 halSleepTimerCompareStart;

/* stores the accumulated sleep time */
static uint32 halAccumulatedSleepTime;

/* ------------------------------------------------------------------------------------------------
 *                                      Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */

void halSleepSetTimer( uint32 timeout );
uint32 halMacTimerElapsed( uint32 *timeout );

/**************************************************************************************************
 * @fn          halSleep
 *
 * @brief       This function is called from the OSAL task loop using and existing OSAL
 *              interface.  It sets the low power mode of the MAC and MCU.
 *
 * input parameters
 *
 * @param       osal_timeout - Next OSAL timer timeout.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halSleep( uint16 osal_timeout )
{
  uint32        timeout;
  uint32        macTimeout;

  /* Avoid using critical section macros because low power mode macros.
   * Assert if we enter sleep without interrupt enabled.
   */
  HAL_ASSERT( HAL_INTERRUPTS_ARE_ENABLED() );

  /* Don't sleep if time too short or if a key is pressed */
  if ( osal_timeout && (osal_timeout < MIN_SLEEP_TIME) )
    return;

  /* initialize the accumulated sleep time */
  halAccumulatedSleepTime = 0;

  /* get next OSAL timer expiration converted to 320 usec units */
  timeout = HAL_SLEEP_MS_TO_320US(osal_timeout);
  if (timeout == 0)
  {
    timeout = MAC_PwrNextTimeout();
  }
  else
  {
    /* get next MAC timer expiration */
    macTimeout = MAC_PwrNextTimeout();

    /* get lesser of two timeouts */
    if ((macTimeout != 0) && (macTimeout < timeout))
    {
      timeout = macTimeout;
    }
  }

  /* Adjust the wakup time so that the radio is awake in time
   * for beacon transmission.
   */
  if ( timeout > HAL_BEACON_ENABLE_EARLY_WAKEUP() )
  {
    timeout -= HAL_BEACON_ENABLE_EARLY_WAKEUP();
  }

  /* HAL_SLEEP_PM2 is entered only if the timeout is zero */
  halPwrMgtMode = (timeout == 0)? HAL_SLEEP_DEEP : HAL_SLEEP_TIMER;

  if ( (timeout > HAL_SLEEP_MS_TO_320US(MIN_SLEEP_TIME)) || (timeout == 0) )
  {

    HAL_SLEEP_DISABLE_INTERRUPTS();

    /* always use "deep sleep" to turn off radio VREG on MSP430 */
    //if (MAC_PwrOffReq(MAC_PWR_SLEEP_DEEP) == MAC_SUCCESS)
    {
      /* Shut down LED */
      HalLedEnterSleep();

      while ( (HAL_SLEEP_MS_TO_320US(halAccumulatedSleepTime) < timeout) || (timeout == 0) )
      {
        /* enable sleep timer interrupt */
        if (timeout != 0)
        {
          /* set sleep timer, timeout is adjusted for the next time */
          halSleepSetTimer( timeout );

          /* set up sleep timer interrupt */
          HAL_SLEEP_TIMER_ENABLE_INT();
        }

        HalKeyEnterSleep();

        /* set MSP430 power mode, global interrupt will be enabled in the macro */
        HAL_SLEEP_SET_POWER_MODE(halPwrMgtMode);
        /* wake up from sleep in ISR */

        HAL_SLEEP_DISABLE_INTERRUPTS();

        /* disable sleep timer interrupt */
        HAL_SLEEP_TIMER_DISABLE_INT();

        if (timeout != 0)
        {
          /* Calculate timer elapsed only if timer sleep */
          halAccumulatedSleepTime += halMacTimerElapsed( &timeout );
        }

        /* Process keyboard "wake-up" interrupt, exit while loop if key interrupt */
        if ( HalKeyExitSleep() || timeout == 0 )
        {
          break;
        }
      }

      /* Restart the LED */
      HalLedExitSleep();

      /* power on the MAC; blocks until completion.
       * the MAC timer will be turned on if off.
       */
      //MAC_PwrOnReq();

      /* adjust OSAL timers */
      osal_adjust_timers();

      HAL_ENABLE_INTERRUPTS();
    }
    else
    {
      HAL_ENABLE_INTERRUPTS();
    }
  }
}

/**************************************************************************************************
 * @fn          halSleepSetTimer
 *
 * @brief       This function sets the MSP430 MAC timer compare value.  First it reads and
 *              stores the value of the sleep timer; this value is used later to update MAC
 *              timers.  Then the timeout value is converted from 320 usec units to system clock
 *              period units and the compare value is set to the timeout.
 *
 * input parameters
 *
 * @param       timeout - Pointer to the timeout value in 320 usec units.  The sleep timer compare
 *                        is set to this value.
 *
 * output parameters - none
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halSleepSetTimer( uint32 timeout )
{
  halIntState_t  s;
  uint32         ticks;

  HAL_ENTER_CRITICAL_SECTION(s);

#ifdef __msp430x54x
  /* read the current MAC timer and store value for later */
  ticks = halSleepTimerStart = HAL_MAC_SLEEP_TIMER_TAR();
#endif

  /* Switch MAC timer clock source and change prescaler
   * halt the timer while updating
   */
  HAL_MAC_SLEEP_TIMER_SLOW_DOWN();

  HAL_EXIT_CRITICAL_SECTION(s);

#ifdef __msp430x54x
  /* With MSP430F5438 RTM silicon, when the counter mode is moved from 'up'
   * to 'stop' and the timer counter equals the capture/compare value in a
   * TACCRx register the transition of modes will pull the TAR bits (15:4)
   * to a '1' momentarily. Re-initialize TAR as a temporary workaround until
   * the post-RTM revisions of silicon with this bug fixed.
   */
  HAL_MAC_SLEEP_TIMER_TAR() = halSleepTimerStart;
#else
  /* read the current MAC timer and store value for later */
  ticks = halSleepTimerStart = HAL_MAC_SLEEP_TIMER_TAR();
#endif

  /* read the current MAC timer compare count and store value for later */
  halSleepTimerCompareStart = HAL_MAC_SLEEP_TIMER_COMPARE();

  /* The MAC tick is 32KHz/8 = 4096Hz. The ratio of 4096Hz ticks
   * to 320 usec ticks is (320e-6)/(1/4096) = 1.31072 = 4096 / 3125.
   */
  ticks += (timeout * 4096 / 3125);

  /* sleep away one chunk at a time */
  if ( ticks > MAX_SLEEP_COUNT )
  {
    /* adjust timeout for the next sleep cycle */
    ticks = MAX_SLEEP_COUNT;
  }

  /* set MAC timer compare */
  HAL_MAC_SLEEP_TIMER_SET_COMPARE(((uint16)ticks));

  /* restart the timer */
  HAL_MAC_SLEEP_TIMER_RESTART();
}

/**************************************************************************************************
 * @fn          TimerElapsed
 *
 * @brief       Determine the number of MAC timer ticks elapsed during sleep.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      Number of msec elapsed during sleep.
 **************************************************************************************************
 */
uint32 TimerElapsed( void )
{
  return ( HAL_SLEEP_320US_TO_MS(halAccumulatedSleepTime) );
}

/**************************************************************************************************
 * @fn          halMacTimerElapsed
 *
 * @brief       Determine the number of MAC timer ticks elapsed during sleep.
 *
 * input parameters
 *
 * @param       timeout - pointer to the 320us timeout.
 *
 * output parameters
 *
 * None.
 *
 * @return      Number of timer ticks elapsed during sleep.
 **************************************************************************************************
 */
uint32 halMacTimerElapsed( uint32 *timeout )
{
  volatile uint16 tar;
  volatile uint16 taccr;
  uint32          ticks;
  uint32          timeout_u320; /* 320 us timeout */
  halIntState_t   s;

  /* read the currrent count + compare count (MAC timer must be in CTC mode) */
  HAL_ENTER_CRITICAL_SECTION(s);

#ifdef __msp430x54x
  tar = HAL_MAC_SLEEP_TIMER_TAR();
#endif

  /* Halt timer before read anything */
  HAL_MAC_SLEEP_TIMER_SPEED_UP();

  HAL_EXIT_CRITICAL_SECTION(s);

#ifdef __msp430x54x
  /* With MSP430F5438 RTM silicon, when the counter mode is moved from 'up'
   * to 'stop' and the timer counter equals the capture/compare value in a
   * TACCRx register the transition of modes will pull the TAR bits (15:4)
   * to a '1' momentarily. Re-initialize TAR as a temporary workaround until
   * the post-RTM revisions of silicon with this bug fixed.
   */
  HAL_MAC_SLEEP_TIMER_TAR() = tar;
#else
  /* Read timer registers */
  tar = HAL_MAC_SLEEP_TIMER_TAR();
#endif

  taccr = HAL_MAC_SLEEP_TIMER_COMPARE();

  /* store current timer count */
  ticks = tar;

  /* calculate elapsed time or somthing else woke up the MCU */
  if ( taccr <= halSleepTimerStart )
  {
    ticks += taccr;
  }
  ticks -= halSleepTimerStart;

  /* adjust timeout for the next sleep cycle */
  timeout_u320 = ticks * 3125 / 4096;
  if ( *timeout >= timeout_u320 )
  {
    *timeout -= timeout_u320;
  }

  /* Restore current MAC count */
  HAL_MAC_SLEEP_TIMER_SET_COMPARE(halSleepTimerCompareStart);
  HAL_MAC_SLEEP_TIMER_TAR() = halSleepTimerStart;

  /* Restart timer */
  HAL_MAC_SLEEP_TIMER_RESTART();

  /* Return elapsed time in units of 320us */
  return ( timeout_u320 );
}

/**************************************************************************************************
 * @fn          halRestoreSleepLevel
 *
 * @brief       Restore the deepest timer sleep level (used in CC2430). This is a stub here.
 *
 * input parameters
 *
 * @param       None
 *
 * output parameters
 *
 *              None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halRestoreSleepLevel( void )
{
  /* Stub */
}
