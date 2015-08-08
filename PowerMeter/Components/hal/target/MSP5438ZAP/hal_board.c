/**************************************************************************************************
  Filename:       hal_board.c
  Revised:        $Date:$
  Revision:       $Revision:$

  Description:    Board specific declarations
  Notes:          This version targets the Texas Instruments EXP5418


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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

#include "ZComDef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "MT.h"
#include "MT_SYS.h"
#include "DebugTrace.h"
#include "zap_app.h"

/* Hal */
#include "hal_lcd.h"
#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_key.h"
#include "hal_led.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task ID not initialized
#define NO_TASK_ID 0xFF

// Minimum length RAM "pattern" for Stack check
#define MIN_RAM_INIT 12

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 OnboardKeyIntEnable;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Registered keys task ID, initialized to NOT USED.
static uint8 registeredKeysTaskID = NO_TASK_ID;

static volatile uint16 halBrdTmrTick;
static volatile uint8 halBrdTmrDlyF;
static uint16 halBrdTmrDly;
static uint16 halBrdTmrTickShdw;
static uint8 halBrdDlySleep;

static UTCTime OSAL_timeSeconds;
static uint16 timeMSec;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void osalClockUpdate( uint16 elapsedMSec );

/*********************************************************************
 * @fn      osalClockUpdate
 *
 * @brief   Updates the OSAL Clock time with elapsed milliseconds.
 *
 * @param   elapsedMSec - elapsed milliseconds
 *
 * @return  none
 */
static void osalClockUpdate( uint16 elapsedMSec )
{
  // Add elapsed milliseconds to the saved millisecond portion of time
  timeMSec += elapsedMSec;

  // Roll up milliseconds to the number of seconds
  if ( timeMSec > 1000 )
  {
    OSAL_timeSeconds += timeMSec / 1000;
    timeMSec = timeMSec % 1000;
  }
}

/*********************************************************************
 * @fn      InitClock()
 * @brief   Initialize the CC2420DB Board Peripherals
 * @param   None
 * @return  None
 */
void InitClock(void)
{
  // Configure TimerA as 1-KHz HAL Board timer to drive OSAL timers and block waiting/sleeping.
  TA0CCR0 = HAL_CPU_CLOCK_MHZ * 1000;
  TA0CTL = TASSEL_2 + MC_1;  // SMCLK, upmode
  TA0CCTL0 = CCIE;
}

/*********************************************************************
 * @fn      InitBoard()
 * @brief   Initialize the CC2420DB Board Peripherals
 * @param   None
 * @return  None
 */
void InitBoard(void)
{
  OnboardKeyIntEnable = HAL_KEY_INTERRUPT_DISABLE;
  HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);
}

/*********************************************************************
 * Keyboard Register function
 *
 * The keyboard handler is setup to send all keyboard changes to
 * one task (if a task is registered).
 *
 * If a task registers, it will get all the keys. You can change this
 * to register for individual keys.
 *********************************************************************/
void RegisterForKeys( uint8 task_id )
{
  registeredKeysTaskID = task_id;
}

/*********************************************************************
 * @fn      OnBoard_SendKeys
 *
 * @brief   Send "Key Pressed" message to application.
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  status
 *********************************************************************/
uint8 OnBoard_SendKeys( uint8 keys, uint8 state )
{
  keyChange_t *msgPtr;

  if ( registeredKeysTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (keyChange_t *)osal_msg_allocate( sizeof(keyChange_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = KEY_CHANGE;
      msgPtr->state = state;
      msgPtr->keys = keys;

      osal_msg_send( registeredKeysTaskID, (uint8 *)msgPtr );
    }
    return ( ZSuccess );
  }
  else
    return ( ZFailure );
}

/*********************************************************************
 * @fn      OnBoard_KeyCallback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 *********************************************************************/
void OnBoard_KeyCallback ( uint8 keys, uint8 state )
{
  uint8 shift;
  (void)state;

  shift = (keys & HAL_KEY_SW_6) ? true : false;

  if ( OnBoard_SendKeys( keys, shift ) != ZSuccess )
  {
    // Process SW1 here
    if ( keys & HAL_KEY_SW_1 )  // Switch 1
    {
    }
    // Process SW2 here
    if ( keys & HAL_KEY_SW_2 )  // Switch 2
    {
    }
    // Process SW3 here
    if ( keys & HAL_KEY_SW_3 )  // Switch 3
    {
    }
    // Process SW4 here
    if ( keys & HAL_KEY_SW_4 )  // Switch 4
    {
    }
    // Process SW5 here
    if ( keys & HAL_KEY_SW_5 )  // Switch 5
    {
    }
    // Process SW6 here
    if ( keys & HAL_KEY_SW_6 )  // Switch 6
    {
    }
  }
}

/*********************************************************************
 * @fn      OnBoard_stack_used
 *
 * @brief   Runs through the stack looking for touched memory.
 *
 * @param   none
 *
 * @return  Maximum number of bytes used by the stack.
 *********************************************************************/
uint16 OnBoard_stack_used(void)
{
  uint8 const *ptr;
  uint8 cnt = 0;

  for (ptr = CSTACK_END; ptr > CSTACK_BEG; ptr--)
  {
    if (STACK_INIT_VALUE == *ptr)
    {
      if (++cnt >= MIN_RAM_INIT)
      {
        ptr += MIN_RAM_INIT;
        break;
      }
    }
    else
    {
      cnt = 0;
    }
  }

  return (uint16)(CSTACK_END - ptr + 1);
}

/*********************************************************************
 * @fn      _itoa
 *
 * @brief   convert a 16bit number to ASCII
 *
 * @param   num -
 *          buf -
 *          radix -
 *
 * @return  void
 *
 *********************************************************************/
void _itoa(uint16 num, uint8 *buf, uint8 radix)
{
  char c,i;
  uint8 *p, rst[5];

  p = rst;
  for ( i=0; i<5; i++,p++ )
  {
    c = num % radix;  // Isolate a digit
    *p = c + (( c < 10 ) ? '0' : '7');  // Convert to Ascii
    num /= radix;
    if ( !num )
      break;
  }

  for ( c=0 ; c<=i; c++ )
    *buf++ = *p--;  // Reverse character order

  *buf = '\0';
}

/*********************************************************************
 * @fn        Onboard_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  uint16 - new random number
 *
 *********************************************************************/
uint16 Onboard_rand( void )
{
  uint16 randy;

  zapSysReq(MT_SYS_RANDOM, (uint8 *)&randy, NULL);
  return randy;
}

/*********************************************************************
 * @fn        Onboard_wait
 *
 * @brief    Delay wait
 *
 * @param   uint16 - time to wait
 *
 * @return  none
 *
 *********************************************************************/
void Onboard_wait( uint16 timeout )
{
  // TODO: MSP430 Port required
  while (timeout--)
  {
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  }
}

/*********************************************************************
 * @fn      osalTimeUpdate
 *
 * @brief   Uses the free running rollover count of the ZAP timer.
 *          This function is intended to be invoked from the background, not interrupt level.
 *
 * @param   None.
 *
 * @return  None.
 */
void osalTimeUpdate(void)
{
  uint16 tmp = halBrdTmrTick;

  if (tmp != halBrdTmrTickShdw)
  {
    // Calculate the elapsed ticks of the free-running timer.
    uint16 elapsedMSec = tmp - halBrdTmrTickShdw;
  
    // Store the ZAP Timer tick count for the next time through this function.
    halBrdTmrTickShdw = tmp;
  
    osalTimerUpdate(elapsedMSec);
    osalClockUpdate(elapsedMSec);
  }
}

/*********************************************************************
 * @fn      osal_getClock
 *
 * @brief   Gets the current time.  This will only return the seconds
 *          portion of time and doesn't include the factional second
 *          counter.
 *
 * @param   none
 *
 * @return  number of seconds since 0 hrs, 0 minutes, 0 seconds,
 *          on the 1st of January 2000 UTC
 */
UTCTime osal_getClock( void )
{
  return ( OSAL_timeSeconds );
}

void HalBoardDelay(uint16 delay, uint8 sleep)
{
  halBrdTmrDly = halBrdTmrTick + delay + 1;
  halBrdTmrDlyF = TRUE;

  if (sleep)
  {
    halBrdDlySleep = TRUE;
  }
  __low_power_mode_1();

  halBrdDlySleep = FALSE;
}

uint8 HalBoardDelayed(void)
{
  return halBrdTmrDlyF;
}

// Timer A0 interrupt service routine
HAL_ISR_FUNCTION(HalBoardTimerRolloverIsr, TIMER0_A0_VECTOR)
{
  if (++halBrdTmrTick == halBrdTmrDly)
  {
    halBrdTmrDlyF = FALSE;
    __low_power_mode_off_on_exit();
  }
  if (!halBrdDlySleep)
  {
    __low_power_mode_off_on_exit();
  }
}

HAL_ISR_FUNCTION(HalBoardPort1Isr, PORT1_VECTOR)
{
  P1IFG = 0;
  __low_power_mode_off_on_exit();
}

/*********************************************************************
*********************************************************************/
