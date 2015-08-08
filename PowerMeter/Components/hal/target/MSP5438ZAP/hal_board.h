/**************************************************************************************************
  Filename:       hal_board.h
  Revised:        $Date: 2010-11-22 08:13:43 -0800 (Mon, 22 Nov 2010) $
  Revision:       $Revision: 24480 $

  Description:    Board specific declarations
  Notes:          This version targets the Texas Instruments EXP5438


  Copyright 2009-2010 Texas Instruments Incorporated. All rights reserved.

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
#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <intrinsics.h>

#include "hal_board_cfg.h"
#include "hal_mcu.h"
#include "hal_uart.h"
#include "hal_sleep.h"
#include "osal.h"
#include "zap_app.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

#undef HAL_MCU_MSP430  // Preclude hal_assert.c from including MAC header files.

/*********************************************************************
 * CONSTANTS
 */

#define aExtendedAddress  znpIEEE

// Timer clock and power-saving definitions
#define TICK_COUNT         1  // TIMAC requires this number to be 1

/* OSAL timer defines */
#define TICK_TIME   1000   // Timer per tick - in micro-sec

// Restart system from absolute beginning
// Disables interrupts, forces WatchDog reset
#define HalReset()          \
{                           \
  HAL_DISABLE_INTERRUPTS(); \
  HAL_SYSTEM_RESET();       \
}

/* Reset reason for reset indication */
#define ResetReason() 0x00  /* TODO: Port needed. */

// Wait for specified microseconds
#define MicroWait(t) Onboard_wait(t)

#define OSAL_SET_CPU_INTO_SLEEP(timeout) halSleep(timeout);  /* Called from OSAL_PwrMgr */

#define HAL_BOARD_WAIT_MODE()  __low_power_mode_1()

#ifdef __IAR_SYSTEMS_ICC__
// Internal (MCU) Stack addresses
#define CSTACK_BEG ((uint8 const *)(_Pragma("segment=\"CSTACK\"") __segment_begin("CSTACK")))
#define CSTACK_END ((uint8 const *)(_Pragma("segment=\"CSTACK\"") __segment_end("CSTACK"))-1)
// Stack Initialization Value
#define STACK_INIT_VALUE  0xCD
#else
#error Check compiler compatibility.
#endif

/* The following Heap sizes are setup for typical TI sample applications,
 * and should be adjusted to your systems requirements.
 */
#if !defined INT_HEAP_LEN
  #define INT_HEAP_LEN  4096
#endif
#define MAXMEMHEAP INT_HEAP_LEN

#define KEY_CHANGE_SHIFT_IDX 1
#define KEY_CHANGE_KEYS_IDX  2

// Initialization levels
#define OB_COLD  0
#define OB_WARM  1
#define OB_READY 2

#ifdef LCD_SUPPORTED
  #define BUZZER_OFF  0
  #define BUZZER_ON   1
  #define BUZZER_BLIP 2
#endif

typedef struct
{
  osal_event_hdr_t hdr;
  uint8 state; // shift
  uint8 keys;  // keys
} keyChange_t;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */

  /*
   * System / OSAL clock.
   */
  extern void InitClock(void);

  /*
   * Final board initialization
   */
  extern void InitBoard(void);

 /*
  * Get elapsed timer clock counts
  */
  extern uint32 TimerElapsed( void );

  /*
   * Register for all key events
   */
  extern void RegisterForKeys( uint8 task_id );

  /*
   * Send "Key Pressed" message to application
   */
  extern uint8 OnBoard_SendKeys( uint8 keys, uint8 shift );

  /*
   * Convert an interger to an ascii string
   */
  extern void _itoa( uint16 num, uint8 *buf, uint8 radix );

  /*
   * Calculate the size of used stack
   */
  extern uint16 OnBoard_stack_used( void );

  /*
   * Callback routine to handle keys
   */
  extern void OnBoard_KeyCallback ( uint8 keys, uint8 state );

  /*
   * Board specific random number generator
   */
  extern uint16 Onboard_rand( void );

  /*
   * Board specific micro-second wait
   */
  extern void Onboard_wait( uint16 timeout );

  void HalBoardDelay(uint16 delay, uint8 sleep);
  uint8 HalBoardDelayed(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // HAL_BOARD_H
