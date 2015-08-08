/**************************************************************************************************
  Filename:       hal_mcu.h
  Revised:        $Date: 2013-10-07 09:18:37 -0700 (Mon, 07 Oct 2013) $
  Revision:       $Revision: 35584 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef HAL_MCU_H
#define HAL_MCU_H

/*
 *  Targets : Texas Instruments MSP430x26x or MSP430x54x
 *
 */


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_defs.h"
#include "hal_types.h"


/* ------------------------------------------------------------------------------------------------
 *                                        Target Defines
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_MCU_MSP430


/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ---------------------- IAR Compiler ---------------------- */
#ifdef __IAR_SYSTEMS_ICC__
#include <msp430.h>
#define HAL_COMPILER_IAR
#define HAL_MCU_LITTLE_ENDIAN()   1
#define _PRAGMA(x) _Pragma(#x)
#define HAL_ISR_FUNC_DECLARATION(f,v) _PRAGMA(vector=v) __interrupt void f(void)
#define HAL_ISR_FUNC_PROTOTYPE(f,v)   _PRAGMA(vector=v) __interrupt void f(void)
#define HAL_ISR_FUNCTION(f,v)         HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)

/* ------------------ Unrecognized Compiler ------------------ */
#else
#error "ERROR: Unknown compiler."
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       Interrupt Macros
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_ENABLE_INTERRUPTS()         asm("eint")
#define HAL_DISABLE_INTERRUPTS()        st( asm("dint"); asm("nop"); )
#define HAL_INTERRUPTS_ARE_ENABLED()    (__get_SR_register() & GIE)

typedef istate_t halIntState_t;
#define HAL_ENTER_CRITICAL_SECTION(x)   st( x = __get_interrupt_state();  HAL_DISABLE_INTERRUPTS(); )
#define HAL_EXIT_CRITICAL_SECTION(x)    st( __set_interrupt_state(x); )
#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t s; HAL_ENTER_CRITICAL_SECTION(s); x; HAL_EXIT_CRITICAL_SECTION(s); )

/* Dummy for this platform */
#define HAL_AES_ENTER_WORKAROUND() 
#define HAL_AES_EXIT_WORKAROUND() 

/* ------------------------------------------------------------------------------------------------
 *                                        Reset Macro
 * ------------------------------------------------------------------------------------------------
 */
#ifdef __msp430x26x
#define HAL_SYSTEM_RESET()              st( WDTCTL = (WDTPW ^ 0xFFFF); )  /* force immediate system reset */
#else
#define HAL_SYSTEM_RESET()              st( PMMCTL0 = 0xA504; )  /* force immediate BOR system reset */
#endif

/* ------------------------------------------------------------------------------------------------
 *                                     Simulated MCU real-time clock
 * ------------------------------------------------------------------------------------------------
 */
extern uint32 macMcuPrecisionCount(void);
#define MCU_SIMULATED_REAL_TIME_CLOCK(x) st(x = macMcuPrecisionCount();)

extern void macMcuPrecisionCountSleepUpdate(uint32 elapsed_time);

/* ------------------------------------------------------------------------------------------------
 *                                     Sleep macros
 * ------------------------------------------------------------------------------------------------
 */
#if defined( POWER_SAVING )
/* To be used in SoC only. Stub here. */
#define ALLOW_SLEEP_MODE()
#endif
  
/**************************************************************************************************
 */
#endif
