/**************************************************************************************************
  Filename:       MSP430FlashUtil.c
  Revised:        $Date: 2008-07-14 13:59:17 -0700 (Mon, 14 Jul 2008) $
  Revision:       $Revision: 17524 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2007 Texas Instruments Incorporated. All rights reserved.

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

//****************************************************************************
//  MSP-FET430P140 Demo - Flash In-System Programming, Copy SegA to SegB
//
//  Description: This program first erases flash seg A, then it increments all
//  values in seg A, then it erases seg B, then  copies seg A to seg B.
//  Assumed MCLK 550kHz - 900kHz.
//  //* Set Breakpoint on NOP in the Mainloop to avoid Stressing Flash *//
//
//               MSP430F149
//            -----------------
//        /|\|              XIN|-
//         | |                 |
//         --|RST          XOUT|-
//           |                 |
//
//  Texas Instruments Inc.
//  Feb 2005
//  Built with IAR Embedded Workbench Version: 3.21A
//******************************************************************************

#include "hal_board_cfg.h"
#include "hal_types.h"
#include "MSP430FlashUtil.h"

#define STATIC  static

STATIC uint8 s_savWDT;

STATIC halIntState_t fuIntState;

// Function prototypes
STATIC void initFlash(void);
STATIC void doneFlash(void);

STATIC void initFlash(void)
{
  HAL_ENTER_CRITICAL_SECTION( fuIntState );  // Hold off interrupts.

  s_savWDT = WDTCTL & 0xFF;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

#if defined FCTL2
  FCTL2 = FWKEY + FSSEL_1 + FN4 + FN3;  // Flash Timing Generator as MCLK/24 -> 333 kHz.
#endif

#if defined HAL_BOARD_F5438
  if (FCTL3 & LOCKA)
  {
    FCTL3 = FWKEY + LOCKA;  // Clear the user info pages lock for writing the IEEE.
  }
#endif
}

STATIC void doneFlash()
{
  // restore WDT
  WDTCTL = WDTPW + s_savWDT;

  HAL_EXIT_CRITICAL_SECTION( fuIntState );   // Re-enable interrupts.

  return;
}

void flashErasePage(uint8 *addr)
{
  initFlash();

  FCTL1 = FWKEY + ERASE;               // Set Erase bit
  FCTL3 = FWKEY;                       // Clear Lock bit
  *addr = 0;                           // Dummy write to erase Flash segment
  FCTL1 = FWKEY;                       // Clear ERASE bit
  FCTL3 = FWKEY + LOCK;                // Set LOCK bit

  doneFlash();
  return;
}

void flashWrite( uint8 *addr, uint16 len, uint8 *buf )
{
  initFlash();

  FCTL3 = FWKEY;                // Clear Lock bit
  FCTL1 = FWKEY + WRT;          // Set WRT bit for write operation

  while ( len-- )
  {
    *addr++ = *buf++;            // Write value to flash
  }

  FCTL1 = FWKEY;                // Clear WRT bit
  FCTL3 = FWKEY + LOCK;         // Set LOCK bit

  doneFlash();
}

/*********************************************************************
*********************************************************************/
