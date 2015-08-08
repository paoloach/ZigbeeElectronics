/**************************************************************************************************
Filename:       hal_dco.c
Revised:        $Date: 2009-12-22 12:38:28 -0800 (Tue, 22 Dec 2009) $
Revision:       $Revision: 21387 $

Description:    This file contains the interface to the DCO Service.

Copyright 2008-2009 Texas Instruments Incorporated. All rights reserved.

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

/**************************************************************************************************
*                                            INCLUDES
**************************************************************************************************/
#include "hal_types.h"
#include "hal_dco.h"
#include "hal_mcu.h"

/**************************************************************************************************
*                                            CONSTANTS
**************************************************************************************************/


/**************************************************************************************************
*                                              MACROS
**************************************************************************************************/


/**************************************************************************************************
*                                            TYPEDEFS
**************************************************************************************************/
#ifndef FLASH28_WORKAROUND
#define FLASH28_WORKAROUND  FALSE
#endif

/**************************************************************************************************
*                                        GLOBAL VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                        EXTERNAL VARIABLES
**************************************************************************************************/


/**************************************************************************************************
 *                                       Local Function Prototypes
 **************************************************************************************************/
static void halBoardGetSystemClockSettings(uint8  systemClockSpeed,
                                           uint8  *setDcoRange,
                                           uint8  *setVCore,
                                           uint16 *setMultiplier);
static void halBoardSetVCoreUp   (uint8 VCore);
static void halBoardSetVCoreDown (uint8 VCore);
static void halBoardSetVCore     (uint8 VCore);


/**************************************************************************************************
* @fn      halBoardSetSystemClock
*
* @brief   Configure DCO for system clock
*
* @param   systemClockSpeed
*
* @return  None
**************************************************************************************************/
void halBoardSetSystemClock(uint8 systemClockSpeed)
{
  uint8  setDcoRange, setVCore;
  uint16 setMultiplier;

  halBoardGetSystemClockSettings( systemClockSpeed, &setDcoRange, &setVCore, &setMultiplier);

  halBoardSetVCore( setVCore );
  UCSCTL0 = 0x00;                           /* Set lowest possible DCOx, MODx */
  UCSCTL1 = setDcoRange;                    /* Select suitable range */

  UCSCTL2 = setMultiplier + FLLD_1;         /* Set DCO Multiplier */
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV ;
}


/**************************************************************************************************
* @fn      halBoardGetSystemClockSettings
*
* @brief   Get the DCO configuration parameters
*
* @param    systemClockSpeed: range from 1 to 25MHz
*          *setDcoRange:      DCO select
*          *setVCore:         VCORE settings
*          *setMultiplier:    Multiplier settings
*
* @return  None
**************************************************************************************************/
void halBoardGetSystemClockSettings(uint8   systemClockSpeed,
                                    uint8  *setDcoRange,
                                    uint8  *setVCore,
                                    uint16 *setMultiplier)
{
  switch (systemClockSpeed)
  {
  case SYSCLK_1MHZ:
    *setDcoRange = DCORSEL_1MHZ;
    *setVCore = VCORE_1MHZ;
    *setMultiplier = DCO_MULT_1MHZ;
    break;
  case SYSCLK_4MHZ:
    *setDcoRange = DCORSEL_4MHZ;
    *setVCore = VCORE_4MHZ;
    *setMultiplier = DCO_MULT_4MHZ;
    break;
  case SYSCLK_8MHZ:
    *setDcoRange = DCORSEL_8MHZ;
    *setVCore = VCORE_8MHZ;
    *setMultiplier = DCO_MULT_8MHZ;
    break;
  case SYSCLK_12MHZ:
    *setDcoRange = DCORSEL_12MHZ;
    *setVCore = VCORE_12MHZ;
    *setMultiplier = DCO_MULT_12MHZ;
    break;
  case SYSCLK_16MHZ:
    *setDcoRange = DCORSEL_16MHZ;
    *setVCore = VCORE_16MHZ;
    *setMultiplier = DCO_MULT_16MHZ;
    break;
  case SYSCLK_18MHZ:
    *setDcoRange = DCORSEL_18MHZ;
    *setVCore = VCORE_18MHZ;
    *setMultiplier = DCO_MULT_18MHZ;
    break;
  case SYSCLK_20MHZ:
    *setDcoRange = DCORSEL_20MHZ;
    *setVCore = VCORE_20MHZ;
    *setMultiplier = DCO_MULT_20MHZ;
    break;
  case SYSCLK_25MHZ:
    *setDcoRange = DCORSEL_25MHZ;
    *setVCore = VCORE_25MHZ;
    *setMultiplier = DCO_MULT_25MHZ;
    break;
  }
}


/**************************************************************************************************
* @fn      halBoardSetVCoreUp
*
* @brief   Tuning Voltage up
*
* @param   level: VCORE setting level
*
* @return  None
**************************************************************************************************/
void halBoardSetVCoreUp (uint8 level)
{
  PMMCTL0_H = 0xA5;                         /* Open PMM module registers for write access */

  /* Set SVS/M high side to new level */
  SVSMHCTL = (SVSMHCTL & ((SVSHRVL0*3 + SVSMHRRL0) ^ 0xFFFF)) | (SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level);
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;  /* Set SVM new Level */
  SVSMLCTL = (SVSMLCTL & (SVSMLRRL_3 ^ 0xFFFF)) | (SVMLE + SVSMLRRL0 * level); /* Set SVS/M low side to new level */
  while ((PMMIFG & SVSMLDLYIFG) == 0);      /* Wait till SVM is settled (Delay) */
  PMMCTL0_L = PMMCOREV0 * level;            /* Set VCore to x */
  PMMIFG &= ((SVMLVLRIFG + SVMLIFG) ^ 0xFFFF);        /* Clear already set flags */
  if ((PMMIFG & SVMLIFG))
  {
    while ((PMMIFG & SVMLVLRIFG) == 0);     /* Wait till level is reached */
  }

  /* Set SVS/M Low side to new level */
  SVSMLCTL = (SVSMLCTL & ~((SVSLRVL0*3 + SVSMLRRL_3) & 0xFFFF)) | (SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level);
  PMMCTL0_H = 0x00;                         /* Lock PMM module registers for write access */
}


/**************************************************************************************************
* @fn      halBoardSetVCoreDown
*
* @brief   Tuning Voltage down
*
* @param   level: VCORE setting level
*
* @return  None
**************************************************************************************************/
void halBoardSetVCoreDown (uint8 level)
{
  PMMCTL0_H = 0xA5;                         /* Open PMM module registers for write access */
  /* Set SVS/M low side to new level */
  SVSMLCTL = (SVSMLCTL & ((SVSLRVL0*3 + SVSMLRRL_3) ^ 0xFFFF)) | (SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level);
  while ((PMMIFG & SVSMLDLYIFG) == 0);      /* Wait till SVM is settled (Delay) */
  PMMCTL0_L = (level * PMMCOREV0);          /* Set VCore to 1.85 V for Max Speed. */
  PMMCTL0_H = 0x00;                         /* Lock PMM module registers for write access */
}


/**************************************************************************************************
* @fn      halBoardSetVCore
*
* @brief   Setting up VCO
*
* @param   level: VCORE setting level
*
* @return  None
**************************************************************************************************/
void halBoardSetVCore (uint8 level)
{
  uint8 actLevel;

  do {
    actLevel = PMMCTL0_L & PMMCOREV_3;
    if (actLevel < level)
    {
      halBoardSetVCoreUp(++actLevel);       /* Set VCore (step by step) */
    }
    if (actLevel > level)
    {
      halBoardSetVCoreDown(--actLevel);     /* Set VCore (step by step) */
    }
  } while (actLevel != level);
}

/**************************************************************************************************
**************************************************************************************************/

