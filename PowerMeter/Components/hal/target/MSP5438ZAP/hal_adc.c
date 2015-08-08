/**************************************************************************************************
  Filename:       hal_adc.c
  Revised:        $Date: 2010-11-24 13:27:14 -0800 (Wed, 24 Nov 2010) $
  Revision:       $Revision: 24498 $

  Description:    This file contains the interface to the HAL ADC.


  Copyright 2007-2010 Texas Instruments Incorporated. All rights reserved.

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
 *                                           INCLUDES
 **************************************************************************************************/

#include "hal_types.h"
#include "hal_adc.h"
#include "hal_mcu.h"

/**************************************************************************************************
 * @fn      HalAdcInit
 *
 * @brief   Initialize ADC Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalAdcInit (void)
{
}

/**************************************************************************************************
 * @fn      HalAdcRead
 *
 * @brief   Read the ADC based on given channel and resolution
 *
 * @param   channel - channel where ADC will be read
 * @param   resolution - the resolution of the value
 *
 * @return  8 to 12-bit value of the ADC in binary unsigned format.
 **************************************************************************************************/
uint16 HalAdcRead (uint8 channel, uint8 resolution)
{
  ADC12CTL0 = ADC12REF2_5V | ADC12REFON | ADC12ON;
  ADC12CTL1 = ADC12SHP;

  switch (channel)
  {
  case HAL_ADC_CHANNEL_TEMP:
    channel = ADC12INCH_10;
    break;

  case HAL_ADC_CHANNEL_VDD:
    channel = ADC12INCH_11;
    ADC12CTL0 &= ~ADC12REF2_5V;
    break;

  default:
    channel &= ADC12INCH_15;
    break;
  }

  switch (resolution)
  {
  case HAL_ADC_RESOLUTION_8:
    resolution = ADC12RES_0;
    break;

  case HAL_ADC_RESOLUTION_10:
    resolution = ADC12RES_1;
    break;

  case HAL_ADC_RESOLUTION_12:
  case HAL_ADC_RESOLUTION_14:
  default:
    resolution = ADC12RES_2;
    break;
  }

  ADC12CTL2 = resolution;
  ADC12MCTL0 = ADC12SREF_1 | ADC12EOS | channel;

  __delay_cycles(6000);                    // Settling time for reference.

  ADC12CTL0 |= ADC12ENC + ADC12SC;         // Sampling and conversion start.
  while (ADC12IFG == 0);
  ADC12CTL0 = 0;                           // Disable the ADC
  ADC12CTL0 = 0;                           // Turn off reference (must be done AFTER clearing ENC).
 
  return ADC12MEM0;
}

/*********************************************************************
 * @fn       HalAdcCheckVdd
 *
 * @brief    Check for minimum Vdd specified.
 *
 * @param   vdd - The board-specific Vdd reading to check for.
 *
 * @return  TRUE if the Vdd measured is greater than the 'vdd' minimum parameter;
 *          FALSE if not.
 *
 *********************************************************************/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADC12CTL0 = ADC12REFON | ADC12ON;
  ADC12CTL1 = ADC12SHP;
  ADC12CTL2 = 0;
  ADC12MCTL0 = ADC12SREF_1 | ADC12EOS | ADC12INCH_11;

  __delay_cycles(6000);                    // Settling time for reference.

  ADC12CTL0 |= ADC12ENC + ADC12SC;         // Sampling and conversion start.
  while (ADC12IFG == 0);
  ADC12CTL0 = 0;                           // Disable the ADC
  ADC12CTL0 = 0;                           // Turn off reference (must be done AFTER clearing ENC).
 
  return ((uint8)ADC12MEM0 > vdd);
}

/**************************************************************************************************
**************************************************************************************************/
