/**************************************************************************************************
  Filename:       hal_lcd.c
  Revised:        $Date: 2009-04-01 22:04:53 -0700 (Wed, 01 Apr 2009) $
  Revision:       $Revision: 19626 $

  Description:    This file contains the interface to the HAL LCD Service.


  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"
#include <stdlib.h>
#include <string.h>

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  #include "DebugTrace.h"
#endif

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/* LCD Max Chars and Buffer */
#define HAL_LCD_MAX_CHARS   16
#define HAL_LCD_MAX_BUFF    25

/* LCD lines */
#define LCD_MAX_LINE_COUNT              3

/* Defines for HW LCD */
#define HAL_LCD_BACKLIGHT_LEVEL         10   //0-15    0:OFF, 15:MAX
#define HAL_LCD_CONTRAST_LEVEL          100   //70-127  70:LOW, 127:HIGH

/**************************************************************************************************
 *                                           MACROS
 **************************************************************************************************/

unsigned char LcdInitMacro[]={
            0x74,0x00,0x00,0x76,0x00,0x01,  // R00 start oscillation
            0x74,0x00,0x01,0x76,0x00,0x0D,  // R01 driver output control
            0x74,0x00,0x02,0x76,0x00,0x4C,  // R02 LCD - driving waveform control
            0x74,0x00,0x03,0x76,0x12,0x14,  // R03 Power control
            0x74,0x00,0x04,0x76,0x04,0x66,  // R04 Contrast control
            0x74,0x00,0x05,0x76,0x00,0x10,  // R05 Entry mode
            0x74,0x00,0x06,0x76,0x00,0x00,  // R06 RAM data write mask
            0x74,0x00,0x07,0x76,0x00,0x15,  // R07 Display control
            0x74,0x00,0x08,0x76,0x00,0x03,  // R08 Cursor Control
            0x74,0x00,0x09,0x76,0x00,0x00,  // R09 RAM data write mask
            0x74,0x00,0x0A,0x76,0x00,0x15,  // R0A
            0x74,0x00,0x0B,0x76,0x00,0x03,  // R0B Horizontal Cursor Position
            0x74,0x00,0x0C,0x76,0x00,0x03,  // R0C Vertical Cursor Position
            0x74,0x00,0x0D,0x76,0x00,0x00,  // R0D
            0x74,0x00,0x0E,0x76,0x00,0x15,  // R0E
            0x74,0x00,0x0F,0x76,0x00,0x03,  // R0F
            0x74,0x00,0x10,0x76,0x00,0x15,  // R0E
            0x74,0x00,0x11,0x76,0x00,0x03,  // R0F
};

#define LCD_ROW                 110
#define LCD_COL                 138
#define LCD_Size                3505
#define LCD_MEM_SIZE            110*17
#define LCD_LAST_PIXEL          3505

#define FONT_HEIGHT		          12

#define INVERT_TEXT             BIT0
#define OVERWRITE_TEXT          BIT2
#define HAL_LCD_TEXT_STYLE      OVERWRITE_TEXT

/**************************************************************************************************
 *                                       GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 *Lcd_Line1;
static uint8 HAL_LCD_SETCONTRAST_CONTROL[] = {0x74,0x00,0x04,0x76,0x04,HAL_LCD_CONTRAST_LEVEL};

unsigned char Read_Block_Address_Macro[]= {0x74,0x00,0x12,0x77,0x00,0x00};
unsigned char Draw_Block_Value_Macro[]=   {0x74,0x00,0x12,0x76,0xFF,0xFF};
unsigned char Draw_Block_Address_Macro[]= {0x74,0x00,0x11,0x76,0x00,0x00};

unsigned int LcdAddress=0, LcdTableAddress=0;

int LCD_MEM[LCD_MEM_SIZE];

const unsigned char fonts_lookup[]={
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,            //Comment
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,            //Comment
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,            //Comment
          0x00,0x00,63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,              //Comment
          64,65,0,69,0,68,67,0,0,1,          //'0' = 48 = 0x30
          2,3,4,5,6,7,8,9,66,0,              //'9' = 57 = 0x39
          0,70,0,62,0,10,11,12,13,14,        //'A' --> 'Z'
          15,16,17,18,19,20,21,22,23,24,
          25,26,27,28,29,30,31,32,33,34,
          35,0,0,0,71,0,0,36,37,38,          //'a' = 97
          39,40,41,42,43,44,45,46,47,48,
          49,50,51,52,53,54,55,56,57,58,
          59,60,61,62,0 ,0, 0, 72,73,74,
          75,76,77,78,79,80,81};             //'z' = 122

const unsigned int fonts[]= {
          0x0000,0x0ffc,0x3c0f,0x3f0f,0x3fcf,0x3ccf,0x3cff,0x3c3f,0x3c0f,0x0ffc,0x0000,0x0000,0x0000,
          0x0000,0x00c0,0x00f0,0x00ff,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x0fff,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f00,0x03c0,0x00f0,0x003c,0x0f0f,0x0fff,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f00,0x0f00,0x03f0,0x0f00,0x0f00,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0f00,0x0fc0,0x0ff0,0x0f3c,0x0f0f,0x3fff,0x0f00,0x0f00,0x3fc0,0x0000,0x0000,0x0000,
          0x0000,0x0fff,0x000f,0x000f,0x000f,0x03ff,0x0f00,0x0f00,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x03f0,0x003c,0x000f,0x000f,0x03ff,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x3fff,0x3c0f,0x3c0f,0x3c00,0x0f00,0x03c0,0x00f0,0x00f0,0x00f0,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f3f,0x03fc,0x0fcf,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x0ffc,0x03c0,0x03c0,0x00f0,0x00fc,0x0000,0x0000,0x0000,
          0x0000,0x00f0,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x0fff,0x0f0f,0x0f0f,0x0f0f,0x0000,0x0000,0x0000,
          0x0000,0x0fff,0x3c3c,0x3c3c,0x3c3c,0x0ffc,0x3c3c,0x3c3c,0x3c3c,0x0fff,0x0000,0x0000,0x0000,
          0x0000,0x0ff0,0x3c3c,0x3c0f,0x000f,0x000f,0x000f,0x3c0f,0x3c3c,0x0ff0,0x0000,0x0000,0x0000,
          0x0000,0x03ff,0x0f3c,0x3c3c,0x3c3c,0x3c3c,0x3c3c,0x3c3c,0x0f3c,0x03ff,0x0000,0x0000,0x0000,
          0x0000,0x3fff,0x303c,0x003c,0x0c3c,0x0ffc,0x0c3c,0x003c,0x303c,0x3fff,0x0000,0x0000,0x0000,
          0x0000,0x3fff,0x3c3c,0x303c,0x0c3c,0x0ffc,0x0c3c,0x003c,0x003c,0x00ff,0x0000,0x0000,0x0000,
          0x0000,0x0ff0,0x3c3c,0x3c0f,0x000f,0x000f,0x3f0f,0x3c0f,0x3c3c,0x3ff0,0x0000,0x0000,0x0000,
          0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0fff,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x3fc0,0x0f00,0x0f00,0x0f00,0x0f00,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x3c3f,0x3c3c,0x0f3c,0x0f3c,0x03fc,0x0f3c,0x0f3c,0x3c3c,0x3c3f,0x0000,0x0000,0x0000,
          0x0000,0x00ff,0x003c,0x003c,0x003c,0x003c,0x303c,0x3c3c,0x3c3c,0x3fff,0x0000,0x0000,0x0000,
          0x0000,0x3c0f,0x3f3f,0x3fff,0x3fff,0x3ccf,0x3c0f,0x3c0f,0x3c0f,0x3c0f,0x0000,0x0000,0x0000,
          0x0000,0x3c0f,0x3c0f,0x3c3f,0x3cff,0x3fff,0x3fcf,0x3f0f,0x3c0f,0x3c0f,0x0000,0x0000,0x0000,
          0x0000,0x03f0,0x0f3c,0x3c0f,0x3c0f,0x3c0f,0x3c0f,0x3c0f,0x0f3c,0x03f0,0x0000,0x0000,0x0000,
          0x0000,0x0fff,0x3c3c,0x3c3c,0x3c3c,0x0ffc,0x003c,0x003c,0x003c,0x00ff,0x0000,0x0000,0x0000,
          0x0000,0x03f0,0x0f3c,0x3c0f,0x3c0f,0x3c0f,0x3f0f,0x3fcf,0x0ffc,0x0f00,0x3fc0,0x0000,0x0000,
          0x0000,0x0fff,0x3c3c,0x3c3c,0x3c3c,0x0ffc,0x0f3c,0x3c3c,0x3c3c,0x3c3f,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f0f,0x000f,0x00fc,0x03c0,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0fff,0x0cf3,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x00f0,0x0000,0x0000,0x0000,
          0x0000,0x3c0f,0x3c0f,0x3c0f,0x3c0f,0x3ccf,0x3ccf,0x0f3c,0x0f3c,0x0f3c,0x0000,0x0000,0x0000,
          0x0000,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x00f0,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x0000,0x0000,0x0000,
          0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x00f0,0x00f0,0x00f0,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x3fff,0x3f0f,0x03c3,0x03c0,0x00f0,0x003c,0x303c,0x3c0f,0x3fff,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03fc,0x0f00,0x0ffc,0x0f0f,0x0f0f,0x3cfc,0x0000,0x0000,0x0000,
          0x0000,0x003f,0x003c,0x003c,0x0ffc,0x3c3c,0x3c3c,0x3c3c,0x3c3c,0x0fcf,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03fc,0x0f0f,0x000f,0x000f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0fc0,0x0f00,0x0f00,0x0ffc,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x3cfc,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03fc,0x0f0f,0x0fff,0x000f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x03f0,0x0f3c,0x003c,0x003c,0x03ff,0x003c,0x003c,0x003c,0x00ff,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3cfc,0x0f0f,0x0f0f,0x0f0f,0x0ffc,0x0f00,0x0f0f,0x03fc,0x0000,
          0x0000,0x003f,0x003c,0x003c,0x0f3c,0x3cfc,0x3c3c,0x3c3c,0x3c3c,0x3c3f,0x0000,0x0000,0x0000,
          0x0000,0x03c0,0x03c0,0x0000,0x03fc,0x03c0,0x03c0,0x03c0,0x03c0,0x3ffc,0x0000,0x0000,0x0000,
          0x0000,0x0f00,0x0f00,0x0000,0x0ff0,0x0f00,0x0f00,0x0f00,0x0f00,0x0f0f,0x0f0f,0x03fc,0x0000,
          0x0000,0x003f,0x003c,0x003c,0x3c3c,0x0f3c,0x03fc,0x0f3c,0x3c3c,0x3c3f,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x03c0,0x03c0,0x03c0,0x03c0,0x03c0,0x03c0,0x03c0,0x3ffc,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0fff,0x3ccf,0x3ccf,0x3ccf,0x3ccf,0x3c0f,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03ff,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0fcf,0x3c3c,0x3c3c,0x3c3c,0x3c3c,0x0ffc,0x003c,0x00ff,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3cfc,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0ffc,0x0f00,0x3fc0,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0f3f,0x3f3c,0x3cfc,0x003c,0x003c,0x00ff,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x03fc,0x0f0f,0x003c,0x03c0,0x0f0f,0x03fc,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0030,0x003c,0x0fff,0x003c,0x003c,0x003c,0x0f3c,0x03f0,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x3cfc,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0f0f,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x00f0,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3c0f,0x3c0f,0x3ccf,0x3ccf,0x0f3c,0x0f3c,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3c0f,0x0f3c,0x03f0,0x03f0,0x0f3c,0x3c0f,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3c3c,0x3c3c,0x3c3c,0x3c3c,0x0ff0,0x0f00,0x03c0,0x00ff,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0fff,0x0f03,0x03c0,0x003c,0x0c0f,0x0fff,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f00,0x03c0,0x00f0,0x00f0,0x0000,0x00f0,0x00f0,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x0f00,0x03c0,0x00f0,0x003c,0x003c,0x003c,0x00f0,0x03c0,0x0f00,0x0000,0x0000,0x0000,
          0x0000,0x003c,0x00f0,0x03c0,0x0f00,0x0f00,0x0f00,0x03c0,0x00f0,0x003c,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x03f0,0x03f0,0x0000,0x0000,0x03f0,0x03f0,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03f0,0x03f0,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x0000,0x3ffc,0x3ffc,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x03c0,0x03c0,0x3ffc,0x3ffc,0x03c0,0x03c0,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x0000,0x0000,0x0000,0x3ffc,0x0000,0x0000,0x3ffc,0x0000,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
          0x0000,0x0ffc,0x3c0f,0x3f0f,0x3fcf,0x3ccf,0x3cff,0x3c3f,0x3c0f,0x0ffc,0x0000,0x0000,0x0000, //0
          0x0000,0x00c0,0x00f0,0x00ff,0x00f0,0x00f0,0x00f0,0x00f0,0x00f0,0x0fff,0x0000,0x0000,0x0000, //1
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f00,0x03c0,0x00f0,0x003c,0x0f0f,0x0fff,0x0000,0x0000,0x0000, //2
          0x0000,0x03fc,0x0f0f,0x0f00,0x0f00,0x03f0,0x0f00,0x0f00,0x0f0f,0x03fc,0x0000,0x0000,0x0000, //3
          0x0000,0x0f00,0x0fc0,0x0ff0,0x0f3c,0x0f0f,0x3fff,0x0f00,0x0f00,0x3fc0,0x0000,0x0000,0x0000, //4
          0x0000,0x0fff,0x000f,0x000f,0x000f,0x03ff,0x0f00,0x0f00,0x0f0f,0x03fc,0x0000,0x0000,0x0000, //5
          0x0000,0x03f0,0x003c,0x000f,0x000f,0x03ff,0x0f0f,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000, //6
          0x0000,0x3fff,0x3c0f,0x3c0f,0x3c00,0x0f00,0x03c0,0x00f0,0x00f0,0x00f0,0x0000,0x0000,0x0000, //7
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f3f,0x03fc,0x0fcf,0x0f0f,0x0f0f,0x03fc,0x0000,0x0000,0x0000, //8
          0x0000,0x03fc,0x0f0f,0x0f0f,0x0f0f,0x0ffc,0x03c0,0x03c0,0x00f0,0x00fc,0x0000,0x0000,0x0000, //9
} ;

/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/

void HalLcd_HW_Init(void);
void HalLcd_HW_InitBackLight(void);
void HalLcd_HW_SetContrast(uint8 level);
void HalLcd_HW_Wait(uint16 i);
void HalLcd_HW_Clear(void);
void HalLcd_HW_SendCmd(uint8 cmd[]);
void HalLcd_HW_SetAddress(int Address);
void HalLcd_HW_WriteLine(char str[], uint8 line);

void HalLcd_HW_DrawCurrentBlock(unsigned int Value);
void HalLcd_HW_DrawBlock(unsigned int Address, unsigned int Value);

void HalLcd_HW_Active(void);
#endif //LCD

/**************************************************************************************************
 * @fn      HalLcdInit
 *
 * @brief   Initilize LCD Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 **************************************************************************************************/
void HalLcdInit(void)
{
#if (HAL_LCD == TRUE)
  Lcd_Line1 = NULL;
  HalLcd_HW_Init();
#endif
}

/*************************************************************************************************
 *                    LCD EMULATION FUNCTIONS
 *
 * Some evaluation boards are equipped with Liquid Crystal Displays
 * (LCD) which may be used to display diagnostic information. These
 * functions provide LCD emulation, sending the diagnostic strings
 * to Z-Tool via the RS232 serial port. These functions are enabled
 * when the "LCD_SUPPORTED" compiler flag is placed in the makefile.
 *
 * Most applications update both lines (1 and 2) of the LCD whenever
 * text is posted to the device. This emulator assumes that line 1 is
 * updated first (saved locally) and the formatting and send operation
 * is triggered by receipt of line 2. Nothing will be transmitted if
 * only line 1 is updated.
 *
 *************************************************************************************************/


/**************************************************************************************************
 * @fn      HalLcdWriteString
 *
 * @brief   Write a string to the LCD
 *
 * @param   str    - pointer to the string that will be displayed
 *          line   - line number to display
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteString ( char *str, uint8 line)
{

#if (HAL_LCD == TRUE)
  uint8 strLen = 0;
  uint8 totalLen = 0;
  uint8 *buf;
  uint8 tmpLen;

  if ( Lcd_Line1 == NULL )
  {
    Lcd_Line1 = osal_mem_alloc( HAL_LCD_MAX_CHARS+1 );
    HalLcdWriteString( "TexasInstruments", 0 );
  }

  strLen = (uint8)osal_strlen( (char*)str );

  /* Check boundries */
  if ( strLen > HAL_LCD_MAX_CHARS )
    strLen = HAL_LCD_MAX_CHARS;

  if ( line == HAL_LCD_LINE_1 )
  {
    /* Line 1 gets saved for later */
    osal_memcpy( Lcd_Line1, str, strLen );
    Lcd_Line1[strLen] = '\0';
  }
  else
  {
    /* Line 2 triggers action */
    tmpLen = (uint8)osal_strlen( (char*)Lcd_Line1 );
    totalLen =  tmpLen + 1 + strLen + 1;
    buf = osal_mem_alloc( totalLen );
    if ( buf != NULL )
    {
      /* Concatenate strings */
      osal_memcpy( buf, Lcd_Line1, tmpLen );
      buf[tmpLen++] = ' ';
      osal_memcpy( &buf[tmpLen], str, strLen );
      buf[tmpLen+strLen] = '\0';

      /* Send it out */
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)

#if (SERIAL_DEBUG_SUPPORTED)
      debug_str( (uint8*)buf );
#endif //LCD_SUPPORTED

#endif //ZTOOL_P1

      /* Free mem */
      osal_mem_free( buf );
    }
  }
  /* Clear the line - print 17 blanks */
  HalLcd_HW_WriteLine ("                 ", line);
  /* Display the string */
  HalLcd_HW_WriteLine (str, line);

#endif //HAL_LCD

}

/**************************************************************************************************
 * @fn      HalLcdWriteValue
 *
 * @brief   Write a value to the LCD
 *
 * @param   value  - value that will be displayed
 *          radix  - 8, 10, 16
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option)
{
#if (HAL_LCD == TRUE)
  uint8 buf[HAL_LCD_MAX_BUFF];

  _ltoa( value, &buf[0], radix );
  HalLcdWriteString( (char*)buf, option );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteScreen
 *
 * @brief   Write a value to the LCD
 *
 * @param   line1  - string that will be displayed on line 1
 *          line2  - string that will be displayed on line 2
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteScreen( char *line1, char *line2 )
{
#if (HAL_LCD == TRUE)
  HalLcdWriteString( line1, 1 );
  HalLcdWriteString( line2, 2 );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title  - Title that will be displayed before the value
 *          value  - value
 *          format - redix
 *          line   - line number
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
#if (HAL_LCD == TRUE)
  uint8 tmpLen;
  uint8 buf[HAL_LCD_MAX_BUFF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  HalLcdWriteString( (char*)buf, line );		
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title   - Title that will be displayed before the value
 *          value1  - value #1
 *          format1 - redix of value #1
 *          value2  - value #2
 *          format2 - redix of value #2
 *          line    - line number
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1,
                                  uint16 value2, uint8 format2, uint8 line )
{

#if (HAL_LCD == TRUE)

  uint8 tmpLen;
  uint8 buf[HAL_LCD_MAX_BUFF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  if ( tmpLen )
  {
    osal_memcpy( buf, title, tmpLen );
    buf[tmpLen++] = ' ';
  }

  err = (uint32)(value1);
  _ltoa( err, &buf[tmpLen], format1 );
  tmpLen = (uint8)osal_strlen( (char*)buf );

  buf[tmpLen++] = ',';
  buf[tmpLen++] = ' ';
  err = (uint32)(value2);
  _ltoa( err, &buf[tmpLen], format2 );

  HalLcdWriteString( (char *)buf, line );		

#endif
}

/**************************************************************************************************
 * @fn      HalLcdDisplayPercentBar
 *
 * @brief   Display percentage bar on the LCD
 *
 * @param   title   -
 *          value   -
 *
 * @return  None
 **************************************************************************************************/
void HalLcdDisplayPercentBar( char *title, uint8 value )
{
#if (HAL_LCD == TRUE)

  uint8 percent;
  uint8 leftOver;
  uint8 buf[17];
  uint32 err;
  uint8 x;

  /* Write the title: */
  HalLcdWriteString( title, HAL_LCD_LINE_1 );

  if ( value > 100 )
    value = 100;

  /* convert to blocks */
  percent = (uint8)(value / 10);
  leftOver = (uint8)(value % 10);

  /* Make window */
  osal_memcpy( buf, "[          ]  ", 15 );

  for ( x = 0; x < percent; x ++ )
  {
    buf[1+x] = '>';
  }

  if ( leftOver >= 5 )
    buf[1+x] = '+';

  err = (uint32)value;
  _ltoa( err, (uint8*)&buf[13], 10 );

  HalLcdWriteString( (char*)buf, HAL_LCD_LINE_2 );

#endif

}

/**************************************************************************************************
 *                                    HARDWARE LCD
 **************************************************************************************************/
#define LCD_BACKLIGHT_OUT P8OUT
#define LCD_BACKLIGHT_DIR P8DIR
#define LCD_BACKLIGHT_SEL P8SEL
#define LCD_BACKLIGHT_PIN BIT3

#define LCD_COMM_OUT_V2   P8OUT
#define LCD_COMM_DIR_V2   P8DIR
#define LCD_COMM_SEL_V2   P8SEL
#define LCD_CS_PIN_V2     BIT1
#define LCD_RESET_PIN_V2  BIT2

#define LCD_COMM_OUT_V3   P9OUT
#define LCD_COMM_DIR_V3   P9DIR
#define LCD_COMM_SEL_V3   P9SEL
#define LCD_CS_PIN_V3     BIT6
#define LCD_RESET_PIN_V3  BIT7

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 * @fn      HalLcd_HW_Init
 *
 * @brief   Initilize HW LCD Driver.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Init(void)
{
  int counter=0, i=0;

  LCD_BACKLIGHT_DIR |= LCD_BACKLIGHT_PIN;
  LCD_BACKLIGHT_OUT |= LCD_BACKLIGHT_PIN;
  LCD_BACKLIGHT_SEL |= LCD_BACKLIGHT_PIN;

  if (HAL_MSP_EXP430F5438_REV_02())
  {
    LCD_COMM_DIR_V2 |= LCD_CS_PIN_V2 | LCD_RESET_PIN_V2;
    LCD_COMM_OUT_V2 |= LCD_CS_PIN_V2 | LCD_RESET_PIN_V2;

    /* Reset LCD */
    LCD_COMM_OUT_V2 &= ~LCD_RESET_PIN_V2;
    for (i = 0x47FF; i > 0; i--);
    LCD_COMM_OUT_V2 |= LCD_RESET_PIN_V2;
  }
  else /* Must be MSP EXP430F5438_REV_03 */
  {
    LCD_COMM_DIR_V3 |= LCD_CS_PIN_V3 | LCD_RESET_PIN_V3;
    LCD_COMM_OUT_V3 |= LCD_CS_PIN_V3 | LCD_RESET_PIN_V3;

    /* Reset LCD */
    LCD_COMM_OUT_V3 &= ~LCD_RESET_PIN_V3;
    for (i = 0x47FF; i > 0; i--);
    LCD_COMM_OUT_V3 |= LCD_RESET_PIN_V3;
  }

  /* UCLK, MOSI setup, SOMI cleared */
  P9SEL |= BIT1 + BIT3;
  P9SEL &= ~BIT2;
  P9DIR |= BIT1 + BIT3;
  P9DIR &= ~BIT2;

  /* 3-pin, 8-bit SPI master */
  UCB2CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;

  /* SMCLK */
  UCB2CTL1 |= UCSSEL_2;
  UCB2BR0 = 3;
  UCB2BR1 = 0;

  /* Initialize USCI */
  UCB2CTL1 &= ~UCSWRST;
  UCB2IFG &= ~UCRXIFG;

  /* Initialize backlight */
#if !defined (POWER_SAVING)
  HalLcd_HW_InitBackLight();
#endif

  /* LCD Initialization Routine Using Predefined settings */
  while (counter < 8*6)
  {
    HalLcd_HW_SendCmd(&LcdInitMacro[counter]);
    counter += 6;
  }

  /* Turn LCD ON */
  HalLcd_HW_Active();

  /* Clear screen */
  HalLcd_HW_Clear();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Control
 *
 * @brief   Write 6 bytes command to the LCD
 *
 * @param   uint8 cmd[] - 6 bytes command to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_SendCmd(uint8 cmd[])
{
  unsigned char i;
  unsigned char rev02 = HAL_MSP_EXP430F5438_REV_02();

  if (rev02)
  {
    LCD_COMM_OUT_V2 &= ~LCD_CS_PIN_V2;      //CS = 0 --> Start Transfer
  }
  else /* Must be MSP EXP430F5438_REV_03 */
  {
    LCD_COMM_OUT_V3 &= ~LCD_CS_PIN_V3;      //CS = 0 --> Start Transfer
  }

  for ( i=0; i<6; i++ )
  {
    while (!(UCB2IFG & UCTXIFG));
    UCB2TXBUF = cmd[i];

    while (UCB2STAT & UCBUSY);
    if (i==2)                               //Pull CS up after 3 bytes
    {
      if (rev02)
      {
        LCD_COMM_OUT_V2 |= LCD_CS_PIN_V2;   //CS = 1 --> Stop Transfer
        LCD_COMM_OUT_V2 &= ~LCD_CS_PIN_V2;  //CS = 0 --> Start Transfer	
      }
      else /* Must be MSP EXP430F5438_REV_03 */
      {
        LCD_COMM_OUT_V3 |= LCD_CS_PIN_V3;   //CS = 1 --> Stop Transfer
        LCD_COMM_OUT_V3 &= ~LCD_CS_PIN_V3;  //CS = 0 --> Start Transfer	
      }
    }
  }

  if (rev02)
  {
    LCD_COMM_OUT_V2 |= LCD_CS_PIN_V2;       //CS = 1 --> Stop Transfer
  }
  else /* Must be MSP EXP430F5438_REV_03 */
  {
    LCD_COMM_OUT_V3 |= LCD_CS_PIN_V3;       //CS = 1 --> Stop Transfer
  }
}


/**************************************************************************************************
 * @fn          HalLcd_HW_SetBackLight
 *
 * @brief       Initialize Backlight with predefined value
 *
 * @param       None
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_InitBackLight(void)
{
  uint8 dutyCycle = 0, i;

   if (HAL_LCD_BACKLIGHT_LEVEL > 0)
  {
    TA0CCTL3 = OUTMOD_4;

    for (i = 0; i < HAL_LCD_BACKLIGHT_LEVEL; i++)
    {
      dutyCycle += 25;
    }
    TA0CCR3 = dutyCycle;
  }
  else
  {  	
    TA0CCTL3 = 0;
  }
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Clear
 *
 * @brief   Clear the HW LCD
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Clear(void)
{
  int i;
  HalLcd_HW_SetAddress(0);
  for ( i = 0; i < LCD_Size; i++)
      HalLcd_HW_DrawCurrentBlock(0x00);
  HalLcd_HW_SetAddress(0);
}

/**************************************************************************************************
 * @fn          halLcdWriteLine
 *
 * @brief       Write one line on display
 *
 * @param       uint8 line - display line
 *              char *pText - text buffer to write
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_WriteLine(char str[], uint8 line)
{
  int i,j,Counter=0, BlockValue;
  int Address,  LCD_MEM_Add, ActualAddress;
  char LookUpChar;
  int temp;

  /* Set correct line based on font size */
  HalLcd_HW_SetAddress( (line * FONT_HEIGHT) << 5) ;  // 0x20 = 2^5

  ActualAddress = LcdAddress;
  Counter =  LcdAddress & 0x1F;
 	i=0;

  while (str[i]!=0)
  {
    LookUpChar = fonts_lookup[str[i]];
    for (j=0;j < FONT_HEIGHT ;j++)
    {
      Address = ActualAddress + j*0x20;
      temp = Address >> 5;
      temp = temp + (temp <<4);

      LCD_MEM_Add = temp + (Address & 0x1F);

      BlockValue = LCD_MEM[ LCD_MEM_Add ];

      if (HAL_LCD_TEXT_STYLE & INVERT_TEXT)
      {
        if (HAL_LCD_TEXT_STYLE & OVERWRITE_TEXT)
          BlockValue = 0xFFFF - fonts[LookUpChar*13+j];
        else
          BlockValue |= 0xFFFF - fonts[LookUpChar*13+j];
      }
      else
      {
        if (HAL_LCD_TEXT_STYLE & OVERWRITE_TEXT)
          BlockValue = fonts[LookUpChar*(FONT_HEIGHT+1) +j];
        else
          BlockValue |= fonts[LookUpChar*(FONT_HEIGHT+1) +j];
      }
      HalLcd_HW_DrawBlock( Address, BlockValue);
    }

    Counter++;
    if (Counter == 17)
    {
      Counter = 0;
      ActualAddress += 0x20*FONT_HEIGHT  - 16;
      if (ActualAddress > LCD_LAST_PIXEL-0x20*FONT_HEIGHT )
        ActualAddress = 0;
    }
    else
      ActualAddress++;
    i++;
  }
  HalLcd_HW_SetAddress(ActualAddress);
}

/**************************************************************************************************
 * @fn          HalLcd_HW_InitContrast
 *
 * @brief       Initialize the constrast based on the predefined value
 *
 * @param       uint8 level 70 to 127 - min/max
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_SetContrast(uint8 level)
{
  HAL_LCD_SETCONTRAST_CONTROL[5] = level;
  HalLcd_HW_SendCmd(HAL_LCD_SETCONTRAST_CONTROL);
}

/**************************************************************************************************
 * @fn          HalLcd_HW_SetAddress
 *
 * @brief       The the current location address
 *
 * @param       Address = LcdAddress  					
 *              LcdTableAddress = Correct Address Row + Column
 *                              = (Address / 0x20)* 17 + Column
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_SetAddress(int Address)
{
  int temp;
  Draw_Block_Address_Macro[4] = Address >> 8;
  Draw_Block_Address_Macro[5] = Address & 0xFF;
  HalLcd_HW_SendCmd(Draw_Block_Address_Macro);
  LcdAddress = Address;
  temp = Address >> 5;                      // Divided by 0x20
  temp = temp + (temp << 4);
  //Multiplied by (1+16) and added by the offset
  LcdTableAddress = temp + (Address & 0x1F);
}

/**************************************************************************************************
 * @fn          HalLcd_Active
 *
 * @brief       Enable the LCD
 *
 * @param       None
 *
 * @return      None
 **************************************************************************************************/
void HalLcd_HW_Active(void)
{
  HalLcd_HW_SendCmd(LcdInitMacro);
  LcdInitMacro[ 3 * 6 + 5 ] |= BIT3 ;
  LcdInitMacro[ 3 * 6 + 5 ] &= ~BIT0;
  HalLcd_HW_SendCmd(&LcdInitMacro[ 3 * 6 ]);
}


/**************************************************************************************************
 * @fn          HalLcd_HW_DrawCurrentBlock
 *
 * @brief       Draw the block at current position
 *              Writes Value to LCD CGRAM
 *              Writes Value to MSP430 Internal LCD Table
 *              Updates LcdAddress and LcdTableAddress due to auto increment feature
 *
 * @param       Value - Value of the current block
 *
 * @return      None
 **************************************************************************************************/
void HalLcd_HW_DrawCurrentBlock(unsigned int Value)
{
  int temp;
  Draw_Block_Value_Macro[4] = Value >> 8;
  Draw_Block_Value_Macro[5] = Value & 0xFF;
  LCD_MEM[ LcdTableAddress ] = Value;
  HalLcd_HW_SendCmd(Draw_Block_Value_Macro);
  LcdAddress++;
  temp = LcdAddress >> 5;                   // Divided by 0x20
  temp = temp + (temp << 4);
  //Multiplied by (1+16) and added by the offset
  LcdTableAddress = temp + (LcdAddress & 0x1F);
  // If LcdAddress gets off the right edge, move to next line
  if ((LcdAddress & 0x1F) > 0x11)
    HalLcd_HW_SetAddress( (LcdAddress & 0xFFE0) + 0x20 );

  if (LcdAddress == LCD_Size)
    HalLcd_HW_SetAddress( 0 );
}

/**************************************************************************************************
 * @fn          HalLcd_HW_DrawBlock
 *
 * @brief       Writes Value to LCD CGRAM and LCD_MEM at Address location
 *
 * @param       Value - Value of the current block
 *
 * @return      None
 **************************************************************************************************/
void HalLcd_HW_DrawBlock(unsigned int Address, unsigned int Value)
{
  HalLcd_HW_SetAddress(Address);
  HalLcd_HW_DrawCurrentBlock(Value);
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Wait
 *
 * @brief   wait for 4 "nop"
 *
 * @param   uint16 i - number of 4xNOP
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Wait(uint16 i)
{
  while(i--)
  {
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
  }
}

/**************************************************************************************************
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
 **************************************************************************************************/
/*
void _itoa(uint16 num, byte *buf, byte radix)
{
  char c,i;
  byte *p, rst[5];

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
*/
#endif

/**************************************************************************************************
**************************************************************************************************/


