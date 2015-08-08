/**************************************************************************************************
  Filename:       hal_spi.c
  Revised:        $Date:$
  Revision:       $Revision:$

  Description:    This file contains the interface to the HAL SPI.


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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/

#include  "hal_board.h"
#include  "hal_defs.h"
#include  "hal_types.h"
#include  "hal_spi.h"

#if (defined HAL_SPI) && (HAL_SPI == TRUE)
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalSpiFlush
 *
 * @brief   Write a buffer to the SPI.
 *
 * @param   port - SPI port.
 *          len - Number of bytes to flush.
 *
 * @return  None
 **************************************************************************************************/
void HalSpiFlush(uint8 port, uint8 len)
{
  (void)port;

  while (len--)
  {
    HAL_SPI_WRITE_BYTE(0);
    HAL_SPI_WAIT_DONE();
  }
}

/**************************************************************************************************
 * @fn      HalSpiInit
 *
 * @brief   Initialize SPI Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiInit(void)
{
  HAL_SPI_INIT();
}

/**************************************************************************************************
 * @fn      HalSpiPoll
 *
 * @brief   Poll the SPI.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiPoll(void)
{
  extern void zapPhySpiPoll(uint8 port);
  zapPhySpiPoll(0);
}

/**************************************************************************************************
 * @fn      HalSpiWrite
 *
 * @brief   Write a buffer to the SPI.
 *
 * @param   port - SPI port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write/read.
 *
 * @return  None
 **************************************************************************************************/
void HalSpiWrite(uint8 port, uint8 *pBuf, uint8 len)
{
  (void)port;

  HAL_SPI_SS_ON();
  while (len--)
  {
    HAL_SPI_WRITE_BYTE(*pBuf);
    HAL_SPI_WAIT_DONE();
    *pBuf++ = HAL_SPI_READ_BYTE();
  }
  HAL_SPI_SS_OFF();
}
#endif

/**************************************************************************************************
**************************************************************************************************/
