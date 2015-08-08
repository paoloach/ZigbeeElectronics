/**************************************************************************************************
  Filename:       hal_uart.c
  Revised:        $Date: 2010-12-01 14:21:40 -0800 (Wed, 01 Dec 2010) $
  Revision:       $Revision: 24527 $

  Description:    This file contains the interface to the UART.


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

/*********************************************************************
 * INCLUDES
 */

#include "hal_board_cfg.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "osal.h"
#include "OSAL_Timers.h"

/*-------------------------------------------------------------------------------------------------
                                               MACROS
 ------------------------------------------------------------------------------------------------*/

/* Get 1 byte from UART */
#define HAL_UART_GETBYTE() UCxRXBUF

/* Put 1 byte into the UART */
#define HAL_UART_PUTBYTE(x)            { UCxTXBUF = (x); }

/* Set Baud rate */
#define HAL_UART_SETBAUDRATE(baudrate) { UCxBR1 = (baudrate) >> 8;  UCxBR0 = (baudrate); }

/* Set Source Clock */
#define HAL_UART_SET_SRC_CLK()         { UCxCTL1 |= UCSSEL_3; } /* SMCLK */

/* Setup TXD and RXD Port */
#define HAL_UART_PORT_CONFIG()         { PxSEL |= PxTX | PxRX; \
                                         PxDIR |= PxTX; \
                                         PxDIR &= ~PxRX; }

/* Setup format frame */
#define HAL_UART_FRAME_CONFIG()        { UCxCTL0 = UCMODE_0;   /* UART Mode */        \
                                         UCxCTL0 &= ~UCPEN;    /* Disable parity */   \
                                         UCxCTL0 &= ~UCSPB;    /* 1 stop bit */       \
                                         UCxCTL0 &= ~UC7BIT;   /* 8bit data*/         \
                                         UCxCTL0 &= ~UCSYNC; } /* Asynchronous mode */

/* Enable/Disable TX INT */
#define HAL_UART_TX_INT_ENABLE()       { UCxIE |= UCTXIE; }
#define HAL_UART_TX_INT_DISABLE()      { UCxIE &= ~UCTXIE; }

/* Enable/Disable RX */
#define HAL_UART_RX_ENABLE()           /* N/A */
#define HAL_UART_RX_DISABLE()          /* N/A */

/* Enable/Disable TX INT */
#define HAL_UART_RX_INT_ENABLE()       { UCxIE |= UCRXIE; }
#define HAL_UART_RX_INT_DISABLE()      { UCxIE &= ~UCRXIE; }

/* Enable/Disable SWRST */
#define HAL_UART_SWRST_ENABLE()        { UCxCTL1 |= UCSWRST; }
#define HAL_UART_SWRST_DISABLE()       { UCxCTL1 &= ~UCSWRST; }

/* Get Rx/Tx status bit */
#define HAL_UART_GET_RXTX_STATUS()    (UCxIFG & (UCRXIFG | UCTXIFG))
#define HAL_UART_GET_RX_STATUS()      (UCxIFG & UCRXIFG)
#define HAL_UART_GET_TX_STATUS()      (UCxIFG & UCTXIFG)
#define HAL_UART_CLR_TX_STATUS()      (UCxIFG &= ~UCTXIFG)

/* UART CTS and RTS */
#define HAL_UART_CTS_PORT             /* N/A */
#define HAL_UART_CTS_BIT              /* N/A */

#define HAL_UART_RTS_PORT             /* N/A */
#define HAL_UART_RTS_BIT              /* N/A */

#define HAL_UART_FLOWCONTROL_INIT()   /* N/A */

/*-------------------------------------------------------------------------------------------------
                                          GLOBAL VARIABLES
-------------------------------------------------------------------------------------------------*/

halUARTCfg_t uartRecord;

#define HAL_GET_UBRR(BAUD_BPS)   ((uint32)(HAL_CPU_CLOCK_MHZ * 1000000) / (uint32)BAUD_BPS)

/* UBRR table */
uint16 UBRRTable[] = { HAL_GET_UBRR (9600),
                       HAL_GET_UBRR (19200),
                       HAL_GET_UBRR (38400),
                       HAL_GET_UBRR (57600),
                       HAL_GET_UBRR (115200) };

/*-------------------------------------------------------------------------------------------------
                                         FUNCTIONS - LOCAL
-------------------------------------------------------------------------------------------------*/

static void Hal_UART_BufferInit(void);
static void Hal_UART_RxProcessEvent(void);
static void Hal_UART_TxProcessEvent(void);
static void Hal_UART_SendCallBack(uint8 port, uint8 event);

/*-------------------------------------------------------------------------------------------------
                                  Application Level Functions
-------------------------------------------------------------------------------------------------*/

/*************************************************************************************************
 * @fn      HalUARTInit()
 *
 * @brief   Initialize the UART
 *
 * @param   none
 *
 * @return  none
 *************************************************************************************************/
void HalUARTInit ( void )
{
  Hal_UART_BufferInit();
}

/*************************************************************************************************
 * @fn      HalBufferInit()
 *
 * @brief   Initialize the UART Buffers
 *
 * @param   none
 *
 * @return  none
 *************************************************************************************************/
static void Hal_UART_BufferInit (void)
{
  uartRecord.configured        = FALSE;
  uartRecord.rx.bufferHead     = 0;
  uartRecord.rx.bufferTail     = 0;
  uartRecord.rx.pBuffer        = (uint8 *)NULL;
  uartRecord.tx.bufferHead     = 0;
  uartRecord.tx.bufferTail     = 0;
  uartRecord.tx.pBuffer        = (uint8 *)NULL;
  uartRecord.rxChRvdTime       = 0;
  uartRecord.intEnable         = FALSE;
}

/*************************************************************************************************
 * @fn      HalUARTOpen()
 *
 * @brief   Open a port based on the configuration
 *
 * @param   port   - UART port
 *          config - contains configuration information
 *          cBack  - Call back function where events will be reported back
 *
 * @return  Status of the function call
 *************************************************************************************************/
uint8 HalUARTOpen ( uint8 port, halUARTCfg_t *config )
{
  /* Save important information */
  uartRecord.baudRate             = config->baudRate;
  uartRecord.flowControl          = config->flowControl;
  uartRecord.flowControlThreshold = config->flowControlThreshold;
  uartRecord.rx.maxBufSize        = config->rx.maxBufSize;
  uartRecord.tx.maxBufSize        = config->tx.maxBufSize;
  uartRecord.idleTimeout          = config->idleTimeout;
  uartRecord.intEnable            = config->intEnable;
  uartRecord.callBackFunc         = config->callBackFunc;

  /* Set SWRST - UART logic held in reset state */
  HAL_UART_SWRST_ENABLE();

  /* Setup GPIO */
  HAL_UART_PORT_CONFIG();

  /* Set Frame Format */
  HAL_UART_FRAME_CONFIG();

  /* Set source clock */
  HAL_UART_SET_SRC_CLK();

  /* Setup Baudrate */
  if (config->baudRate > HAL_UART_BR_115200)
  {
    return HAL_UART_BAUDRATE_ERROR;
  }
  else
  {
    HAL_UART_SETBAUDRATE(UBRRTable[config->baudRate]);  /* Set baud rate */
  }

  /* Setup Flow Control */
  if (uartRecord.flowControl)
  {
    HAL_UART_FLOWCONTROL_INIT();
    Hal_UART_FlowControlSet (port, uartRecord.flowControl);
  }

  /* Setup threshold */
  if (config->flowControlThreshold > config->rx.maxBufSize)
    uartRecord.flowControlThreshold = 0;
  else
    uartRecord.flowControlThreshold = config->flowControlThreshold;

  /* Allocate memory for Rx and Tx buffer */
  uartRecord.rx.pBuffer = osal_mem_alloc (uartRecord.rx.maxBufSize);
  uartRecord.tx.pBuffer = osal_mem_alloc (uartRecord.tx.maxBufSize);

  /* Validate buffers */
  if ((uartRecord.rx.pBuffer) && (uartRecord.tx.pBuffer))
  {
    /* Enable RX  Enable Rx Int */
    HAL_UART_RX_ENABLE();

    /* Clear SWRST - releaset to operation */
    HAL_UART_SWRST_DISABLE();

    /* Enable interrupt (optional) */
    if (config->intEnable)
    {
      HAL_UART_RX_INT_ENABLE();
    }
    else
    {
      HAL_UART_RX_INT_DISABLE();
    }

    HAL_UART_CLR_TX_STATUS();

    /* Mark record as "configured" */
    uartRecord.configured = TRUE;

    /* Ready to be used. */
    return HAL_UART_SUCCESS;
  }
  else
  {
    /* If memory allocation failed, not "configured" */
    uartRecord.configured = FALSE;

    /* Failed to allocate Rx and Tx Buffer, end of the story */
    return HAL_UART_MEM_FAIL;
  }
}

/*************************************************************************************************
 * @fn      Hal_UARTPoll
 *
 * @brief   This routine simulate polling and has to be called by the main loop
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
void HalUARTPoll(void)
{
  if (!uartRecord.configured)  // If port is not configured, no point to poll it.
  {
    return;
  }

  if (!uartRecord.intEnable)  // Check Port for items to process.
  {
    if (HAL_UART_GET_RX_STATUS())
    {
      Hal_UART_RxProcessEvent();
    }

    if (HAL_UART_GET_TX_STATUS())
    {
      Hal_UART_TxProcessEvent();
    }
  }

  if ((Hal_UART_RxBufLen(0) + 1) >= uartRecord.rx.maxBufSize)  // Report if Rx Buffer is full.
  {
    Hal_UART_SendCallBack (0, HAL_UART_RX_FULL) ;
  }

  if ((uartRecord.rxChRvdTime != 0) &&  // Report if Rx Buffer is idled.
     ((osal_GetSystemClock() - uartRecord.rxChRvdTime) > uartRecord.idleTimeout))
  {
    Hal_UART_SendCallBack (0, HAL_UART_RX_TIMEOUT) ;
    uartRecord.rxChRvdTime = 0;
  }

  /* Send back warning when buffer it threshold  and turn off flow */
  if (Hal_UART_RxBufLen(0) >= uartRecord.rx.maxBufSize - uartRecord.flowControlThreshold)
  {
    Hal_UART_SendCallBack (0, HAL_UART_RX_ABOUT_FULL) ;
  }

  if (uartRecord.flowControl)
  {
    if (Hal_UART_RxBufLen(0) > uartRecord.rx.maxBufSize / 2)
    {
      Hal_UART_FlowControlSet (0, HAL_UART_FLOW_OFF);
    }
    else
    {
      Hal_UART_FlowControlSet (0, HAL_UART_FLOW_ON);
    }
  }
}

/*************************************************************************************************
 * @fn      HalUARTClose()
 *
 * @brief   Close the UART
 *
 * @param   port - UART port (not used.)
 *
 * @return  none
 *************************************************************************************************/
void HalUARTClose ( uint8 port )
{
  /* Disable Rx Int */
  HAL_UART_RX_INT_DISABLE();

  /* Disable RX */
  HAL_UART_RX_DISABLE();

  /* Deallocate Rx and Tx Buffers */
  if (uartRecord.configured)
  {
    /* Free Tx and Rx buffer */
    osal_mem_free (uartRecord.rx.pBuffer);
    osal_mem_free (uartRecord.tx.pBuffer);

    /* Re-Initialze buffers again */
    Hal_UART_BufferInit();
  }
}

/*************************************************************************************************
 * @fn      HalUARTRead()
 *
 * @brief   Read a buffer from the UART
 *
 * @param   port - UART port (not used.)
 *          ppBuffer - pointer to a pointer that points to the data that will be read
 *          length - length of the requested buffer
 *
 * @return  length of buffer that was read
 *************************************************************************************************/
uint16 HalUARTRead ( uint8 port, uint8 *pBuffer, uint16 length )
{
  uint16 cnt, idx;

  if (!uartRecord.configured)  // If port is not configured, no point to read it.
  {
    return 0;
  }

  // If requested length is bigger than what in buffer, re-adjust it to the buffer length.
  cnt = Hal_UART_RxBufLen(0);
  if (cnt < length)
  {
    length = cnt;
  }

  idx = uartRecord.rx.bufferHead;
  for (cnt = 0; cnt < length; cnt++)
  {
    pBuffer[cnt] = uartRecord.rx.pBuffer[idx++];

    if (idx >= uartRecord.rx.maxBufSize)
    {
      idx = 0;
    }
  }
  uartRecord.rx.bufferHead = idx;

  return length;  // Return number of bytes read.
}

/*************************************************************************************************
 * @fn      HalUARTWrite()
 *
 * @brief   Write a buffer to the UART
 *
 * @param   port    - UART port (not used.)
 *          pBuffer - pointer to the buffer that will be written
 *          length  - length of
 *
 * @return  length of the buffer that was sent
 *************************************************************************************************/
uint16 HalUARTWrite ( uint8 port, uint8 *pBuffer, uint16 length )
{
  uint16 cnt, idx;
  halIntState_t intState;

  if (!uartRecord.configured)
  {
    return 0;
  }

  // Capture the value of the volatile variables.
  idx = uartRecord.tx.bufferHead;
  cnt = uartRecord.tx.bufferTail;
  if (cnt == idx)
  {
    cnt = uartRecord.tx.maxBufSize;
  }
  else if (cnt > idx)
  {
    cnt = uartRecord.tx.maxBufSize - cnt + idx;
  }
  else // (cnt < idx)
  {
    cnt = idx - cnt;
  }
   
  // Accept "all-or-none" on write request.
  if (cnt < length)
  {
    return 0;
  }

  idx = uartRecord.tx.bufferTail;

  for (cnt = 0; cnt < length; cnt++)
  {
    uartRecord.tx.pBuffer[idx++] = pBuffer[cnt];

    if (idx >= uartRecord.tx.maxBufSize)
    {
      idx = 0;
    }
  }

  HAL_ENTER_CRITICAL_SECTION(intState);  // Hold off interrupts.
  cnt = uartRecord.tx.bufferTail;
  if (cnt == uartRecord.tx.bufferHead)
  {
    if (uartRecord.intEnable)
    {
      HAL_UART_TX_INT_ENABLE();
    }
    HAL_UART_PUTBYTE(uartRecord.tx.pBuffer[uartRecord.tx.bufferHead]);  // Send a char to UART.
  }
  uartRecord.tx.bufferTail = idx;
  HAL_EXIT_CRITICAL_SECTION(intState);  // Restore interrupt enable.

  return length;  // Return the number of bytes actually put into the buffer.
}

/*************************************************************************************************
 * @fn      Hal_UART_RxBufLen()
 *
 * @brief   Calculate Rx Buffer length of a port
 *
 * @param   port - UART port (not used.)
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
uint16 Hal_UART_RxBufLen (uint8 port)
{
  int16 length = uartRecord.rx.bufferTail;

  length -= uartRecord.rx.bufferHead;
  if  (length < 0)
    length += uartRecord.rx.maxBufSize;

  return (uint16)length;
}

/*************************************************************************************************
 * @fn      Hal_UART_TxBufLen()
 *
 * @brief   Calculate Tx Buffer length of a port
 *
 * @param   port - UART port (not used.)
 *
 * @return  length of current Tx buffer
 *************************************************************************************************/
uint16 Hal_UART_TxBufLen ( uint8 port )
{
  int16 length = uartRecord.tx.bufferTail;

  length -= uartRecord.tx.bufferHead;
  if  (length < 0)
    length += uartRecord.tx.maxBufSize;

  return (uint16)length;
}

/*-------------------------------------------------------------------------------------------------
                                           HELP FUNCTIONS
-------------------------------------------------------------------------------------------------*/

/*************************************************************************************************
 * @fn      HalUARTSendCallBack
 *
 * @brief   Send Callback back to the caller
 *
 * @param   port - UART port
 *          event - event that causes the call back
 *
 * @return  None
 *************************************************************************************************/
static void Hal_UART_SendCallBack(uint8 port, uint8 event)
{
  if (uartRecord.callBackFunc)
  {
    (uartRecord.callBackFunc)(port, event);
  }
}

/*************************************************************************************************
 * @fn      Hal_UART_ProcessRxEvent
 *
 * @brief   Process the Rx data by putting them in Rx Buffer. Callback will happen if idle timeout
 *          or Rx buffer is full
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
static void Hal_UART_RxProcessEvent(void)
{
  uint8 ch = HAL_UART_GETBYTE();

  uartRecord.rx.pBuffer[uartRecord.rx.bufferTail++] = ch;
  if (uartRecord.rx.bufferTail >= uartRecord.rx.maxBufSize)
  {
    uartRecord.rx.bufferTail = 0;
  }

  uartRecord.rxChRvdTime = osal_GetSystemClock();
}

/*************************************************************************************************
 * @fn      Hal_UART_ProcessTxEvent
 *
 * @brief   Process Tx buffer and events
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
static void Hal_UART_TxProcessEvent(void)
{
  uint16 tail = uartRecord.tx.bufferTail;

  if (++uartRecord.tx.bufferHead >= uartRecord.tx.maxBufSize)
  {
    uartRecord.tx.bufferHead = 0;
  }

  if (uartRecord.tx.bufferHead != tail)
  {
    HAL_UART_PUTBYTE(uartRecord.tx.pBuffer[uartRecord.tx.bufferHead]);  // Send a char to UART.
  }
  else
  {
    HAL_UART_TX_INT_DISABLE();
    HAL_UART_CLR_TX_STATUS();
  }
}

/*************************************************************************************************
 * @fn      Hal_UART_SetFlowControl
 *
 * @brief   Set UART Rx flow control
 *
 * @param   port: serial port (not used.)
 *          on:   0=OFF, !0=ON
 *
 * @return  none
 *
 *************************************************************************************************/
void Hal_UART_FlowControlSet( uint8 port, uint8 status )
{
}

/*-------------------------------------------------------------------------------------------------
                                    Interrupt Service Routines
-------------------------------------------------------------------------------------------------*/

/*************************************************************************************************
 * @fn      UART Rx/Tx ISR
 *
 * @brief   Called when a serial byte is ready to read and/or write.
 *  NOTE:   Assumes that uartRecord.configured is TRUE if this interrupt is enabled.
 *
 * @param   void
 *
 * @return  void
**************************************************************************************************/
INTERRUPT_UART()
{
  do {
    if (HAL_UART_GET_RX_STATUS())
    {
      Hal_UART_RxProcessEvent();
    }

    if (HAL_UART_GET_TX_STATUS())
    {
      Hal_UART_TxProcessEvent();
    }
  } while (HAL_UART_GET_RXTX_STATUS());
}

