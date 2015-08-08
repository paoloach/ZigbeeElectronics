/**************************************************************************************************
  Filename:       hal_board_cfg.h
  Revised:        $Date: 2013-10-21 17:25:23 -0700 (Mon, 21 Oct 2013) $
  Revision:       $Revision: 35745 $

  Description:    Declarations for the MSP430F5438 as a ZAP master of the
                  CC2530ZNP.


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

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_assert.h"
#include "hal_dco.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Board Indentifier
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_BOARD_F5438

/* ------------------------------------------------------------------------------------------------
 *                                          Clock Speed
 *
 *   Note that when the HAL_CPU_CLOCK_MHZ is changed, the halBoardSetSystemClock() in
 *   HAL_CLOCK_INIT() macro must also be changed. Also note that the maximum Clock Speed
 *   is 18MHz. 25MHz should not be used until MSP430F5438A part is available.
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_CPU_CLOCK_MHZ     12.0000

/* ------------------------------------------------------------------------------------------------
 *                                          Clock Type
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_CLOCK_TYPE_DCO         0
#define HAL_CLOCK_TYPE_CRYSTAL     1

#if defined (HAL_CLOCK_CRYSTAL)
#define HAL_CLOCK_TYPE             HAL_CLOCK_TYPE_CRYSTAL
#else
#define HAL_CLOCK_TYPE             HAL_CLOCK_TYPE_DCO
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       LED Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_NUM_LEDS            2
#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x34000; i++) { }; } )

/* LED1 - GREEN */
#define LED1_BV           BV(0)
#define LED1_PORT         P1OUT
#define LED1_DDR          P1DIR

/* LED2 - RED */
#define LED2_BV           BV(1)
#define LED2_PORT         P1OUT
#define LED2_DDR          P1DIR

/* LED3 - not supported - set to LED 1 for now */
#define LED3_BV           LED1_BV
#define LED3_PORT         LED1_PORT
#define LED3_DDR          LED1_DDR

/* LED4 - not supported - set to LED 2 for now */
#define LED4_BV           LED2_BV
#define LED4_PORT         LED2_PORT
#define LED4_DDR          LED2_DDR


/* ------------------------------------------------------------------------------------------------
 *                                    Push Button Configuration
 * ------------------------------------------------------------------------------------------------
 */

#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!    /* double negation forces result to be '1' */

/* UP */
#define PUSH1_BV          BV(4)
#define PUSH1_PORT        P2IN
#define PUSH1_POLARITY    ACTIVE_LOW

/* RIGHT */
#define PUSH2_BV          BV(2)
#define PUSH2_PORT        P2IN
#define PUSH2_POLARITY    ACTIVE_LOW

/* DOWN */
#define PUSH3_BV          BV(5)
#define PUSH3_PORT        P2IN
#define PUSH3_POLARITY    ACTIVE_LOW

/* LEFT */
#define PUSH4_BV          BV(1)
#define PUSH4_PORT        P2IN
#define PUSH4_POLARITY    ACTIVE_LOW

/* PUSH */
#define PUSH5_BV          BV(3)
#define PUSH5_PORT        P2IN
#define PUSH5_POLARITY    ACTIVE_LOW

/* BUTTON 1 */
#define PUSH6_BV          BV(6)
#define PUSH6_PORT        P2IN
#define PUSH6_POLARITY    ACTIVE_LOW

/* BUTTON 2 */
#define PUSH7_BV          BV(7)
#define PUSH7_PORT        P2IN
#define PUSH7_POLARITY    ACTIVE_LOW


/* ------------------------------------------------------------------------------------------------
 *                                     Board REV identification
 * ------------------------------------------------------------------------------------------------
 */

#define BOARD_ID_PIN                          BV(7)
#define BOARD_ID_PORT                         P8IN
#define HAL_BOARD_ID_CONFIG()             st( P8DIR &= ~BV(7); P8REN |= BV(7); P8OUT |= BV(7); )
#define HAL_BOARD_ID_DISABLE_PULLUP()     st( P8REN &= ~BV(7); )
#define HAL_MSP_EXP430F5438_REV_02()        ((BOARD_ID_PORT & BOARD_ID_PIN) != 0)
#define HAL_MSP_EXP430F5438_REV_03()        ((BOARD_ID_PORT & BOARD_ID_PIN) == 0)


/* ------------------------------------------------------------------------------------------------
 *                         OSAL NV implemented by internal flash pages.
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_FLASH_PAGE_SIZE        512
#define HAL_FLASH_WORD_SIZE        1

// NV page definitions must coincide with segment declaration in "MSP430F5438.xcl" file
#define HAL_NV_PAGE_END            124
#define HAL_NV_PAGE_CNT            24
#define HAL_NV_PAGE_BEG           (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)
#define HAL_NV_IEEE_ADDR           0x19F8  /* The end of INFOA page. */


/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_CLOCK_INIT(x)                            \
{                                                    \
  /* The DCO library does not handle the stack */    \
  /* frame well. As a result, this macro must */     \
  /* be the first thing that runs after main. */     \
  /* Disable watchdog timer */                       \
  WDTCTL = WDTPW | WDTHOLD;                          \
                                                     \
  if (x == HAL_CLOCK_TYPE_CRYSTAL)                   \
  {                                                  \
    /* Configure external crystal TBD */             \
  }                                                  \
  else if (x == HAL_CLOCK_TYPE_DCO)                  \
  {                                                  \
    /* Note that when halBoardSetSystemClock() */    \
    /* is changed, the HAL_CPU_CLOCK_MHZ must */     \
    /* also be changed. */                           \
    halBoardSetSystemClock(SYSCLK_12MHZ);            \
  }                                                  \
  else                                               \
  {                                                  \
    /* Unknown clock type */                         \
    HAL_ASSERT(0);                                   \
  }                                                  \
                                                     \
  /* CPU15 Errata workaround */                      \
  __bic_SR_register(SCG0);                           \
                                                     \
}

/* ----------- Board Initialization ---------- */
#define HAL_BOARD_INIT()                                         \
{                                                                \
  /* initialize MCU clocks */                                    \
  HAL_CLOCK_INIT(HAL_CLOCK_TYPE);                                \
                                                                 \
  /* reset does not affect GPIO state */                         \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  HAL_TURN_OFF_LED3();                                           \
  HAL_TURN_OFF_LED4();                                           \
                                                                 \
  /* set direction for GPIO outputs  */                          \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  LED3_DDR |= LED3_BV;                                           \
  LED4_DDR |= LED4_BV;                                           \
                                                                 \
  /* configure board ID GPIO */                                  \
  HAL_BOARD_ID_CONFIG();                                         \
}

/* ----------- Debounce ---------- */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/* ----------- Push Buttons ---------- */
#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_PORT & PUSH1_BV))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_PORT & PUSH2_BV))
#define HAL_PUSH_BUTTON3()        (PUSH3_POLARITY (PUSH3_PORT & PUSH3_BV))
#define HAL_PUSH_BUTTON4()        (PUSH4_POLARITY (PUSH4_PORT & PUSH4_BV))
#define HAL_PUSH_BUTTON5()        (PUSH5_POLARITY (PUSH5_PORT & PUSH5_BV))
#define HAL_PUSH_BUTTON6()        (PUSH6_POLARITY (PUSH6_PORT & PUSH6_BV))
#define HAL_PUSH_BUTTON7()        (PUSH7_POLARITY (PUSH7_PORT & PUSH7_BV))

/* ----------- LED's ---------- */
#define HAL_TURN_OFF_LED1()       st( LED1_PORT &= ~LED1_BV; )
#define HAL_TURN_OFF_LED2()       st( LED2_PORT &= ~LED2_BV; )
#define HAL_TURN_OFF_LED3()       st( LED3_PORT &= ~LED3_BV; )
#define HAL_TURN_OFF_LED4()       st( LED4_PORT &= ~LED4_BV; )

#define HAL_TURN_ON_LED1()        st( LED1_PORT |=  LED1_BV; )
#define HAL_TURN_ON_LED2()        st( LED2_PORT |=  LED2_BV; )
#define HAL_TURN_ON_LED3()        st( LED3_PORT |=  LED3_BV; )
#define HAL_TURN_ON_LED4()        st( LED4_PORT |=  LED4_BV; )

#define HAL_TOGGLE_LED1()         st( LED1_PORT ^=  LED1_BV; )
#define HAL_TOGGLE_LED2()         st( LED2_PORT ^=  LED2_BV; )
#define HAL_TOGGLE_LED3()         st( LED3_PORT ^=  LED3_BV; )
#define HAL_TOGGLE_LED4()         st( LED4_PORT ^=  LED4_BV; )

#define HAL_STATE_LED1()          (LED1_PORT & LED1_BV)
#define HAL_STATE_LED2()          (LED2_PORT & LED2_BV)

/* ----------- Minimum safe bus voltage ---------- */

// Vdd/2 / Internal Reference X ENOB --> (Vdd / 2) / 1.5 X 255
#define VDD_2_0  170  // 2.0 V required to safely read/write internal flash.

#define VDD_MIN_RUN  VDD_2_0
#define VDD_MIN_NV  (VDD_2_0+9)  // 5% margin over minimum to survive a page erase and compaction.

#define HAL_ZNP_RST_BIT           2
#if ZAP_PHY_RESET_ZNP
#define HAL_ZNP_RST_CFG()         st( HAL_ZNP_RST();                 \
                                      P1DIR |=  BV(HAL_ZNP_RST_BIT); )
#else
#define HAL_ZNP_RST_CFG()         st( HAL_ZNP_RUN();                 \
                                      P1DIR |=  BV(HAL_ZNP_RST_BIT); )
#endif
#define HAL_ZNP_RUN()             st( P1OUT |=  BV(HAL_ZNP_RST_BIT); )
#define HAL_ZNP_RST()             st( P1OUT &= ~BV(HAL_ZNP_RST_BIT); )

#define HAL_ZNP_MRDY_BIT          6
#define HAL_ZNP_MRDY_CFG()        st( P1SEL &= ~BV(HAL_ZNP_MRDY_BIT); \
                                      P1OUT |=  BV(HAL_ZNP_MRDY_BIT); \
                                      HAL_ZNP_MRDY_CLR();             \
                                      P1DIR |=  BV(HAL_ZNP_MRDY_BIT); )
#define HAL_ZNP_MRDY_CLR()        st( P1OUT |=  BV(HAL_ZNP_MRDY_BIT); )
#define HAL_ZNP_MRDY_SET()        st( P1OUT &= ~BV(HAL_ZNP_MRDY_BIT); )
  
#define HAL_ZNP_SRDY_BIT          4
#define HAL_ZNP_SRDY_CFG()        st( P1SEL &= ~BV(HAL_ZNP_SRDY_BIT); \
                                      P1DIR &= ~BV(HAL_ZNP_SRDY_BIT); )
#define HAL_ZNP_SRDY_CLR()          ((P1IN & BV(HAL_ZNP_SRDY_BIT)) != 0)
#define HAL_ZNP_SRDY_SET()          ((P1IN & BV(HAL_ZNP_SRDY_BIT)) == 0)
#define HAL_ZNP_SRDY_RX()             HAL_ZNP_SRDY_SET()
#define HAL_ZNP_SRDY_TX()             HAL_ZNP_SRDY_CLR()

#ifdef ZAP_DEBUG
#define HAL_ZAP_DEBUG_BIT         7
#define HAL_ZAP_DEBUG_SET()        st( P1OUT |=  BV(HAL_ZAP_DEBUG_BIT); )
#define HAL_ZAP_DEBUG_CLR()        st( P1OUT &= ~BV(HAL_ZAP_DEBUG_BIT); )
#define HAL_ZAP_DEBUG_CFG()        st( P1SEL &= ~BV(HAL_ZAP_DEBUG_BIT);  \
                                      P1DIR |=  BV(HAL_ZAP_DEBUG_BIT);   \
                                      P1OUT |=  BV(HAL_ZAP_DEBUG_BIT);   \
                                      HAL_ZAP_DEBUG_CLR();)
#else
#define HAL_ZAP_DEBUG_SET() 
#define HAL_ZAP_DEBUG_CLR()      
#define HAL_ZAP_DEBUG_CFG()
#endif

              

// Configure for ISR on falling edge: SRDY.
#define HAL_ZNP_SRDY1_CFG(PORT)   st( P1IE  &= ~BV(HAL_ZNP_SRDY_BIT); \
                                      P1IES |=  BV(HAL_ZNP_SRDY_BIT); \
                                      P1IE  |=  BV(HAL_ZNP_SRDY_BIT); )
// Configure for ISR on rising edge: SRDY.
#define HAL_ZNP_SRDY2_CFG(PORT)   st( P1IE  &= ~BV(HAL_ZNP_SRDY_BIT); \
                                      P1IES &= ~BV(HAL_ZNP_SRDY_BIT); \
                                      P1IE  |=  BV(HAL_ZNP_SRDY_BIT); )

// Configure as an input with pullup for power-saving while driving 1 for use extern 32-kHz clock.
#define HAL_ZNP_CFG0_BIT          Not Connected.
#define HAL_ZNP_CFG0_HI()

#define HAL_ZNP_CFG1_BIT          2
#define HAL_ZNP_CFG1_DIR          P8DIR
#define HAL_ZNP_CFG1_OUT          P8OUT
#define HAL_ZNP_CFG1_REN          P8REN
#define HAL_ZNP_CFG1_SEL          P8SEL

// Configure as an input with pullup for power-saving while driving 1 for use SPI transport.
#define HAL_ZNP_CFG1_HI()         st( HAL_ZNP_CFG1_SEL &= ~BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_OUT |=  BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_REN |=  BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_DIR &= ~BV(HAL_ZNP_CFG1_BIT); )

// Configure as output low for use UART transport.
#define HAL_ZNP_CFG1_LO()         st( HAL_ZNP_CFG1_SEL &= ~BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_OUT &= ~BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_REN &= ~BV(HAL_ZNP_CFG1_BIT); \
                                      HAL_ZNP_CFG1_DIR |=  BV(HAL_ZNP_CFG1_BIT); )

/* ------------------------------------------------------------------------------------------------
 *                                     Driver Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER FALSE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD TRUE
#endif

/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/* Set to TRUE enable KEY usage, FALSE disable it */
#ifndef HAL_KEY
#define HAL_KEY TRUE
#endif

#ifndef HAL_UART 
#define HAL_UART (ZAP_PHY_UART || ZAP_ZNP_MT || ZAP_SBL_PROXY)
#endif

#ifndef HAL_SPI
#define HAL_SPI  ZAP_PHY_SPI
#endif

#define UCxBR0                         UCA1BR0
#define UCxBR1                         UCA1BR1
#define UCxCTL0                        UCA1CTL0
#define UCxCTL1                        UCA1CTL1
#define UCxIE                          UCA1IE
#define UCxIFG                         UCA1IFG
#define UCxRXBUF                       UCA1RXBUF
#define UCxTXBUF                       UCA1TXBUF
#define PxDIR                          P5DIR
#define PxSEL                          P5SEL
#define PxRX                           BV(7)
#define PxTX                           BV(6)
#define INTERRUPT_UART()               HAL_ISR_FUNCTION( halBoardUart1Isr, USCI_A1_VECTOR )

/* ------------------------------------------------------------------------------------------------
 *                                    Interrupt Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* ----------- timer interrupts ---------- */
#define INTERRUPT_TIMERB_OC_CC0()     HAL_ISR_FUNCTION( haBoardTimerB0Isr, TIMERB0_VECTOR )
#define INTERRUPT_TIMERB_OC_CC1_6()   HAL_ISR_FUNCTION( haBoardTimerB1Isr, TIMERB1_VECTOR )

/* ----------- key interrupts ---------- */
#define INTERRUPT_KEYBD()             HAL_ISR_FUNCTION( halBoardPort1Isr, PORT2_VECTOR )


#endif
/*******************************************************************************************************
*/
