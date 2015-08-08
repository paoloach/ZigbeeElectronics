/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2008-06-16 10:34:03 -0700 (Mon, 16 Jun 2008) $
  Revision:       $Revision: 17253 $

  Description:    This file contains the interface to the HAL KEY Service.


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
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "hal_key.h"
#include "osal.h"
#include "Onboard.h"
#include "hal_drivers.h"
#include "hal_mcu.h"


/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_SW_MASK 0x7F  /* Total of 7 switches - UP, DOWN, LEFT, RIGHT, PUSH, B1, B2 */


/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/* KEYS */

/* Non interrupt key read */
#define HAL_KEY_KEYS(x)                           \
{                                                 \
  x  = (HAL_PUSH_BUTTON1() != 0) * HAL_KEY_SW_1;  \
  x |= (HAL_PUSH_BUTTON2() != 0) * HAL_KEY_SW_2;  \
  x |= (HAL_PUSH_BUTTON3() != 0) * HAL_KEY_SW_3;  \
  x |= (HAL_PUSH_BUTTON4() != 0) * HAL_KEY_SW_4;  \
  x |= (HAL_PUSH_BUTTON5() != 0) * HAL_KEY_SW_5;  \
  x |= (HAL_PUSH_BUTTON6() != 0) * HAL_KEY_SW_6;  \
  x |= (HAL_PUSH_BUTTON7() != 0) * HAL_KEY_SW_7;  \
}

/* Interrupt key read */
#define HAL_KEY_KEYS_INT       P2IFG
#define HAL_KEY_KEYS_INT_BIT   0xFE  /* JOY interrupt P2.1:7 */

/* Interrupt key read */
#define HAL_KEY_INT_KEYS(x)                       \
{                                                 \
  x  = ((P2IFG & PUSH1_BV) != 0) * HAL_KEY_SW_1;  \
  x |= ((P2IFG & PUSH2_BV) != 0) * HAL_KEY_SW_2;  \
  x |= ((P2IFG & PUSH3_BV) != 0) * HAL_KEY_SW_3;  \
  x |= ((P2IFG & PUSH4_BV) != 0) * HAL_KEY_SW_4;  \
  x |= ((P2IFG & PUSH5_BV) != 0) * HAL_KEY_SW_5;  \
  x |= ((P2IFG & PUSH6_BV) != 0) * HAL_KEY_SW_6;  \
  x |= ((P2IFG & PUSH7_BV) != 0) * HAL_KEY_SW_7;  \
}

#define HAL_KEY_WAKE_INIT()

/* Rising edge trigger for key */
#define HAL_KEY_INT_INIT()                       \
{                                                \
  P2IES &= ~HAL_KEY_KEYS_INT_BIT;                \
  P2SEL &= ~HAL_KEY_KEYS_INT_BIT;                \
  P2DIR &= ~HAL_KEY_KEYS_INT_BIT;                \
  P2REN |= HAL_KEY_KEYS_INT_BIT;                 \
  P2OUT |= HAL_KEY_KEYS_INT_BIT;                 \
}

// Joystick interrupt on P2.1:7
#define HAL_ENABLE_KEY_INT()       { P2IE  |= HAL_KEY_KEYS_INT_BIT;     /* Enable Interrupt */           \
                                     P2IES |= HAL_KEY_KEYS_INT_BIT;     /* Interrupt Edge Select H->L */ \
                                     P2IFG &= ~HAL_KEY_KEYS_INT_BIT; }  /* Clear pending interrupt */
#define HAL_DISABLE_KEY_INT()      { P2IE  &= ~(HAL_KEY_KEYS_INT_BIT);  }  /* Disable int */
#define HAL_CLEAR_KEY_INT()        { P2IFG &= ~(HAL_KEY_KEYS_INT_BIT);  }  /* Clear pending int */


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
#if (HAL_KEY == TRUE)
static uint8 halSavedKeys;
static uint8 halIntKeys;
static halKeyCBack_t pHal_KeyProcessFunction;
bool Hal_KeyIntEnable;
#endif /* HAL_KEY */

/**************************************************************************************************
 *                                        EXTERNAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
#if (HAL_KEY == TRUE)
  /* Initialize previous key to 0 */
  halSavedKeys = 0;

  /* Initialize callback function */
  pHal_KeyProcessFunction  = NULL;
#endif /* HAL_KEY */

}

/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig( bool interruptEnable, halKeyCBack_t cback)
{
#if (HAL_KEY == TRUE)
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHal_KeyProcessFunction = cback;

  /* Initialize GPIO */
  HAL_KEY_INT_INIT();

  /* Determine if interrupt is enable or not */
  if ( Hal_KeyIntEnable )
  {
    /* Enable interrupts */
    HAL_ENABLE_KEY_INT();

    /* Cancel polling if there is one active */
    osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);
  }
  else
  {
    /* Disable interrupts */
    HAL_DISABLE_KEY_INT();

    /* Start polling */
    osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead( void )
{
  uint8 keys = 0;

#if (HAL_KEY == TRUE)
  if (!Hal_KeyIntEnable)
  {
    HAL_KEY_KEYS (keys);
  }
  else
  {
    HAL_KEY_INT_KEYS (keys);
  }
#endif /* HAL_KEY */

  return keys;
}

/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Send call back if key(s) is pressed
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll( void )
{
#if (HAL_KEY == TRUE)
  uint8 keys = 0;

  /* if polling is using */
  if (!Hal_KeyIntEnable)
  {
    /* Get keys */
    HAL_KEY_KEYS (keys);

    /* If same as before, no keys */
    if ( keys == halSavedKeys )
      return;

    /* Store the current keys for comparation next time */
    halSavedKeys = keys;

  }
  else
  {
    /* Keys are read in ISR */
    keys = halIntKeys;
  }

  /* If it's not SW, return */
  if ( !(keys & HAL_KEY_SW_MASK) )
    return;

  /* Callback */
  if (pHal_KeyProcessFunction)
    (pHal_KeyProcessFunction) (keys, ((keys & HAL_KEY_SW_6) ? HAL_KEY_STATE_SHIFT : HAL_KEY_STATE_NORMAL));

#endif /* HAL_KEY */
}

#ifdef POWER_SAVING
/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
  /* nothing to do */
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  uint8 keys = 0;

  /* Get keys */
  if (!Hal_KeyIntEnable)
  {
    HAL_KEY_KEYS (keys);
  }
  else
  {
    HAL_KEY_INT_KEYS (keys);
  }
  return ( keys );
}
#endif /* POWER_SAVING */
/**************************************************************************************************
 * @fn      INTERRUPT_KEYBD
 *
 * @brief   Interrupt Service Routine for keyboard
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
INTERRUPT_KEYBD()
{
#if (HAL_KEY == TRUE)

#ifdef POWER_SAVING
  /* Must allow key interrupt to cancel sleep */
  __low_power_mode_off_on_exit();
#endif

  /* Read the key before it gone */
  HAL_KEY_INT_KEYS(halIntKeys);

  /* Clear depending interrupt */
  HAL_CLEAR_KEY_INT();

  /* A key is pressed, let HalKeyPoll routing handle the keys at a later time */
  osal_start_timerEx( Hal_TaskID, HAL_KEY_EVENT, 100 );
#endif /* HAL_KEY */
}

/**************************************************************************************************
**************************************************************************************************/



