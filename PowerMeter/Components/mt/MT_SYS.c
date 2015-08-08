/******************************************************************************
  Filename:       MT_SYS.c
  Revised:        $Date: 2014-08-04 15:38:03 -0700 (Mon, 04 Aug 2014) $
  Revision:       $Revision: 39653 $

  Description:   MonitorTest functions for SYS commands.

  Copyright 2007-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

 *****************************************************************************/

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "MT.h"
#include "MT_SYS.h"
#include "MT_VERSION.h"
#include "OSAL.h"
#include "OSAL_NV.h"
#include "Onboard.h"  /* This is here because RAM read/write macros need it */
#include "hal_adc.h"
#include "OSAL_Clock.h"
#include "mac_low_level.h"
#include "ZMAC.h"
#include "mac_radio_defs.h"
#if defined ( MT_SYS_JAMMER_FEATURE )
  #include "mac_rx.h"
#endif
#if !defined(CC253X_MACNP)
  #include "ZGlobals.h"
#endif
#if (defined INCLUDE_REVISION_INFORMATION) && ((defined MAKE_CRC_SHDW) || (defined FAKE_CRC_SHDW)) //built for bootloader
  #include "hal_flash.h"
  #include "sb_shared.h"
#endif
#include "ZDiags.h"
#include "MT_UART.h"

/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/

/* Max possible MT response length, limited by TX buffer size and sizeof uint8 */
#define MT_MAX_RSP_LEN  ( MIN( MT_UART_DEFAULT_MAX_TX_BUFF, 255 ) )

/* Max possible MT response data length, MT protocol overhead */
#define MT_MAX_RSP_DATA_LEN  ( (MT_MAX_RSP_LEN - 1) - SPI_0DATA_MSG_LEN )
  
#define MT_SYS_DEVICE_INFO_RESPONSE_LEN 14

#if !defined HAL_GPIO || !HAL_GPIO
#define GPIO_DIR_IN(IDX)
#define GPIO_DIR_OUT(IDX)
#define GPIO_TRI(IDX)
#define GPIO_PULL_UP(IDX)
#define GPIO_PULL_DN(IDX)
#define GPIO_SET(IDX)
#define GPIO_CLR(IDX)
#define GPIO_TOG(IDX)
#define GPIO_GET(IDX) 0
#define GPIO_HiD_SET() (val = 0)
#define GPIO_HiD_CLR() (val = 0)
#endif

#if defined ( MT_SYS_SNIFFER_FEATURE )
#if defined ( HAL_MCU_CC2530 ) && !defined ( HAL_BOARD_CC2530USB )
  // This only work with the CC253x chips
  #define HAL_BOARD_ENABLE_INTEGRATED_SNIFFER() st         \
  (                                                                                                                                                                                                                                   \
    OBSSEL3 = 0xFD;                                        \
    OBSSEL4 = 0xFC;                                        \
    RFC_OBS_CTRL1 = 0x09; /* 9 - sniff clk */              \
    RFC_OBS_CTRL2 = 0x08; /* 8 - sniff data */             \
    MDMTEST1 |= 0x04;                                      \
  )

  // This only work with the CC253x chips
  #define HAL_BOARD_DISABLE_INTEGRATED_SNIFFER() st        \
  (                                                                                                                                                                                                                                   \
    OBSSEL3 &= ~0x80;                                                                                                                                                                             \
    OBSSEL4 &= ~0x80;                                                                                                                                                                             \
    RFC_OBS_CTRL1 = 0x00; /* 0 - constant value 0 to rfc_obs_sigs[1] */                                                                                   \
    RFC_OBS_CTRL2 = 0x00; /* 0 - constant value 0 to rfc_obs_sigs[2] */                                                                                   \
    MDMTEST1 &= ~0x04;                                                                                                                                                         \
  )
#else
  #define HAL_BOARD_ENABLE_INTEGRATED_SNIFFER() { status = FAILURE; }
  #define HAL_BOARD_DISABLE_INTEGRATED_SNIFFER() { status = FAILURE; }
#endif
#endif // MT_SYS_SNIFFER_FEATURE

#define RESET_HARD     0
#define RESET_SOFT     1
#define RESET_SHUTDOWN 2

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/

#if !defined MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA
#define MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA  FALSE
#endif

const uint16 MT_SysOsalEventId [] = {
                                      MT_SYS_OSAL_EVENT_0,
                                      MT_SYS_OSAL_EVENT_1,
                                      MT_SYS_OSAL_EVENT_2,
                                      MT_SYS_OSAL_EVENT_3
                                    };

typedef enum {
  GPIO_DIR,
  GPIO_TRI,
  GPIO_SET,
  GPIO_CLR,
  GPIO_TOG,
  GPIO_GET,
  GPIO_HiD = 0x12
} GPIO_Op_t;

#if defined ( MT_SYS_JAMMER_FEATURE )
  #define JAMMER_CHECK_EVT                           0x0001

  #if !defined ( JAMMER_DETECT_CONTINUOUS_EVENTS )
    #define JAMMER_DETECT_CONTINUOUS_EVENTS          150
  #endif
  #if !defined ( JAMMER_DETECT_PERIOD_TIME )
    #define JAMMER_DETECT_PERIOD_TIME                100  // In milliseconds
  #endif
  #if !defined ( JAMMER_HIGH_NOISE_LEVEL )
    #define JAMMER_HIGH_NOISE_LEVEL                  -65
  #endif
#endif // MT_SYS_JAMMER_FEATURE

/***************************************************************************************************
 * LOCAL VARIABLES
 ***************************************************************************************************/
#if defined ( MT_SYS_JAMMER_FEATURE )
  static uint8 jammerTaskID;
  static uint16 jammerContinuousEvents = JAMMER_DETECT_CONTINUOUS_EVENTS;
  static uint16 jammerDetections = JAMMER_DETECT_CONTINUOUS_EVENTS;
  static int8 jammerHighNoiseLevel = JAMMER_HIGH_NOISE_LEVEL;
  static uint32 jammerDetectPeriodTime = JAMMER_DETECT_PERIOD_TIME;
#endif 
  
#if defined ( MT_SYS_SNIFFER_FEATURE )
static uint8 sniffer = FALSE;
#endif

/***************************************************************************************************
 * LOCAL FUNCTIONS
 ***************************************************************************************************/
#if defined (MT_SYS_FUNC)
void MT_SysReset(uint8 *pBuf);
void MT_SysPing(void);
void MT_SysVersion(void);
void MT_SysSetExtAddr(uint8 *pBuf);
void MT_SysGetExtAddr(void);
void MT_SysOsalNVItemInit(uint8 *pBuf);
void MT_SysOsalNVDelete(uint8 *pBuf);
void MT_SysOsalNVLength(uint8 *pBuf);
void MT_SysOsalNVRead(uint8 *pBuf);
void MT_SysOsalNVWrite(uint8 *pBuf);
void MT_SysOsalStartTimer(uint8 *pBuf);
void MT_SysOsalStopTimer(uint8 *pBuf);
void MT_SysRandom(void);
void MT_SysAdcRead(uint8 *pBuf);
void MT_SysGpio(uint8 *pBuf);
void MT_SysStackTune(uint8 *pBuf);
void MT_SysSetUtcTime(uint8 *pBuf);
void MT_SysGetUtcTime(void);
void MT_SysSetTxPower(uint8 *pBuf);
#if defined ( MT_SYS_JAMMER_FEATURE )
  void MT_SysJammerParameters( uint8 *pBuf );
#endif /* MT_SYS_JAMMER_FEATURE */
#if defined ( MT_SYS_SNIFFER_FEATURE )
void MT_SysSnifferParameters( uint8 *pBuf );
#endif /* MT_SYS_SNIFFER_FEATURE */
#if defined ( FEATURE_SYSTEM_STATS )
void MT_SysZDiagsInitStats(void);
void MT_SysZDiagsClearStats(uint8 *pBuf);
void MT_SysZDiagsGetStatsAttr(uint8 *pBuf);
void MT_SysZDiagsRestoreStatsFromNV(void);
void MT_SysZDiagsSaveStatsToNV(void);
#endif /* FEATURE_SYSTEM_STATS */
#endif /* MT_SYS_FUNC */
void powerOffSoc(void);

#if defined (MT_SYS_FUNC)
/***************************************************************************************************
 * @fn      MT_SysProcessing
 *
 * @brief   Process all the SYS commands that are issued by test tool
 *
 * @param   pBuf - pointer to the msg buffer
 *
 *          | LEN  | CMD0  | CMD1  |  DATA  |
 *          |  1   |   1   |   1   |  0-255 |
 *
 * @return  status
 ***************************************************************************************************/
uint8 MT_SysCommandProcessing(uint8 *pBuf)
{
  uint8 status = MT_RPC_SUCCESS;

  switch (pBuf[MT_RPC_POS_CMD1])
  {
    case MT_SYS_RESET_REQ:
      MT_SysReset(pBuf);
      break;

    case MT_SYS_PING:
      MT_SysPing();
      break;

    case MT_SYS_VERSION:
      MT_SysVersion();
      break;

    case MT_SYS_SET_EXTADDR:
      MT_SysSetExtAddr(pBuf);
      break;

    case MT_SYS_GET_EXTADDR:
      MT_SysGetExtAddr();
      break;

// CC253X MAC Network Processor does not have NV support
#if !defined(CC253X_MACNP)
    case MT_SYS_OSAL_NV_DELETE:
      MT_SysOsalNVDelete(pBuf);
      break;

    case MT_SYS_OSAL_NV_ITEM_INIT:
      MT_SysOsalNVItemInit(pBuf);
      break;

    case MT_SYS_OSAL_NV_LENGTH:
      MT_SysOsalNVLength(pBuf);
      break;

    case MT_SYS_OSAL_NV_READ:
      MT_SysOsalNVRead(pBuf);
      break;

    case MT_SYS_OSAL_NV_WRITE:
      MT_SysOsalNVWrite(pBuf);
      break;

    case MT_SYS_OSAL_NV_READ_EXT:
      MT_SysOsalNVRead(pBuf);
      break;
      
    case MT_SYS_OSAL_NV_WRITE_EXT:
      MT_SysOsalNVWrite(pBuf);
      break;
#endif

    case MT_SYS_OSAL_START_TIMER:
      MT_SysOsalStartTimer(pBuf);
      break;

    case MT_SYS_OSAL_STOP_TIMER:
      MT_SysOsalStopTimer(pBuf);
      break;

    case MT_SYS_RANDOM:
      MT_SysRandom();
      break;

    case MT_SYS_ADC_READ:
      MT_SysAdcRead(pBuf);
      break;

    case MT_SYS_GPIO:
      MT_SysGpio(pBuf);
      break;

    case MT_SYS_STACK_TUNE:
      MT_SysStackTune(pBuf);
      break;

    case MT_SYS_SET_TIME:
      MT_SysSetUtcTime(pBuf);
      break;

    case MT_SYS_GET_TIME:
      MT_SysGetUtcTime();
      break;

    case MT_SYS_SET_TX_POWER:
      MT_SysSetTxPower(pBuf);
      break;
      
#if defined ( MT_SYS_JAMMER_FEATURE )
    case MT_SYS_JAMMER_PARAMETERS:
      MT_SysJammerParameters( pBuf );
      break;
#endif      

#if defined ( MT_SYS_SNIFFER_FEATURE )
    case MT_SYS_SNIFFER_PARAMETERS:
      MT_SysSnifferParameters( pBuf );
      break;
#endif      

#if defined ( FEATURE_SYSTEM_STATS )
    case MT_SYS_ZDIAGS_INIT_STATS:
      MT_SysZDiagsInitStats();
      break;

    case MT_SYS_ZDIAGS_CLEAR_STATS:
      MT_SysZDiagsClearStats(pBuf);
      break;

    case MT_SYS_ZDIAGS_GET_STATS:
      MT_SysZDiagsGetStatsAttr(pBuf);
       break;

    case MT_SYS_ZDIAGS_RESTORE_STATS_NV:
      MT_SysZDiagsRestoreStatsFromNV();
      break;

    case MT_SYS_ZDIAGS_SAVE_STATS_TO_NV:
      MT_SysZDiagsSaveStatsToNV();
      break;
#endif /* FEATURE_SYSTEM_STATS */

    default:
      status = MT_RPC_ERR_COMMAND_ID;
      break;
  }

  return status;
}

/**************************************************************************************************
 * @fn      MT_SysReset
 *
 * @brief   Reset the device.
 * @param   typID: 0=reset, 1=serial bootloader, 
 *
 * @return  None
 *************************************************************************************************/
void MT_SysReset( uint8 *pBuf )
{
  switch( pBuf[MT_RPC_POS_DAT0] )
  {
    case MT_SYS_RESET_HARD:
      SystemReset();
      break;
      
    case MT_SYS_RESET_SOFT:
#if !(defined(HAL_BOARD_F2618) || defined(HAL_BOARD_F5438) || defined(HAL_BOARD_LM3S))
      SystemResetSoft();  // Especially useful for CC2531 to not break comm with USB Host.
#endif
      break;
      
    case MT_SYS_RESET_SHUTDOWN:
      {
        // Disable interrupts and put into deep sleep, use hardware reset to wakeup
        powerOffSoc();
      }
      break;
  }
}

/***************************************************************************************************
 * @fn      MT_SysPing
 *
 * @brief   Process the Ping command
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysPing(void)
{
  uint16 tmp16;
  uint8 retArray[2];

  /* Build Capabilities */
  tmp16 = MT_CAP_SYS | MT_CAP_MAC | MT_CAP_NWK | MT_CAP_AF | MT_CAP_ZDO |
          MT_CAP_SAPI | MT_CAP_UTIL | MT_CAP_DEBUG | MT_CAP_APP | MT_CAP_GP | MT_CAP_ZOAD;

  /* Convert to high byte first into temp buffer */
  retArray[0] = LO_UINT16( tmp16 );
  retArray[1] = HI_UINT16( tmp16 );

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), MT_SYS_PING,
                                sizeof (tmp16), retArray );
}

/***************************************************************************************************
 * @fn      MT_SysVersion
 *
 * @brief   Process the Version command
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysVersion(void)
{
#if !defined INCLUDE_REVISION_INFORMATION
  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), MT_SYS_VERSION,
                               sizeof(MTVersionString), (uint8 *)MTVersionString);

#else
  uint8 verStr[sizeof(MTVersionString) + 4];
  uint8 *pBuf = &verStr[sizeof(MTVersionString)];
  
#if (defined MAKE_CRC_SHDW) || (defined FAKE_CRC_SHDW)  //built for bootloader
  uint32 sblSig;
  uint32 sblRev;
#endif

  osal_memcpy(verStr, (uint8 *)MTVersionString, sizeof(MTVersionString));

#if (defined MAKE_CRC_SHDW) || (defined FAKE_CRC_SHDW)  //built for bootloader
  HalFlashRead(SBL_SIG_ADDR / HAL_FLASH_PAGE_SIZE,
               SBL_SIG_ADDR % HAL_FLASH_PAGE_SIZE,
               (uint8 *)&sblSig, sizeof(sblSig));

  if (sblSig == SBL_SIGNATURE)
  {
    // SBL is supported and its revision is provided (in a known flash location)
    HalFlashRead(SBL_REV_ADDR / HAL_FLASH_PAGE_SIZE,
                 SBL_REV_ADDR % HAL_FLASH_PAGE_SIZE,
                 (uint8 *)&sblRev, sizeof(sblRev));
  }
  else
  {
    //  SBL is supported but its revision is not provided
    sblRev = 0x00000000;
  }
#else
  // SBL is NOT supported
  sblRev = 0xFFFFFFFF;
#endif

  // Plug the SBL revision indication
  UINT32_TO_BUF_LITTLE_ENDIAN(pBuf,sblRev);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), MT_SYS_VERSION,
                               sizeof(verStr), verStr);

#endif
}

/***************************************************************************************************
 * @fn      MT_SysSetExtAddr
 *
 * @brief   Set the Extended Address
 *
 * @param   pBuf
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysSetExtAddr(uint8 *pBuf)
{
  uint8 retValue = ZFailure;
  uint8 cmdId;

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  if ( ZMacSetReq(ZMacExtAddr, pBuf) == ZMacSuccess )
  {
// CC253X MAC Network Processor does not have NV support
#if !defined(CC253X_MACNP)
    retValue = osal_nv_write(ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, pBuf);
#endif
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmdId, 1, &retValue);

}

/***************************************************************************************************
 * @fn      MT_SysGetExtAddr
 *
 * @brief   Get the Extended Address
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysGetExtAddr(void)
{
  uint8 extAddr[Z_EXTADDR_LEN];

  ZMacGetReq( ZMacExtAddr, extAddr );

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), MT_SYS_GET_EXTADDR,
                               Z_EXTADDR_LEN, extAddr);
}

#if !defined(CC253X_MACNP)
/***************************************************************************************************
 * @fn      MT_SysOsalNVRead
 *
 * @brief   Attempt to read an NV value
 *
 * @param   pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalNVRead(uint8 *pBuf)
{
  uint8 cmdId;
  uint16 nvId;
  uint16 dataLen;
  uint16 dataOfs;
  uint16 nvItemLen;
  uint8 error = ZSuccess;

  /* MT command ID */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  /* Skip over RPC header */
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* NV item ID */
  nvId = BUILD_UINT16(pBuf[0], pBuf[1]);
  /* Length of entire NV item data */
  nvItemLen = osal_nv_item_len(nvId);

#if !MT_SYS_OSAL_NV_READ_CERTIFICATE_DATA
  if ((ZCD_NV_IMPLICIT_CERTIFICATE == nvId) ||
      (ZCD_NV_CA_PUBLIC_KEY == nvId)        ||
      (ZCD_NV_DEVICE_PRIVATE_KEY == nvId))
  {
    /* Access to Security Certificate Data is denied */
    error = ZInvalidParameter;
  }
#endif

#if !MT_SYS_KEY_MANAGEMENT
  if ( (nvId == ZCD_NV_NWK_ACTIVE_KEY_INFO) ||
       (nvId == ZCD_NV_NWK_ALTERN_KEY_INFO) ||
      ((nvId >= ZCD_NV_TCLK_TABLE_START) && (nvId <= ZCD_NV_TCLK_TABLE_END)) ||
      ((nvId >= ZCD_NV_APS_LINK_KEY_DATA_START) && (nvId <= ZCD_NV_APS_LINK_KEY_DATA_END)) ||
       (nvId == ZCD_NV_PRECFGKEY) )
  {
    /* Access to Security Key Data is denied */
    error = ZInvalidParameter;
  }
#endif //!MT_SYS_KEY_MANAGEMENT
  
  /* Get NV data offset */
  if (cmdId == MT_SYS_OSAL_NV_READ)
  {
    /* MT_SYS_OSAL_NV_READ has 1-byte offset */
    dataOfs = (uint16)pBuf[2];
  }
  else
  {
    /* MT_SYS_OSAL_NV_READ_EXT has 2-byte offset */
    dataOfs = BUILD_UINT16(pBuf[2], pBuf[3]);
  }
  if (nvItemLen <= dataOfs)
  {
    /* Offset is past end of data */
    error = ZInvalidParameter;
  }

  if (error == ZSuccess)
  {
    uint8 *pRetBuf;
    uint8 respLen = 2;  /* Response header: [0]=status,[1]=length */

    dataLen = nvItemLen - dataOfs;
    if (dataLen > (uint16)(MT_MAX_RSP_DATA_LEN - respLen))
    {
      /* Data length is limited by TX buffer size and MT protocol */
      dataLen = (MT_MAX_RSP_DATA_LEN - respLen);
    }
    respLen += dataLen;

    pRetBuf = osal_mem_alloc(respLen);
    if (pRetBuf != NULL)
    {
      osal_memset(&pRetBuf[2], 0, dataLen);
      if (((osal_nv_read( nvId, dataOfs, dataLen, &pRetBuf[2])) == ZSUCCESS))
      {
        pRetBuf[0] = ZSuccess;
        pRetBuf[1] = dataLen;
        MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                       cmdId, respLen, pRetBuf );
      }
      else
      {
        error = NV_OPER_FAILED;
      }
      osal_mem_free(pRetBuf);
    }
    else
    {
      /* Could not get buffer for NV data */
      error = ZMemError;
    }
  }
  
  if (error != ZSuccess)
  {
    uint8 tmp[2] = { error, 0 };
    MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                   cmdId, 2, tmp);
  }
}

/***************************************************************************************************
 * @fn      MT_SysOsalNVWrite
 *
 * @brief   Attempt to write an NV item
 *
 * @param   pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalNVWrite(uint8 *pBuf)
{
  uint8 cmdId;
  uint16 nvId;
  uint16 dataLen;
  uint16 dataOfs;
  uint16 nvItemLen;
  uint8 rtrn = ZSuccess;

  /* MT command ID */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  /* Skip over RPC header */
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* NV item ID */
  nvId = BUILD_UINT16(pBuf[0], pBuf[1]);
  
  /* Get NV data offset & length */
  if ( cmdId == MT_SYS_OSAL_NV_WRITE )
  {
    /* MT_SYS_OSAL_NV_WRITE has 1-byte offset & length */
    dataOfs = (uint16)pBuf[2];
    dataLen = (uint16)pBuf[3];
    pBuf += 4;
  }
  else
  {
    /* MT_SYS_OSAL_NV_WRITE_EXT has 2-byte offset & length */
    dataOfs = BUILD_UINT16(pBuf[2], pBuf[3]);
    dataLen = BUILD_UINT16(pBuf[4], pBuf[5]);
    pBuf += 6;
  }

  /* Length of entire NV item data */
  nvItemLen = osal_nv_item_len(nvId);
  if ((dataOfs + dataLen) <= nvItemLen)
  {
    if (dataOfs == 0)
    {
      /* Set the Z-Globals value of this NV item */
      zgSetItem( nvId, dataLen, pBuf );
    }
    
    if ((osal_nv_write(nvId, dataOfs, dataLen, pBuf)) == ZSUCCESS)
    {
      if (nvId == ZCD_NV_EXTADDR)
      {
        rtrn = ZMacSetReq(ZMacExtAddr, pBuf);
      }
    }
    else
    {
      rtrn = NV_OPER_FAILED;
    }
  }
  else
  {
    /* Bad length or/and offset */
    rtrn = ZInvalidParameter;
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                 cmdId, 1, &rtrn);
}

/***************************************************************************************************
 * @fn      MT_SysOsalNVItemInit
 *
 * @brief   Attempt to create an NV item
 *
 * @param   pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalNVItemInit(uint8 *pBuf)
{
  uint8 ret;
  uint8 idLen;
  uint16 nvId;
  uint16 nvLen;

  /* Skip over RPC header */
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* NV item ID */
  nvId = BUILD_UINT16(pBuf[0], pBuf[1]);
  /* NV item length */
  nvLen = BUILD_UINT16(pBuf[2], pBuf[3]);
  /* Initialization data length */
  idLen = pBuf[4];
  pBuf += 5;

  if ( idLen < nvLen )
  {
    /* Attempt to create a new NV item */
    ret = osal_nv_item_init( nvId, nvLen, NULL );
    if ( (ret == NV_ITEM_UNINIT) && (idLen > 0) )
    {
      /* Write initialization data to first part of new item */
      (void) osal_nv_write( nvId, 0, (uint16)idLen, pBuf );
    }
  }
  else
  {
    /* Attempt to create/initialize a new NV item */
    ret = osal_nv_item_init( nvId, nvLen, pBuf );
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                 MT_SYS_OSAL_NV_ITEM_INIT, 1, &ret);
}

/***************************************************************************************************
 * @fn      MT_SysOsalNVDelete
 *
 * @brief   Attempt to delete an NV item
 *
 * @param   pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalNVDelete(uint8 *pBuf)
{
  uint16 nvId;
  uint16 nvLen;
  uint8 ret;

  /* Skip over RPC header */
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* Get the ID */
  nvId = BUILD_UINT16(pBuf[0], pBuf[1]);
  /* Get the length */
  nvLen = BUILD_UINT16(pBuf[2], pBuf[3]);

  /* Attempt to delete the NV item */
  ret = osal_nv_delete( nvId, nvLen );

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                 MT_SYS_OSAL_NV_DELETE, 1, &ret);
}

/***************************************************************************************************
 * @fn      MT_SysOsalNVLength
 *
 * @brief   Attempt to get the length to an NV item
 *
 * @param   pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalNVLength(uint8 *pBuf)
{
  uint16 nvId;
  uint16 nvLen;
  uint8 rsp[2];

  /* Skip over RPC header */
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* Get the ID */
  nvId = BUILD_UINT16(pBuf[0], pBuf[1]);

  /* Attempt to get NV item length */
  nvLen = osal_nv_item_len( nvId );
  rsp[0] = LO_UINT16( nvLen );
  rsp[1] = HI_UINT16( nvLen );

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                 MT_SYS_OSAL_NV_LENGTH, 2, rsp);
}
#endif // !defined(CC253X_MACNP)

/***************************************************************************************************
 * @fn      MT_SysOsalStartTimer
 *
 * @brief
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalStartTimer(uint8 *pBuf)
{
  uint16 eventId;
  uint8 retValue = ZFailure;
  uint8 cmdId;

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  if (*pBuf <= 3)
  {
    eventId = (uint16) MT_SysOsalEventId[*pBuf];
    retValue = osal_start_timerEx(MT_TaskID, eventId, BUILD_UINT16(pBuf[1], pBuf[2]));
  }
  else
  {
    retValue = ZInvalidParameter;
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmdId, 1, &retValue);
}

/***************************************************************************************************
 * @fn      MT_SysOsalStopTimer
 *
 * @brief
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysOsalStopTimer(uint8 *pBuf)
{
  uint16 eventId;
  uint8 retValue = ZFailure;
  uint8 cmdId;

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  if (*pBuf <= 3)
  {
    eventId = (uint16) MT_SysOsalEventId[*pBuf];
    retValue = osal_stop_timerEx(MT_TaskID, eventId);
  }
  else
  {
    retValue = ZInvalidParameter;
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmdId, 1, &retValue );
}

/***************************************************************************************************
 * @fn      MT_SysRandom
 *
 * @brief
 *
 * @param   uint8 pData - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysRandom()
{
  uint16 randValue = Onboard_rand();
  uint8 retArray[2];

  retArray[0] = LO_UINT16(randValue);
  retArray[1] = HI_UINT16(randValue);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), MT_SYS_RANDOM, 2, retArray );
}

/***************************************************************************************************
 * @fn      MT_SysAdcRead
 *
 * @brief   Reading ADC value, temperature sensor and voltage
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysAdcRead(uint8 *pBuf)
{
#ifndef HAL_BOARD_LM3S
  uint8 channel, resolution;
  uint16 tempValue;
  uint8 retArray[2];
  uint8 cmdId;

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* Channel */
  channel = *pBuf++;

  /* Resolution */
  resolution = *pBuf++;

  /* Voltage reading */
  switch (channel)
  {
    /* Analog input channel */
    case HAL_ADC_CHANNEL_0:
    case HAL_ADC_CHANNEL_1:
    case HAL_ADC_CHANNEL_2:
    case HAL_ADC_CHANNEL_3:
    case HAL_ADC_CHANNEL_4:
    case HAL_ADC_CHANNEL_5:
    case HAL_ADC_CHANNEL_6:
    case HAL_ADC_CHANNEL_7:
      tempValue = HalAdcRead(channel, resolution);
      break;

    /* Temperature sensor */
    case(HAL_ADC_CHANNEL_TEMP):
      tempValue = HalAdcRead(HAL_ADC_CHANNEL_TEMP, HAL_ADC_RESOLUTION_14);
      break;

    /* Voltage reading */
    case(HAL_ADC_CHANNEL_VDD):
      tempValue = HalAdcRead(HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_14);
      break;

    /* Undefined channels */
    default:
      tempValue = 0x00;
      break;
  }

  retArray[0] = LO_UINT16(tempValue);
  retArray[1] = HI_UINT16(tempValue);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmdId, 2, retArray);
#endif /* #ifndef HAL_BOARD_LM3S */
}

/**************************************************************************************************
 * @fn      MT_SysGpio
 *
 * @brief   ZAccel RPC interface for controlling the available GPIO pins.
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 *************************************************************************************************/
void MT_SysGpio(uint8 *pBuf)
{
  uint8 cmd, val;
  GPIO_Op_t op;

  cmd = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  op = (GPIO_Op_t)(*pBuf++);
  val = *pBuf;

  switch (op)
  {
    case GPIO_DIR:
      if (val & BV(0)) {GPIO_DIR_OUT(0);} else {GPIO_DIR_IN(0);}
      if (val & BV(1)) {GPIO_DIR_OUT(1);} else {GPIO_DIR_IN(1);}
      if (val & BV(2)) {GPIO_DIR_OUT(2);} else {GPIO_DIR_IN(2);}
      if (val & BV(3)) {GPIO_DIR_OUT(3);} else {GPIO_DIR_IN(3);}
      break;

    case GPIO_TRI:
      if(val & BV(0)) {GPIO_TRI(0);} else if(val & BV(4)) {GPIO_PULL_DN(0);} else {GPIO_PULL_UP(0);}
      if(val & BV(1)) {GPIO_TRI(1);} else if(val & BV(5)) {GPIO_PULL_DN(1);} else {GPIO_PULL_UP(1);}
      if(val & BV(2)) {GPIO_TRI(2);} else if(val & BV(6)) {GPIO_PULL_DN(2);} else {GPIO_PULL_UP(2);}
      if(val & BV(3)) {GPIO_TRI(3);} else if(val & BV(7)) {GPIO_PULL_DN(3);} else {GPIO_PULL_UP(3);}
      break;

    case GPIO_SET:
      if (val & BV(0)) {GPIO_SET(0);}
      if (val & BV(1)) {GPIO_SET(1);}
      if (val & BV(2)) {GPIO_SET(2);}
      if (val & BV(3)) {GPIO_SET(3);}
      break;

    case GPIO_CLR:
      if (val & BV(0)) {GPIO_CLR(0);}
      if (val & BV(1)) {GPIO_CLR(1);}
      if (val & BV(2)) {GPIO_CLR(2);}
      if (val & BV(3)) {GPIO_CLR(3);}
      break;

    case GPIO_TOG:
      if (val & BV(0)) {GPIO_TOG(0);}
      if (val & BV(1)) {GPIO_TOG(1);}
      if (val & BV(2)) {GPIO_TOG(2);}
      if (val & BV(3)) {GPIO_TOG(3);}
      break;

    case GPIO_GET:
      break;

    case GPIO_HiD:
      (val) ? GPIO_HiD_SET() :  GPIO_HiD_CLR();
      break;

    default:
      break;
  }

  val  = (GPIO_GET(0)) ? BV(0) : 0;
  val |= (GPIO_GET(1)) ? BV(1) : 0;
  val |= (GPIO_GET(2)) ? BV(2) : 0;
  val |= (GPIO_GET(3)) ? BV(3) : 0;

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmd, 1, &val);
}

/**************************************************************************************************
 * @fn      MT_SysStackTune
 *
 * @brief   ZAccel RPC interface for tuning the stack parameters to adjust performance
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 *************************************************************************************************/
void MT_SysStackTune(uint8 *pBuf)
{
  uint8 cmd, rtrn;

  cmd = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  switch (*pBuf++)
  {
  case STK_TX_PWR:
    rtrn = ZMacSetReq(ZMacPhyTransmitPowerSigned, pBuf);
    break;

  case STK_RX_ON_IDLE:
    if ((*pBuf != TRUE) && (*pBuf != FALSE))
    {
      (void)ZMacGetReq(ZMacRxOnIdle, &rtrn);
    }
    else
    {
      rtrn = ZMacSetReq(ZMacRxOnIdle, pBuf);
    }
    break;

  default:
    rtrn = ZInvalidParameter;
    break;
  }

  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS), cmd, 1, &rtrn);
}

/***************************************************************************************************
 * @fn      MT_SysSetUtcTime
 *
 * @brief   Set the OSAL UTC Time. UTC rollover is: 06:28:16 02/07/2136
 *
 * @param   pBuf - pointer to time parameters
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysSetUtcTime(uint8 *pBuf)
{
  uint8 cmdId;
  uint8 retStat;
  UTCTime utcSecs;

  /* Parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  utcSecs = osal_build_uint32( pBuf, 4 );
  if ( utcSecs == 0 )
  {
    UTCTimeStruct utc;

    /* Skip past UTC time */
    pBuf += 4;

    /* Get time and date parameters */
    utc.hour = *pBuf++;
    utc.minutes = *pBuf++;
    utc.seconds = *pBuf++;
    utc.month = (*pBuf++) - 1;
    utc.day = (*pBuf++) - 1;
    utc.year = osal_build_uint16 ( pBuf );

    if ((utc.hour < 24) && (utc.minutes < 60) && (utc.seconds < 60) &&
        (utc.month < 12) && (utc.day < 31) && (utc.year > 1999) && (utc.year < 2136))
    {
      /* Got past the course filter, now check for leap year */
      if ((utc.month != 1) || (utc.day < (IsLeapYear( utc.year ) ? 29 : 28)))
      {
        /* Numbers look reasonable, convert to UTC */
        utcSecs = osal_ConvertUTCSecs( &utc );
      }
    }
  }

  if ( utcSecs == 0 )
  {
    /* Bad parameter(s) */
    retStat = ZInvalidParameter;
  }
  else
  {
    /* Parameters accepted, set the time */
    osal_setClock( utcSecs );
    retStat = ZSuccess;
  }

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                 cmdId, 1, &retStat);
}

/***************************************************************************************************
 * @fn      MT_SysGetUtcTime
 *
 * @brief   Get the OSAL UTC time
 *
 * @param   None
 *
 * @return  32-bit and Parsed UTC time
 ***************************************************************************************************/
void MT_SysGetUtcTime(void)
{
  uint8 len;
  uint8 *buf;

  len = sizeof( UTCTime ) + sizeof( UTCTimeStruct );

  buf = osal_mem_alloc( len );
  if ( buf )
  {
    uint8 *pBuf;
    UTCTime utcSecs;
    UTCTimeStruct utcTime;

    // Get current 32-bit UTC time and parse it
    utcSecs = osal_getClock();
    osal_ConvertUTCTime( &utcTime, utcSecs );

    // Start with 32-bit UTC time
    pBuf = osal_buffer_uint32( buf, utcSecs );

    // Concatenate parsed UTC time fields
    *pBuf++ = utcTime.hour;
    *pBuf++ = utcTime.minutes;
    *pBuf++ = utcTime.seconds;
    *pBuf++ = utcTime.month + 1;  // Convert to human numbers
    *pBuf++ = utcTime.day + 1;
    *pBuf++ = LO_UINT16( utcTime.year );
    *pBuf++ = HI_UINT16( utcTime.year );

    /* Build and send back the response */
    MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                   MT_SYS_GET_TIME, (uint8)(pBuf-buf), buf);

    osal_mem_free( buf );
  }
}

/***************************************************************************************************
 * @fn      MT_SysSetTxPower
 *
 * @brief   Set the transmit power.
 *
 * @param   pBuf - MT message containing the ZMacTransmitPower_t power level to set.
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysSetTxPower(uint8 *pBuf)
{
  /* A local variable to hold the signed dBm value of TxPower that is being requested. */
  uint8 signed_dBm_of_TxPower_requeseted;

  /*
   * A local variable to hold the signed dBm value of TxPower that can be set which is closest to
   * the requested dBm value of TxPower, but which is also valid according to a complex set of
   * compile-time and run-time configuration which is interpreted by the macRadioSetTxPower()
   * function.
   */
  uint8 signed_dBm_of_TxPower_range_corrected;

  /* Parse the requested dBm from the RPC message. */
  signed_dBm_of_TxPower_requeseted = pBuf[MT_RPC_POS_DAT0];

  /*
   * MAC_MlmeSetReq() will store an out-of-range dBm parameter value into the NIB. So it is not
   * possible to learn the actual dBm value that will be set by invoking MACMlmeGetReq().
   * But this actual dBm value is a required return value in the SRSP to this SREQ. Therefore,
   * it is necessary to make this redundant pre-call to macRadioSetTxPower() here in order to run
   * the code that will properly constrain the requested dBm to a valid range based on both the
   * compile-time and the run-time configurations that affect the available valid ranges
   * (i.e. MAC_MlmeSetReq() itself will invoke for a second time the macRadioSetTxPower() function).
   */
  signed_dBm_of_TxPower_range_corrected = macRadioSetTxPower(signed_dBm_of_TxPower_requeseted);

  /*
   * Call the function to store the requested dBm in the MAC PIB and to set the TxPower as closely
   * as possible within the TxPower range that is valid for the compile-time and run-time
   * configuration.
   */
  (void)MAC_MlmeSetReq(MAC_PHY_TRANSMIT_POWER_SIGNED, &signed_dBm_of_TxPower_requeseted);

  /* Build and send back the response that includes the actual dBm TxPower that can be set. */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                       MT_SYS_SET_TX_POWER, 1,
                                       &signed_dBm_of_TxPower_range_corrected);
}

#if defined ( FEATURE_SYSTEM_STATS )
/***************************************************************************************************
 * @fn      MT_SysZDiagsInitStats
 *
 * @brief   Initialize the statistics table in NV or restore values from
 *          NV into the Statistics table in RAM
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysZDiagsInitStats(void)
{
  uint8 retValue;

  retValue = ZDiagsInitStats();

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                MT_SYS_ZDIAGS_INIT_STATS, 1, &retValue);
}

/***************************************************************************************************
 * @fn      MT_SysZDiagsClearStats
 *
 * @brief   Clears the statistics table in RAM and NV if option flag set.
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysZDiagsClearStats(uint8 *pBuf)
{
  uint8 cmdId;
  uint8 clearNV;
  uint32 sysClock;
  uint8 retBuf[4];

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  clearNV = *pBuf;

  /* returns the system clock of the time when the statistics were cleared */
  sysClock = ZDiagsClearStats( clearNV );

  retBuf[0] = BREAK_UINT32( sysClock, 0);
  retBuf[1] = BREAK_UINT32( sysClock, 1);
  retBuf[2] = BREAK_UINT32( sysClock, 2);
  retBuf[3] = BREAK_UINT32( sysClock, 3);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                               cmdId, 4, retBuf);
}

/***************************************************************************************************
 * @fn      MT_SysZDiagsGetStatsAttr
 *
 * @brief   Reads specific system (attribute) ID statistics and/or metrics.
 *
 * @param   uint8 pBuf - pointer to the data
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysZDiagsGetStatsAttr(uint8 *pBuf)
{
  uint8 cmdId;
  uint16 attributeId;
  uint32 attrValue;
  uint8 retBuf[4];

  /* parse header */
  cmdId = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  /* Get the Attribute ID */
  attributeId = BUILD_UINT16(pBuf[0], pBuf[1]);

  attrValue = ZDiagsGetStatsAttr( attributeId );

  retBuf[0] = BREAK_UINT32( attrValue, 0);
  retBuf[1] = BREAK_UINT32( attrValue, 1);
  retBuf[2] = BREAK_UINT32( attrValue, 2);
  retBuf[3] = BREAK_UINT32( attrValue, 3);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                               cmdId, 4, retBuf);
}

/***************************************************************************************************
 * @fn      MT_SysZDiagsRestoreStatsFromNV
 *
 * @brief   Restores the statistics table from NV into the RAM table.
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysZDiagsRestoreStatsFromNV(void)
{
  uint8 retValue;

  retValue = ZDiagsRestoreStatsFromNV();

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                MT_SYS_ZDIAGS_RESTORE_STATS_NV, 1, &retValue);
}

/***************************************************************************************************
 * @fn      MT_SysZDiagsSaveStatsToNV
 *
 * @brief   Saves the statistics table from RAM to NV.
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysZDiagsSaveStatsToNV(void)
{
  uint32 sysClock;
  uint8 retBuf[4];

  /* returns the system clock of the time when the statistics were saved to NV */
  sysClock = ZDiagsSaveStatsToNV();

  retBuf[0] = BREAK_UINT32( sysClock, 0);
  retBuf[1] = BREAK_UINT32( sysClock, 1);
  retBuf[2] = BREAK_UINT32( sysClock, 2);
  retBuf[3] = BREAK_UINT32( sysClock, 3);

  /* Build and send back the response */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                               MT_SYS_ZDIAGS_SAVE_STATS_TO_NV, 4, retBuf);
}
#endif /* FEATURE_SYSTEM_STATS */

#endif /* MT_SYS_FUNC */

/***************************************************************************************************
 * SUPPORT
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      MT_SysResetInd()
 *
 * @brief   Sends a ZTOOL "reset response" message.
 *
 * @param   None
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_SysResetInd(void)
{
  uint8 retArray[6];

  retArray[0] = ResetReason();   /* Reason */
  osal_memcpy( &retArray[1], MTVersionString, 5 );   /* Revision info */

  /* Send out Reset Response message */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_AREQ | (uint8)MT_RPC_SYS_SYS), MT_SYS_RESET_IND,
                                sizeof(retArray), retArray);
}

/***************************************************************************************************
 * @fn      MT_SysOsalTimerExpired()
 *
 * @brief   Sends a SYS Osal Timer Expired
 *
 * @param   None
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_SysOsalTimerExpired(uint8 Id)
{
  uint8 retValue;
  retValue = Id;
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_AREQ | (uint8)MT_RPC_SYS_SYS), MT_SYS_OSAL_TIMER_EXPIRED, 1, &retValue);
}

#if defined ( MT_SYS_JAMMER_FEATURE )
/***************************************************************************************************
 * @fn      MT_SysJammerParameters
 *
 * @brief   Set the Jammer detection parameters.
 *
 * @param   pBuf - MT message containing the parameters.
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysJammerParameters( uint8 *pBuf )
{
  uint8 status = SUCCESS;
  
  // Adjust for the data
  pBuf += MT_RPC_FRAME_HDR_SZ;
  
  // Number of continuous events needed to detect Jam
  jammerContinuousEvents = BUILD_UINT16( pBuf[0], pBuf[1] );
  jammerDetections = jammerContinuousEvents;
  pBuf += 2;
  
  // Noise Level need to be a Jam
  jammerHighNoiseLevel = *pBuf++;
  
  // The time between each noise level reading
  jammerDetectPeriodTime = BUILD_UINT32( pBuf[0], pBuf[1], pBuf[2], pBuf[3] );
  
  // Update the timer
  osal_start_reload_timer( jammerTaskID, JAMMER_CHECK_EVT, jammerDetectPeriodTime );
  
  /* Build and send back the response that includes the actual dBm TxPower that can be set. */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                       MT_SYS_JAMMER_PARAMETERS, 1, &status );
}

/***************************************************************************************************
 * @fn      MT_SysJammerInd()
 *
 * @brief   Sends a jammer indication message.
 *
 * @param   jammerInd - TRUE if jammer detected, FALSE if changed to undetected
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_SysJammerInd( uint8 jammerInd )
{
  /* Send out Reset Response message */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_AREQ | (uint8)MT_RPC_SYS_SYS), MT_SYS_JAMMER_IND,
                                1, &jammerInd );
}

/***************************************************************************************************
 * @fn      jammerInit()
 *
 * @brief   Jammer Detection task initialization function
 *
 * @param   taskId - task ID 
 *
 * @return  None
 *
 ***************************************************************************************************/
void jammerInit( uint8 taskId )
{
  jammerTaskID = taskId; 
  
  // Start the jammer check timer
  osal_start_reload_timer( taskId, JAMMER_CHECK_EVT, jammerDetectPeriodTime );
}

/***************************************************************************************************
 * @fn      jammerEventLoop()
 *
 * @brief   Jammer Detection task event processing function
 *
 * @param   taskId - task ID 
 * @param   events - task events 
 *
 * @return  remaining events
 *
 ***************************************************************************************************/
uint16 jammerEventLoop( uint8 taskId, uint16 events )
{
  osal_event_hdr_t  *pMsg;

  if (events & SYS_EVENT_MSG)
  {
    if ( (pMsg = (osal_event_hdr_t *) osal_msg_receive( taskId )) != NULL )
    {
      switch ( pMsg->event )
      {
        default:
          break;
      }

      osal_msg_deallocate( (byte *)pMsg );
    }

    events ^= SYS_EVENT_MSG;
  }
  else if ( events & JAMMER_CHECK_EVT )
  {
    // Make sure we aren't currently receiving a message
    // and the radio is active.
    if ( MAC_RX_IS_PHYSICALLY_ACTIVE() == MAC_RX_ACTIVE_NO_ACTIVITY )
    {
      int8 rssiDbm = -128;
      
      // Read the noise level
      if ( RSSISTAT & 0x01 )
      {
        /* Add the RSSI offset */  
        rssiDbm = RSSI + MAC_RADIO_RSSI_OFFSET;

        /* Adjust for external PA/LNA, if any */
        MAC_RADIO_RSSI_LNA_OFFSET( rssiDbm );
      
        // Check for a noise level that's high
        if ( jammerDetections && (rssiDbm  > jammerHighNoiseLevel) )
        {
          jammerDetections--;
          if ( jammerDetections == 0 )
          {
            // Jam detected
            MT_SysJammerInd( TRUE );
          }
        }
        else if ( rssiDbm <= jammerHighNoiseLevel )
        {
          if ( jammerDetections == 0 )
          {
            // Jam cleared
            MT_SysJammerInd( FALSE );
          }
          jammerDetections = jammerContinuousEvents;
        }
      }
    }
    events ^= JAMMER_CHECK_EVT;
  }
  else
  {
    events = 0;  /* Discard unknown events. */
  }

  return ( events );
}
#endif // MT_SYS_JAMMER_FEATURE

#if defined ( MT_SYS_SNIFFER_FEATURE )
/***************************************************************************************************
 * @fn      MT_SysSnifferParameters
 *
 * @brief   Set the sniffer parameters.
 *
 * @param   pBuf - MT message containing the parameters.
 *
 * @return  None
 ***************************************************************************************************/
void MT_SysSnifferParameters( uint8 *pBuf )
{
  uint8 status = SUCCESS;
  uint8 param;
  
  // Adjust for the data
  pBuf += MT_RPC_FRAME_HDR_SZ;
  
  // Noise Level need to be a Jam
  param = *pBuf++;
  
  if ( param == MT_SYS_SNIFFER_DISABLE )
  {
    // Disable Sniffer
    HAL_BOARD_DISABLE_INTEGRATED_SNIFFER();
    sniffer = FALSE;
  }
  else if ( param == MT_SYS_SNIFFER_ENABLE )
  {
    // Enable the Sniffer
    HAL_BOARD_ENABLE_INTEGRATED_SNIFFER();
    sniffer = TRUE;
  }
  else if ( param == MT_SYS_SNIFFER_GET_SETTING )
  {
    status = sniffer; // sniffer setting
  }
  else 
  {
    status = INVALIDPARAMETER;
  }
  
  /* Build and send back the response that includes the actual dBm TxPower that can be set. */
  MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_SYS),
                                       MT_SYS_SNIFFER_PARAMETERS, 1, &status );
}
#endif // MT_SYS_SNIFFER_FEATURE

/**************************************************************************************************
 * @fn          powerOffSoc
 *
 * @brief       put the device in lowest power mode infinitely.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void powerOffSoc(void)
{
#ifdef ENABLE_MT_SYS_RESET_SHUTDOWN
  HAL_DISABLE_INTERRUPTS();

  /* turn off the RF front end device */
  //TBD, based on the rf-front-end being used

  /* turn off the receiver */
  MAC_RADIO_RXTX_OFF();
  
  /* just in case a receive was about to start, flush the receive FIFO */
  MAC_RADIO_FLUSH_RX_FIFO();
  
  /* clear any receive interrupt that happened to squeak through */
  MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();
  
  /* put MAC timer to sleep */
  MAC_RADIO_TIMER_SLEEP();
  
  /* power of radio */
  MAC_RADIO_TURN_OFF_POWER();
  
  STIF = 0; //HAL_SLEEP_TIMER_CLEAR_INT;
  
  if (ZNP_CFG1_UART == znpCfg1)
  {
    HalUARTSuspend();
  }
  
  /* Prep CC2530 power mode */
  //HAL_SLEEP_PREP_POWER_MODE(3);
  SLEEPCMD &= ~PMODE; /* clear mode bits */
  SLEEPCMD |= 3;      /* set mode bits  to PM3 */
  while (!(STLOAD & LDRDY));
  
  while (1) //just in case we wake up for some unknown reason
  {
    PCON = halSleepPconValue; //execution is suppose to halt with this command. Interrupts are disabled, so the only way to exit this state is using a hardware reset.
    asm("NOP");
  }
#endif
}

/***************************************************************************************************
 ***************************************************************************************************/
