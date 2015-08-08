/**************************************************************************************************
  Filename:       zcl_ezmode.c
  Revised:        $Date: 2014-06-10 10:56:57 -0700 (Tue, 10 Jun 2014) $
  Revision:       $Revision: 38928 $

  Description:    Zigbee Cluster Library - EZ Mode


  Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"

#if defined ( INTER_PAN )
  #include "stub_aps.h"
#endif

#ifdef ZCL_EZMODE

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL PROTOTYPES
 */
static void zcl_SetEZModeError( uint8 errorCode );
static void zcl_SetEZModeState( zlcEZMode_State_t newState );
static void zcl_ProcessEZMode( void );


/*********************************************************************
 * GLOBAL VARIABLES
 */

// internal EZ-Mode state machine
uint8  zclEZModeRegistered;
uint8  zclEZModeErr;
uint8  zclEZModeState;
uint8  zclEZModeOpener;
uint8  zclEZModeMatched;              // we were matched by a remote node
uint16 zclEZModeQueryRspNwkAddr;      // short address (on QueryRsp)
uint8  zclEZModeQueryRspEP;           // endpoint (on QueryRsp)
zclEZMode_RegisterData_t zclEZModeRegisterData;   // registered once on init
zclEZMode_InvokeData_t   zclEZModeInvokeData;     // user's configuration parameters (what endpoint, initiator, etc...)


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zcl_RegisterEZMode
 *
 * @brief   Called upon task initialation, to initialize EZ-Mode.
 *
 * @param   pData - task ID, App Callback routine, etc..
 *
 * @return  none
 */
void zcl_RegisterEZMode( zclEZMode_RegisterData_t const *pData )
{
  // make a copy of the data. Cannot fail.
  osal_memcpy( &zclEZModeRegisterData, pData, sizeof(zclEZMode_RegisterData_t) );
  zclEZModeRegistered = TRUE;
}

/*********************************************************************
 * @fn      zcl_InvokeEZMode
 *
 * @brief   Called to invoke EZ-Mode on an endpoint. This is a toggle
 *          (will cancel if EZ-Mode currently enabled). Note: there is only 1
 *          state machine. EZ-Mode can only be invoked on 1 endpoint at a time.
 *
 * @param   none
 *
 * @return  none
 */
void zcl_InvokeEZMode( zclEZMode_InvokeData_t *pData )
{
  // if not registered, do nothing
  if(!zclEZModeRegistered)
  {
    return;
  }

  // there is only 1 EZ-Mode state machine. If already in EZ-Mode, cancel it
  if(zclEZModeState != EZMODE_STATE_READY)
  {
    zcl_SetEZModeError ( EZMODE_ERR_CANCELLED );
    zcl_SetEZModeState ( EZMODE_STATE_FINISH );  // needed to shut down timers, turn off joining, etc...
    return;
  }

  // copy the data, so we remember which endpoint, etc...
  osal_memcpy( &zclEZModeInvokeData, pData, sizeof(zclEZMode_InvokeData_t) );

  // start with no error, and no QueryResponses in our list
  zcl_SetEZModeError ( EZMODE_ERR_SUCCESS );
  zclEZModeOpener = zclEZModeMatched = 0;

  // if already on network, just go to identify state
  if ( zclEZModeInvokeData.onNetwork )
  {
    zcl_SetEZModeState( EZMODE_STATE_OPENER );
  }

  // not already on network, form/join a network
  else
  {
    zcl_SetEZModeState( EZMODE_STATE_JOINER );
  }

  // start a total timeout for EZ_Mode (will cancel if not finished in this time)
  osal_start_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.timeoutEvt, EZMODE_TIME );
}

/*********************************************************************
 * @fn      zcl_EZModeAction
 *
 * @brief   Called when the application needs to inform EZ-Mode of some action
 *          (now on the network, identify mode query, etc...)
 *
 * @param   action - which action has taken place
 *          pData  - the data unique to the action
 *
 * @return  none
 */
void zcl_EZModeAction(zclEzMode_Action_t action, zclEZMode_ActionData_t *pData)
{
  ZDO_MatchDescRsp_t *pMatchDescRsp;
  zAddrType_t dstAddr;

  // not in the EZ-Mode state machine, so do nothing
  if( zclEZModeState == EZMODE_STATE_READY )
    return;

  switch ( action )
  {
    case EZMODE_ACTION_PROCESS:
      zcl_ProcessEZMode();  // process next state
    break;

    case EZMODE_ACTION_NETWORK_STARTED:
      // once on the network, time to go on to the identify state
      if( zclEZModeState == EZMODE_STATE_JOINER )
      {
        // set local permit joining on locally only for joiners (openers turn it on across the network)
        NLME_PermitJoiningRequest( (byte)(EZMODE_TIME / 1000) ); // in seconds
        zcl_SetEZModeState( EZMODE_STATE_IDENTIFYING );
      }
    break;

    // received identify query
    case EZMODE_ACTION_IDENTIFY_QUERY:

      // targets just go to autoclose once they have been identified
      if ( !zclEZModeInvokeData.initiator )
      {
        zcl_SetEZModeState( EZMODE_STATE_AUTOCLOSE );
      }
    break;

    // received identify query response
    case EZMODE_ACTION_IDENTIFY_QUERY_RSP:

      // remember the node we found via identify query
      zclEZModeQueryRspNwkAddr = pData->pIdentifyQueryRsp->srcAddr->addr.shortAddr;
      zclEZModeQueryRspEP = pData->pIdentifyQueryRsp->srcAddr->endPoint;

      // initiate match descriptor request on the remote node
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = zclEZModeQueryRspNwkAddr;
      ZDP_MatchDescReq( &dstAddr, zclEZModeQueryRspNwkAddr,
                        ZCL_HA_PROFILE_ID,
                        zclEZModeInvokeData.numActiveOutClusters, zclEZModeInvokeData.pActiveOutClusterIDs,
                        zclEZModeInvokeData.numActiveInClusters, zclEZModeInvokeData.pActiveInClusterIDs,
                        FALSE );
      zcl_SetEZModeState( EZMODE_STATE_WAITING_MATCHDESCRSP );
    break;

    // received match descriptor response, see if active clusters match
    case EZMODE_ACTION_MATCH_DESC_RSP:

      pMatchDescRsp = pData->pMatchDescRsp;
      if ( ( pMatchDescRsp && pMatchDescRsp->status == ZSuccess ) && ( pMatchDescRsp->cnt>0 ) )
      {
        zclEZModeMatched = TRUE;

        // BindingEntry_t *bindAddEntry( byte srcEpInt, zAddrType_t *dstAddr, byte dstEpInt, byte numClusterIds, uint16 *clusterIds )
        dstAddr.addr.shortAddr = zclEZModeQueryRspNwkAddr;
        dstAddr.addrMode = Addr16Bit;

        // bind each matching input cluster
        if ( zclEZModeInvokeData.numActiveInClusters )
        {
          bindAddEntry( zclEZModeInvokeData.endpoint, &dstAddr, zclEZModeQueryRspEP,
                        zclEZModeInvokeData.numActiveInClusters, zclEZModeInvokeData.pActiveInClusterIDs );
        }

        // bind each matching output cluster
        if ( zclEZModeInvokeData.numActiveOutClusters )
        {
          bindAddEntry( zclEZModeInvokeData.endpoint, &dstAddr, zclEZModeQueryRspEP,
                        zclEZModeInvokeData.numActiveOutClusters, zclEZModeInvokeData.pActiveOutClusterIDs );
        }
      }

      // time to close (wait a bit before finishing, to allow for multiple initiators)
      zcl_SetEZModeState( EZMODE_STATE_AUTOCLOSE );
    break;

    // timed out of EZ-Mode
    case EZMODE_ACTION_TIMED_OUT:
      // timed out
      if(zclEZModeState != EZMODE_STATE_READY)
      {
        zcl_SetEZModeError( EZMODE_ERR_TIMEDOUT );
        zcl_SetEZModeState( EZMODE_STATE_FINISH );
      }
    break;
  }   // switch ( action )

}

/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zcl_SetEZModeState
 *
 * @brief   Move on to new state after a short wait.
 *
 * @param   none
 *
 * @return  none
 */
static void zcl_SetEZModeState( zlcEZMode_State_t newState )
{
  zclEZModeState = newState;
  osal_start_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.processEvt, 5 );
}

/*********************************************************************
 * @fn      zclSampleSw_SetEZModeError
 *
 * @brief   Called to set error code that will be reported on finish. Starts as EZMODE_ERR_SUCCESS.
 *
 * @param   none
 *
 * @return  none
 */
static void zcl_SetEZModeError( uint8 errorCode )
{
  zclEZModeErr = errorCode;
}

/*********************************************************************
 * @fn      zcl_ProcessEZMode
 *
 * @brief   Called when EZ-Mode changes state. See EZMODE_STATE_xxxx in zcl_ezmode.h
 *
 * @param   none
 *
 * @return  status
 */
static void zcl_ProcessEZMode( void )
{
  zAddrType_t dstAddr;
  afAddrType_t afDstAddr;
  zclEZMode_CBData_t cbData;

  dstAddr.addr.shortAddr = 0xfffc;        // all routers (for PermitJoin) devices
  dstAddr.addrMode = AddrBroadcast;

  afDstAddr.addr.shortAddr = 0xffff;      // all devices (for IdentifyQuery)
  afDstAddr.addrMode = afAddrBroadcast;
  afDstAddr.endPoint = 0xff;

  switch(zclEZModeState)
  {
    // openers will broadcast permit joining
    case EZMODE_STATE_OPENER:
      zclEZModeOpener = 1;

      // enable joining both locally and over-the-air
      NLME_PermitJoiningRequest( (byte)(EZMODE_TIME / 1000)  );
      ZDP_MgmtPermitJoinReq( &dstAddr, (byte)(EZMODE_TIME / 1000), TRUE, FALSE);

      // then go to identifying state
      zcl_SetEZModeState(EZMODE_STATE_IDENTIFYING);
    break;

    // joiners will try to join the network, and if success will go to identifying state
    case EZMODE_STATE_JOINER:
      zclEZModeOpener = 0;
      ZDOInitDevice(0);   // see ZDO_STATE_CHANGE in zclSampleSw_event_loop()
    break;

    // go into identify state
    case EZMODE_STATE_IDENTIFYING:

      // tell app to go into identify mode
      if ( zclEZModeRegisterData.pfnNotifyCB )
      {
        (*zclEZModeRegisterData.pfnNotifyCB)( zclEZModeState, NULL );
      }

      // initiators start looking for other nodes in identify mode
      if ( zclEZModeInvokeData.initiator )
      {
        zcl_SetEZModeState ( EZMODE_STATE_WAITING_IDENTIFYQUERYRSP );
      }
    break;

    // timeout out with no query response, send another
    case EZMODE_STATE_WAITING_IDENTIFYQUERYRSP:
      // ZStatus_t zclGeneral_SendIdentifyQuery( uint8 srcEP, afAddrType_t *dstAddr, uint8 disableDefaultRsp, uint8 seqNum );
      // NOTE: Ensure that Identify Cluster is enabled to use this function for EZ-Mode
      zclGeneral_SendIdentifyQuery( zclEZModeInvokeData.endpoint, &afDstAddr, TRUE, (*zclEZModeRegisterData.pZclSeqNum)++ );

      // wait some time before sending out the next IdentifyQuery, will stop when we get a response
      osal_start_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.processEvt, EZMODE_IDQUERYTIME );
      break;

    // waiting for simple descriptor response
    case EZMODE_STATE_WAITING_MATCHDESCRSP:
    break;

    // if waiting on autoclose, then we're done. Go to success.
    case EZMODE_STATE_AUTOCLOSE:

      // special case: if 2 initators, we only fail if no match from either side
      if( zclEZModeInvokeData.initiator && !zclEZModeMatched )
      {
        zcl_SetEZModeError ( EZMODE_ERR_NOMATCH );
      }

      // if user specified callback, call on AutoClose
      if ( zclEZModeRegisterData.pfnNotifyCB )
      {
        cbData.sAutoClose.err = zclEZModeErr;
        (*zclEZModeRegisterData.pfnNotifyCB)( zclEZModeState, &cbData );
      }

      // no longer will timeout, since cannot fail
      osal_stop_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.timeoutEvt );

      // wait a little to turn off identify mode, to give time for the other side to discover
      // in case of complex devices (both target/initiator)
      osal_start_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.processEvt, EZMODE_AUTOCLOSETIME );

      // go to finish state after autoclose. Don't use zcl_SetEZModeState() because we don't want it to happen immediately
      zclEZModeState = EZMODE_STATE_FINISH;
    break;

    case EZMODE_STATE_FINISH:

      // no longer will timeout, since we're done
      osal_stop_timerEx( *zclEZModeRegisterData.pTaskID, zclEZModeRegisterData.timeoutEvt );

      // if we opened the network, close it now (turn off joining)
      if ( zclEZModeOpener )
      {
        ZDP_MgmtPermitJoinReq( &dstAddr, 0, TRUE, FALSE);
      }

      // if user callback, inform them of the finish, which will also turn off identify
      if ( zclEZModeRegisterData.pfnNotifyCB )
      {
        cbData.sFinish.err = zclEZModeErr;
        cbData.sFinish.ep = zclEZModeQueryRspEP;
        cbData.sFinish.nwkaddr = zclEZModeQueryRspNwkAddr;
        (*zclEZModeRegisterData.pfnNotifyCB)( zclEZModeState, &cbData );
      }

      // done, back to ready state
      zclEZModeState = EZMODE_STATE_READY;
    break;
  }

}

#endif // ZCL_EZMODE
