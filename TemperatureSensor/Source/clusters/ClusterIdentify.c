/**************************************************************************************************
  Filename:       ClusterIdentify.cpp

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "hal_led.h"

#include "ClusterIdentify.h"

#define ON_TIME 600
#define OFF_TIME 400

uint16 identifyTime=0;

extern byte temperatureSensorTaskID;

static uint8  onOff;

void identifyInit(){
	P0DIR |= 1;
 	P0SEL &= 0xFE;
 	P0_0 = 0;
}

uint16 identifyLoop(uint16 events){
	if (onOff){
		osal_start_timerEx( temperatureSensorTaskID, IDENTIFY_TIMEOUT_EVT, OFF_TIME );
		onOff=0;
		P0_0 = 1;
	} else {
		onOff=1;
		if ( identifyTime > 0 ){
    		identifyTime--;
		}
    	if (identifyTime>0){
			osal_start_timerEx( temperatureSensorTaskID, IDENTIFY_TIMEOUT_EVT, ON_TIME );
			osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_HOLD);
			P0_0 = 0;
		} else{
			osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_CONSERVE);
			P0_0 = 0;
		}
	}
    return ( events ^ IDENTIFY_TIMEOUT_EVT );
}


/*********************************************************************
 * @fn      processIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
void processIdentifyTimeChange( void ){
	if ( identifyTime > 0 ) {
		osal_start_timerEx( temperatureSensorTaskID, IDENTIFY_TIMEOUT_EVT, ON_TIME );
		onOff=1;
		P0_0 = 1;
		osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_HOLD);
	}  else {
		osal_stop_timerEx( temperatureSensorTaskID, IDENTIFY_TIMEOUT_EVT );
		osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_CONSERVE);
	}
}

ZStatus_t processIdentifyClusterServerCommands( zclIncoming_t *pInMsg ){

	switch(pInMsg->hdr.commandID){
	case COMMAND_IDENTIFY:
		identifyTime = BUILD_UINT16( pInMsg->pData[0], pInMsg->pData[1] );
		processIdentifyTimeChange();
		return ZSuccess;
	case COMMAND_IDENTIFY_QUERY:
		if ( identifyTime > 0 ) {
			zcl_SendCommand( pInMsg->msg->endPoint, &pInMsg->msg->srcAddr, ZCL_CLUSTER_ID_GEN_IDENTIFY,
                         COMMAND_IDENTIFY_QUERY_RSP, TRUE,
                         ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0, pInMsg->hdr.transSeqNum, 2, (uint8 *) &identifyTime );
			return  ZCL_STATUS_CMD_HAS_RSP;
	    } else {
			return ZSuccess;
		}
	default:
    	return ZFailure;   // Error ignore the command
	}
}

ZStatus_t processIdentifyClusterClientCommands( zclIncoming_t *pInMsg ){
	if ( pInMsg->hdr.commandID != COMMAND_IDENTIFY_QUERY_RSP )
		return ZFailure;

	return ZFailure;
}

