/**************************************************************************************************
  Filename:       ClusterOnOff.c

  Autorh:  Paolo Achdjia
  Created: 27/10/2014

**************************************************************************************************/

#include "zcl_general.h"

#include "ClusterOnOff.h"
#include "onboard.h"

uint8  onOffValue = LIGHT_ON;

void setIOStatus(void){
	if ( onOffValue  == LIGHT_ON )
	  P1_4=1;
  	else
	  P1_4=0;
}

void setLightStatus(uint8 status){
	onOffValue = status;
	setIOStatus();
}



ZStatus_t processOnOffClusterServerCommands(zclIncoming_t *pInMsg) {
	switch(pInMsg->hdr.commandID){
	case COMMAND_ON:
		onOffValue = LIGHT_ON;
		break;
	case COMMAND_OFF:
		onOffValue = LIGHT_OFF;
		break;
	case COMMAND_TOGGLE:
		if ( onOffValue == LIGHT_OFF )
      		onOffValue = LIGHT_ON;
    	else
      		onOffValue = LIGHT_OFF;
		break;
	default:
		return ZFailure;
	}
	setIOStatus();
	return ZSuccess;
}
