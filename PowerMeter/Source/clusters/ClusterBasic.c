/**************************************************************************************************
  Filename:       ClusterBasic.cpp

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#include "ClusterBasic.h"


const uint8 HWRevision = 1;
const uint8 ZCLVersion = 1;
const uint8 manufacturerName[] = { 20, 'A','c','h','d','j','i','a','n',' ','T','e','s','t',' ',' ',' ',' ',' ',' ',' ' };
const uint8 modelId[] = { 16, 'P','o','w','e','r','M','e','t','e','r',' ',' ',' ',' ',' ',' ' };
const uint8 dateCode[] = { 16, '2','0','1','5','0','8','0','9',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 powerSource = POWER_SOURCE_DC;

uint8 locationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 physicalEnvironment = 0;
uint8 deviceEnable = DEVICE_ENABLED;


/*********************************************************************
 * @fn      zclSampleLight_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
void basicResetCB( void ){
  // Reset all attributes to default values
}

ZStatus_t processBasicClusterCommands( zclIncoming_t *pInMsg ){
	if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) ) {
		switch(pInMsg->hdr.commandID){
			case COMMAND_BASIC_RESET_FACT_DEFAULT:
				basicResetCB();
				return ZSuccess;
		default:
      		return ZFailure;   // Error ignore the command
		}
	}
	return ZSuccess;
}