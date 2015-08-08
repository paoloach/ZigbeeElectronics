/**************************************************************************************************
  Filename:       ClusterOnOff.h

  Autorh:  Paolo Achdjia
  Created: 27/10/2014

**************************************************************************************************/

#ifndef __CLUSTER_ON_OFF__H_
#define __CLUSTER_ON_OFF__H_

#include "zcl.h"

#ifndef LIGHT_OFF
#define LIGHT_OFF                       0x00
#endif

#ifndef LIGHT_ON
#define LIGHT_ON                        0x01
#endif

extern uint8  onOffValue;

void setIOStatus(void);

#define ONOF_ATTRIBUTES   {ZCL_CLUSTER_ID_GEN_ON_OFF,{ATTRID_ON_OFF, ZCL_DATATYPE_BOOLEAN, ACCESS_CONTROL_READ,(void *)&onOffValue,&setIOStatus } },


void setLightStatus(uint8 status);

ZStatus_t processOnOffClusterServerCommands(zclIncoming_t *pInMsg);

#endif