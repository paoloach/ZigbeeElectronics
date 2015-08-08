/**************************************************************************************************
  Filename:       ClusterPower.h

  Autorh:  Paolo Achdjian
  Created: 12/11/2014

**************************************************************************************************/

#ifndef __CLUSTER_POWER__H__
#define __CLUSTER_POWER__H__

#include "zcl_general.h"
#include "zcl.h"
#include "ClusterOSALEvents.h"

extern uint16 mainVoltage;
extern uint16 batteryVoltage;
extern uint8  batteryAlarmMask;

#define POWER_ATTRIBUTES  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_MAINS_VOLTAGE,  ZCL_DATATYPE_UINT16, ACCESS_CONTROL_READ, (void *)&mainVoltage, NULL  }  },  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_BATTERY_VOLTAGE,  ZCL_DATATYPE_UINT8, ACCESS_CONTROL_READ, (void *)&batteryVoltage, NULL  }  },  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_BAT_ALARM_MASK,  ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&batteryAlarmMask, NULL  }  },
	

#endif