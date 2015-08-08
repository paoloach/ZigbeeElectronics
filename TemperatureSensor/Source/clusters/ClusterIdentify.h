/**************************************************************************************************
  Filename:       ClusterIdentify.h

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#ifndef __CLUSTER_IDENTIFY__H_
#define __CLUSTER_IDENTIFY__H_

#include "zcl_general.h"
#include "zcl.h"
#include "ClusterOSALEvents.h"

extern uint16 identifyTime;

#define IDENTIFY_ATTRIBUTES  { ZCL_CLUSTER_ID_GEN_IDENTIFY, {  ATTRID_IDENTIFY_TIME,  ZCL_DATATYPE_UINT16, ACCESS_CONTROL_R_W, (void *)&identifyTime, &processIdentifyTimeChange  }  },

void identifyInit(void);
void processIdentifyTimeChange( void );
uint16 identifyLoop(uint16 events);

ZStatus_t processIdentifyClusterServerCommands( zclIncoming_t *pInMsg );
ZStatus_t processIdentifyClusterClientCommands( zclIncoming_t *pInMsg );

#endif