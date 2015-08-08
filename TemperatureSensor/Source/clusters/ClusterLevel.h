/**************************************************************************************************
  Filename:       ClusterLevel.h

  Autorh:  Paolo Achdjia
  Created: 01/11/2014

**************************************************************************************************/

#ifndef __CLUSTER_LEVEL__H__
#define __CLUSTER_LEVEL__H__

#include "zcl_general.h"
#include "zcl.h"

extern uint16 currentLevel;


#define LEVEL_ATTRIBUTES   \
	{ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,  {ATTRID_LEVEL_CURRENT_LEVEL, ZCL_DATATYPE_UINT8  , ACCESS_CONTROL_R_W, (void *)&currentLevel  , &updateLevel}}, 


void clusterLevelInit(void);
void setCurrentLevel(uint8 level);
void updateLevel(void);

uint16 processLevelTimeChange(uint16 events);

ZStatus_t processLevelClusterServerCommands( zclIncoming_t *pInMsg );

#endif