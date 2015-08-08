/**************************************************************************************************
  Filename:       ClusterScenes.h

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#ifndef __CLUSTER_SCENES__H__
#define __CLUSTER_SCENES__H__

#include "zcl_general.h"
#include "zcl.h"

extern uint8 sceneCount;
extern uint8 currentScene;
extern uint16 currentGroup;
extern uint8 sceneValid;
extern uint8 nameSupported;


#define SCENES_ATTRIBUTES   \
	{ZCL_CLUSTER_ID_GEN_SCENES,  {ATTRID_SCENES_COUNT        , ZCL_DATATYPE_UINT8  , ACCESS_CONTROL_READ, (void *)&sceneCount  , NULL}}, \
		{ZCL_CLUSTER_ID_GEN_SCENES,  {ATTRID_SCENES_CURRENT_SCENE, ZCL_DATATYPE_UINT8  , ACCESS_CONTROL_READ, (void *)&currentScene, NULL}  }, \
		{ ZCL_CLUSTER_ID_GEN_SCENES, {ATTRID_SCENES_CURRENT_GROUP, ZCL_DATATYPE_UINT16 , ACCESS_CONTROL_READ, (void *)&currentGroup, NULL}  }, \
		{ ZCL_CLUSTER_ID_GEN_SCENES, {ATTRID_SCENES_SCENE_VALID  , ZCL_DATATYPE_BOOLEAN, ACCESS_CONTROL_READ, (void *)&sceneValid, NULL}    }, \
		{ ZCL_CLUSTER_ID_GEN_SCENES, {ATTRID_SCENES_NAME_SUPPORT , ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&nameSupported, NULL}  },



ZStatus_t processSceneClusterServerCommands(zclIncoming_t *pInMsg);
uint8 scenesInitNV( void );
void scenesRestoreFromNV( void );

#endif