/**************************************************************************************************
  Filename:       ClusterBasic.h

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#ifndef __CLUSTER_BASIC__H__
#define __CLUSTER_BASIC__H__

#include "zcl_general.h"
#include "zcl.h"

extern const uint8 HWRevision;
extern const uint8 ZCLVersion;
extern const uint8 manufacturerName[];
extern const uint8 modelId[];
extern const uint8 dateCode[];
extern const uint8 powerSource;
extern uint8 locationDescription[];
extern uint8 physicalEnvironment;
extern uint8 deviceEnable;


#define BASIC_ATTRIBUTE \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_HW_VERSION       , ZCL_DATATYPE_UINT8   , ACCESS_CONTROL_READ, (void *)&HWRevision     , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_ZCL_VERSION      , ZCL_DATATYPE_UINT8   , ACCESS_CONTROL_READ, (void *)&ZCLVersion     , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_MANUFACTURER_NAME, ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)manufacturerName, NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_MODEL_ID         , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)modelId         , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_DATE_CODE        , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)dateCode        , NULL     }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_POWER_SOURCE     , ZCL_DATATYPE_ENUM8,    ACCESS_CONTROL_READ, (void *)&powerSource    , NULL     }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_LOCATION_DESC    , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_R_W , (void *)locationDescription, NULL  } }, \
    { ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_PHYSICAL_ENV     , ZCL_DATATYPE_ENUM8   , ACCESS_CONTROL_R_W , (void *)&physicalEnvironment , NULL} }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_DEVICE_ENABLED   , ZCL_DATATYPE_BOOLEAN , ACCESS_CONTROL_R_W , (void *)&deviceEnable   , NULL     } },

void basicResetCB( void );
ZStatus_t processBasicClusterCommands( zclIncoming_t *pInMsg );

#endif