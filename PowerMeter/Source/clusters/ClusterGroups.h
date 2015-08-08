/**************************************************************************************************
  Filename:       ClusterGroups.h

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#ifndef __CLUSTER_GROUPS__H__
#define __CLUSTER_GROUPS__H__

#include "zcl_general.h"
#include "zcl.h"

extern uint8 groupNameSupported;

#define GROUPS_ATTRIBUTES  {ZCL_CLUSTER_ID_GEN_GROUPS, {ATTRID_GROUP_NAME_SUPPORT, ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&groupNameSupported, NULL }  },

ZStatus_t processGroupsClusterServerCommands(zclIncoming_t *pInMsg);

#endif