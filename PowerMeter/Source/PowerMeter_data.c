/**************************************************************************************************
  Filename:       Light_achdjian_data.c
  Autorh: 		 Paolo Achdjia
  Created: 30/10/2014
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "PowerMeter.h"
#include "clusters/ClusterIdentify.h"
#include "clusters/ClusterBasic.h"
#include "clusters/ClusterPower.h"
#include "clusters/ClusterTemperatureMeasurement.h"

/*********************************************************************
 * CONSTANTS
 */

#define DEVICE_VERSION     0
#define FLAGS              0


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */



/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t lightAchdjianAttrs[] = {
	BASIC_ATTRIBUTE
	IDENTIFY_ATTRIBUTES
	POWER_ATTRIBUTES
	TEMPERATURE_MEASUREMENT_ATTRIBUTES
 	LAST_CLUSTER_ATTRIBUTE
};

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLSAMPLELIGHT_MAX_INCLUSTERS       4
const cId_t zclSampleLight_InClusterList[ZCLSAMPLELIGHT_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_POWER_CFG,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};

#define ZCLSAMPLELIGHT_MAX_OUTCLUSTERS       1
const cId_t zclSampleLight_OutClusterList[ZCLSAMPLELIGHT_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC
};

SimpleDescriptionFormat_t zclSampleLight_SimpleDesc =
{
  ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,        //  uint16 AppDeviceId[2];
  DEVICE_VERSION,            //  int   AppDevVer:4;
  FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/


