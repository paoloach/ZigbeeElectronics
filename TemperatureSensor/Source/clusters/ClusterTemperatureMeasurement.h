/**************************************************************************************************

 DESCRIPTION:
  Temperature Measuremente Cluster

 CREATED: 12/11/2014, by Paolo Achdjian

 FILE: ClusterTemperatureMeasurement.h

***************************************************************************************************/

#ifndef __CLUSTER_TEMPERATURE_MEASUREMENT__H__
#define __CLUSTER_TEMPERATURE_MEASUREMENT__H__

#include "zcl_general.h"
#include "zcl.h"
#include "ClusterOSALEvents.h"

extern int16 temperatureValue;
extern int16 minTemperatureValue;
extern int16 maxTemperatureValue;
extern uint16 toleranceTemperature;

#define ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT            0x0402

#define ATTRID_TEMPERATURE_MEASURE_VALUE		0
#define ATTRID_TEMPERATURE_MIN_MEASURE_VALUE	1
#define ATTRID_TEMPERATURE_MAX_MEASURE_VALUE	2
#define ATTRID_TEMPERATURE_TOLERANCE			3


#define TEMPERATURE_MEASUREMENT_ATTRIBUTES  \
	{ ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, {  ATTRID_TEMPERATURE_MEASURE_VALUE,      ZCL_DATATYPE_INT16 , ACCESS_CONTROL_READ, (void *)&temperatureValue, NULL  }  },  \
	{ ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, {  ATTRID_TEMPERATURE_MIN_MEASURE_VALUE,  ZCL_DATATYPE_INT16 , ACCESS_CONTROL_READ, (void *)&minTemperatureValue, NULL  }  },  \
	{ ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, {  ATTRID_TEMPERATURE_MAX_MEASURE_VALUE,  ZCL_DATATYPE_INT16 , ACCESS_CONTROL_READ, (void *)&maxTemperatureValue, NULL  }  }, \
	{ ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, {  ATTRID_TEMPERATURE_TOLERANCE,          ZCL_DATATYPE_UINT16, ACCESS_CONTROL_READ, (void *)&toleranceTemperature, NULL  }  },

uint16 readTemperatureLoop(uint16 events);
void readTemperature(void);
void clusterTemperatureMeasurementeInit(void);


#endif