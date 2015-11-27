
/**************************************************************************************************

 DESCRIPTION:
  Metering Cluster

 CREATED: 08/09/2015, by Paolo Achdjian

 FILE: ClusterMetering.c

***************************************************************************************************/

#include "ClusterMetering.h"


uint48 currentSummationDelivered=0;
uint8  status=0;
uint8  unitOfMeasure=0; // kW (kilo-Watts) & kWh (kilo-WattHours) in pure Binary format
uint8  summationFormatting=0;
uint8  metteringDeviceType=0; //Electric Metering