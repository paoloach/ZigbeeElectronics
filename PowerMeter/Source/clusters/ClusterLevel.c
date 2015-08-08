/**************************************************************************************************
  Filename:       ClusterLevel.c

  Autorh:  Paolo Achdjia
  Created: 01/11/2014

**************************************************************************************************/

#include "ioCC2530.h"

#include "ClusterLevel.h"
#include "ClusterOSALEvents.h"

extern byte lightAchdjianTaskID;
uint16 currentLevel=1;

#define UP 0
#define DOWN 1
static uint8 up;
static uint8 step;
static uint16 newLevel;
static uint32 timeEnd;

// frequency 32M 
// prescalare 1/128 
//   timer frequency = 250k 
//   cycle length = 1ms
void clusterLevelInit(){
	P1DIR |= 0x08;
	P1SEL |= 0x10;
	P1_4 = 0;
	T3CC1 = currentLevel;
	T3CCTL1 = 0x24; // 00100100
	T3CTL = 0xF0;
}

void setCurrentLevel(uint8 level) {
	currentLevel = level;
	updateLevel();
}

void updateLevel(void) {
	if (currentLevel > 0xFF){
		currentLevel = 0xFF;
	}
	T3CC1 = (uint8)currentLevel;
}

static void enableTransiction(uint24 transitionTime) {
	uint8 diff;
	if (newLevel > currentLevel){
		diff = newLevel-currentLevel;
		up=UP;
	} else {
		diff = currentLevel - newLevel;
		up=DOWN;
	}
	timeEnd = transitionTime + osal_GetSystemClock();
	if (transitionTime > diff){
		uint24 timeStep = transitionTime / diff;
		step = 1;
		osal_start_timerEx( lightAchdjianTaskID, LEVEL_MOVEMENT, (uint16)timeStep );
	} else {
		step = diff /transitionTime;
		osal_start_timerEx( lightAchdjianTaskID, LEVEL_MOVEMENT, (uint16)1 );
	}
}

uint16 processLevelTimeChange(uint16 events) {
	uint32 currentTime;
	uint32 timeLeft;
	
	if (up==UP){
		currentLevel += step;
	} else {
		currentLevel -= step;
	}
	currentTime = osal_GetSystemClock();
	if (currentTime >= timeEnd){
		currentLevel = newLevel;
	} else {
		timeLeft = timeEnd - currentTime;
		enableTransiction(timeLeft);
	}
	updateLevel();
	return ( events ^ IDENTIFY_TIMEOUT_EVT );
}

ZStatus_t processLevelClusterServerCommands( zclIncoming_t *pInMsg ) {
	uint8 diff;
	uint16 transitionTime;
	
	switch ( pInMsg->hdr.commandID ){
	case COMMAND_LEVEL_MOVE_TO_LEVEL:
	case  COMMAND_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF:
		newLevel = pInMsg->pData[0];
		transitionTime = BUILD_UINT16( pInMsg->pData[1], pInMsg->pData[2]);
		enableTransiction(transitionTime*100);									   
        return ZSuccess;
	case COMMAND_LEVEL_MOVE:
	case COMMAND_LEVEL_MOVE_WITH_ON_OFF:
		up = pInMsg->pData[0];
		step = pInMsg->pData[1];
		if (up==UP){
			newLevel = 0xFF;
			diff = 0xFF-currentLevel;
		} else {
			newLevel = 0;
			diff = currentLevel;
		}
		if (step == 0xff){
			currentLevel = newLevel;
			updateLevel();
		} else {
			timeEnd =  osal_GetSystemClock() + diff/step+1000;
			osal_start_timerEx( lightAchdjianTaskID, LEVEL_MOVEMENT, 1000 );
		}
		return ZSuccess;		 
	case COMMAND_LEVEL_STEP:
	case COMMAND_LEVEL_STEP_WITH_ON_OFF:
		up = pInMsg->pData[0];
		step = pInMsg->pData[1];
		if (up==UP){
			newLevel = currentLevel + step;
			if (newLevel > 0xFF)
				newLevel = 0xFF;
		} else {
			if (step > currentLevel){
				newLevel =0;
			} else {
				newLevel = currentLevel - step;
			}
		}
		transitionTime = BUILD_UINT16( pInMsg->pData[1], pInMsg->pData[2]);
		enableTransiction(100*transitionTime);
		return ZSuccess;
	case COMMAND_LEVEL_STOP:
	case COMMAND_LEVEL_STOP_WITH_ON_OFF:			
		osal_stop_timerEx(lightAchdjianTaskID ,LEVEL_MOVEMENT);
		return ZSuccess;
	default:
		return ZFailure;
	}
}