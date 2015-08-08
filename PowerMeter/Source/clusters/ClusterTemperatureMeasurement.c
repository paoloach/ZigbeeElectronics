/**************************************************************************************************

 DESCRIPTION:
  Temperature Measurement Cluster

 CREATED: 12/11/2014, by Paolo Achdjian

 FILE: ClusterTemperatureMeasurement.c

RESOURCES:
 P1.2 Generaio I/O
 T3

***************************************************************************************************/
#include "ioCC2530.h"
#include "hal_mcu.h"
#include "ZDApp.h"
#include "hal_led.h"
#include "ClusterTemperatureMeasurement.h"
#include "OSAL_PwrMgr.h"


__sfr __no_init volatile union {
	struct {
		unsigned char T3_mode: 2;
		unsigned char T3_clear: 1;
		unsigned char T3_OVFIM: 1;
		unsigned char T3_start: 1;
		unsigned char T3_div: 3;
	};
} @ 0xCB;

__sfr __no_init volatile union {
	struct {
		unsigned char T3CCTL0_cap: 2;
		unsigned char T3CCTL0_mode: 1;
		unsigned char T3CCTL0_cmp: 3;
		unsigned char T3CCTL0_im: 1;
	};
} @ 0xCC;

__sfr __no_init volatile union {
	struct {
		unsigned char RFIE: 1;
		unsigned char P2IE: 1;
		unsigned char UTX0IE: 1;
		unsigned char UTX11E: 1;
		unsigned char P11E: 1;
		unsigned char WDTIE: 1;
	};
} @ 0x9A


#define WAIT_FOR_480us 	T3_start=0;		T3CC0=240;T3_clear=1; T3_start=1;
#define WAIT_FOR_2us T3_start=0;		T3CC0=8; T3_clear=1;T3_start=1;
#define WAIT_FOR_1us T3_start=0;		T3CC0=4; T3_clear=1;T3_start=1;
#define WAIT_FOR_12us T3_start=0;		T3CC0=48; T3_clear=1;T3_start=1;
#define WAIT_FOR_47us T3_start=0;	T3_clear=1;	T3CC0=188; T3_start=1;
#define WAIT_FOR_58us T3_start=0;	T3_clear=1;	T3CC0=232; T3_start=1;
#define WAIT_FOR_60us T3_start=0;	T3_clear=1;	T3CC0=240; T3_start=1;
#define WAIT_FOR_1ms T3_start=0;	T3_clear=1;	T3CC0=250; T3_start=1;
//#define WAIT_FOR_600us st(T3CTL &=0xE0;T3CTL |= 0x04;T3CC1=150;T3CTL &= 0xFB;)
#define WAIT_FOR_300us st(T3CTL &=0xE0; T3CTL |= 0x04;T3CC1=75;T3CTL &= 0xFB;)
#define WAIT_FOR_4us st(T3CTL &=0xE0; T3CTL |= 0x04;T3CC1=1;T3CTL &= 0xFB;)
#define WAIT_FOR_8us st(T3CTL &=0xE0; T3CTL |= 0x04;T3CC1=2;T3CTL &= 0xFB;)
#define WAIT_FOR_16us st(T3CTL &=0xE0; T3CTL |= 0x04;T3CC1=4;T3CTL &= 0xFB;)

#define ENABLE_P1EN st( IEN2 |= 0x10;)
#define DISABLE_P1EN st(IEN2 &= 0xEF;);
#define ENABLE_P1_2_INT  st(IEN2 |= 0x10; P1IEN |= 0x04;)
#define DISABLE_P1_2_INT  st(P1IEN &= 0xFB;)
#define P1_2_RISING_INT st(PICTL &= 0xFD;)
#define P1_2_FALLING_INT st(PICTL |= 0x02;)

#define STOP_T3 st(T3CNT &= 0xEF;);
#define START_T3 st(T3CNT |= 0x10;);
#define P1_LOW st(P1DIR  |= 0x04;);
#define P1_HIGH st(P1DIR &= 0xFB;);

#define ENABLE_T3_CH0_INT  T3CCTL0_im = 1;
#define DISABLE_T3_CH0_INT T3CCTL0_im=0;
#define RESET_TM3   st(T3CNT=0;)
#define RESET_T3CH0_INT st(T3CH0IF=0;)
#define RESET_P1_2_INT st(P1IF=0; P1IFG =0;);



int16 temperatureValue=0;
int16 tempTemperatureValue;
int16 decTemperatureValue;
int16 minTemperatureValue=-10;
int16 maxTemperatureValue=80;
uint16 toleranceTemperature=10;


static void write(unsigned char byte);
static uint8  read(void);
static void readSyncronus(void);
static uint8 reset(void);
static void finalizeReadTemp(void);


extern byte temperatureSensorTaskID;
extern devStates_t devState;

#define TIME_READ_ms 10*1000

void clusterTemperatureMeasurementeInit(void) {
	P1SEL &=0xFB;
	P1DIR &= 0xFB;
	P1_2 = 0;
	
//	T3CTL = 0x04 | 0xA0; //Clear counter. interrupt disable. Compare mode. 4us at cycle
//	T3CCTL0 = 0x4; // compare mode
//	T3CCTL1 = 0;
//	P0DIR=0xFF;
//	P0=0;
	readTemperature();
	osal_start_timerEx( temperatureSensorTaskID, READ_TEMP_EVT, TIME_READ_ms );
}

uint16 readTemperatureLoop(uint16 events) {
	if (events & READ_TEMP_EVT){
		readTemperature();
		return ( events ^ READ_TEMP_EVT );
	};
	if (events & END_READ_TEMP_EVT){
		finalizeReadTemp();
		return ( events ^ END_READ_TEMP_EVT );
	}
	return events;
}

void readTemperature(void) {
	osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_HOLD);
#if 1
	readSyncronus();
//	osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_CONSERVE);
#else
	readAsyncronus();
#endif
	osal_start_timerEx( temperatureSensorTaskID, READ_TEMP_EVT, TIME_READ_ms );
}

void readSyncronus(void) {

	P1SEL &=0xFB;
	P1DIR &= 0xFB;
	P1_2 = 0;
	
	T3CTL = 0x04 | 0xA0; //Clear counter. interrupt disable. Compare mode. 4us at cycle
	T3CCTL0 = 0x4; // compare mode
	T3CCTL1 = 0;
	P0DIR=0xFF;
		
	DISABLE_P1_2_INT;
	T3IF=0;
	T3CH0IF=0;
	st(T3IE=0;);
	if (reset()==0)
		return;
	
	write(0xCC);
	write(0x44);
	
	osal_start_timerEx( temperatureSensorTaskID, END_READ_TEMP_EVT, 750 );
	
	/*
	T3_div=7;
	for(uint16 i=0; i < 750; i++){
		T3_clear=1;
		while(T3CNT < 250);
		
	}
	finalizeReadTemp();
	*/
}

void finalizeReadTemp(void){
	uint8 low;
	uint8 heigh;
	reset();
	write(0xCC);
	write(0xBE);
	low = read();
	heigh = read();
	
	tempTemperatureValue = BUILD_UINT16(low,heigh);
	temperatureValue = (tempTemperatureValue >> 4)*100;
	decTemperatureValue = (tempTemperatureValue & 0x0F)*100;
	
	temperatureValue += decTemperatureValue >> 4;
	osal_pwrmgr_task_state(temperatureSensorTaskID, PWRMGR_CONSERVE);
}

uint8 reset() {
	P1_LOW;
	T3_div=6;
	T3_clear=1;
	T3_start=1;
	while(T3CNT < 244);
	P1_HIGH;
	T3_clear=1;
	while(T3CNT < 30);
	T3_clear=1;
	while(T3CNT < 240  && P1_2 == 1);
	
	if (P1_2 == 1){
		return 0;
	}
	while(P1_2==0);
	return 1;
}

void write(unsigned char byte){
	uint8 bit=8;
	T3_start = 0;
	T3_div=5;
	T3_start=1;
	T3_clear=1;
	while(T3CNT < 2);

	while(bit > 0){
		P1_LOW;
		T3_clear=1;
		if (byte & 0x1){
			while(T3CNT < 10);
		} else {
			while(T3CNT < 60);
		}
		
		P1_HIGH;
		while(T3CNT < 62);
		byte = byte >> 1;
		bit--;
	}
	
}

uint8  read(void) {
	uint8 bit=8;
	uint8 result=0;
	
	T3_start = 0;
	T3_div=5;
	T3_start=1;
	
	T3_clear=1;
	while(T3CNT < 2);
	while(bit > 0){
		P1_LOW;
		T3_clear=1;
		while(T3CNT < 2);
		P1_HIGH;
		T3_clear=1;
		while(T3CNT < 10);
		result >>= 1;
		if (P1_2){
			result |= 0x80;
		}
		T3_clear=1;
		while(T3CNT < 30);
		bit--;
	}
	return result;
}



