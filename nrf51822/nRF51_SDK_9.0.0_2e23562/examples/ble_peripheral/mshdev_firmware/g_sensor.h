//*Mikhail Sharonov, aka obelix662000@yahoo.com, Copyright (c) 2016. All Rights Reserved.
//*
//*
#ifndef G_SENSOR_H
#define G_SENSOR_H
#include "LIS2DH.h"

typedef enum       //
{
    INT_IDLE,
	  INT_AOI,
	  INT_WMK
} int_state_t;


typedef enum
{
    G_SENSORS_EVENT_NOTHING = 0,                  /**< Assign this event to an action to prevent the action from generating an event (disable the action). */
    G_SENSORS_EVENT_FIFO_READY,
	  G_SENSORS_EVENT_AT_REST,
	  G_AOI_EVENT,
} gsensor_event_t;

typedef void (* gsensor_event_callback_t)(gsensor_event_t); 
void ACC_callback_init(gsensor_event_callback_t callback);

// mode: false: output aX, aY, aZ (DC); true: output filtered HP (AC)
// range (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
// restTime, ms time after sensor considered to be at rest (record ready)

void ACC_init_Sensor(uint32_t restTime);
// aSpeed:0 - off, 01:1Hz(2uA), 02:10Hz(4uA), 03:25Hz(6uA), 04:50Hz(11uA), 05:100Hz(20uA), 06:200Hz(38uA), 07:400Hz(73uA)
void ACC_setAOIFreq(uint8_t aSpeed);
//true- filtered output, DC - as is
void ACC_setACmode(bool mode);
bool ACC_getACMode(void);
// range (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
void ACC_setRange(uint8_t range);
// sTrigger: trigger sensitivity: 1LSb = 16mg @FS=2g, 1LSb = 32 mg @FS=4g, 1LSb = 62 mg @FS=8g, 1LSb = 186 mg @FS=16g
void ACC_setAOITrig(uint8_t trig);
void ACC_setFIFODepth(uint8_t depth);
void ACC_setFIFOFreq(uint8_t rate);
void ACC_setSensorState(int_state_t state);
uint8_t  ACC_getAccData(int16_t* ax, int16_t* ay, int16_t* az);
void ACC_setAOI_int(void);
void ACC_setLiveMode(bool mode);
void ACC_setDefault(void); //deafult, power off
void ACC_start(void);
void ACC_stop(void);
uint8_t  ACC_getStatusReg(void);
void ACC_setTrigMode(uint8_t mode);
void ACC_setRestTimerIntv(uint32_t time);
#endif // G_SENSOR_H
