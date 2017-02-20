//*Mikhail Sharonov
//*
//*
#include "g_sensor.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"



static int_state_t int_state=INT_IDLE;
static uint8_t aoiFreq=5; //100Hz
static uint8_t fifoFreq=5; //100Hz
static uint8_t fifoDepth=31; //max 31
static uint8_t aoiTrig=4; //
static uint8_t accRange=0; //2g
static uint8_t trigMode=0x2a; //OR, all axis high
//b10xxxxxx  AND
//b00xxxxxx  OR
//bxx1xxxxx Z higher than threshold
//bxxx1xxxx Z lower than threshold
//bxxxx1xxx, bxxxxx1xx y high, low
//bxxxxxx1x, bxxxxxxx1 x high, low

static bool acMode=true; //HF filter on (true)


static gsensor_event_callback_t   m_registered_callback         = NULL;

void ACC_callback_init(gsensor_event_callback_t callback);

void ACC_init_Sensor(uint32_t restTime);
void ACC_setAOIFreq(uint8_t aSpeed);
void ACC_setACmode(bool mode);
void ACC_setRange(uint8_t range);
void ACC_setAOITrig(uint8_t trig);
void ACC_setFIFODepth(uint8_t depth);
void ACC_setFIFOFreq(uint8_t rate);
void ACC_setTrigMode(uint8_t mode);



static app_timer_id_t   rest_timer_id;
static uint32_t restTimerIntv;
static bool isAOIeventAllowed=true;  //do not let G_AOI_EVENT until rest timer

static void rest_timer_handler(void * p_context)
{
m_registered_callback(G_SENSORS_EVENT_AT_REST); //restTimerIntv passed since AOI event
app_timer_stop(rest_timer_id);
isAOIeventAllowed=true; 
}



// aSpeed:0 - off, 01:1Hz(2uA), 02:10Hz(4uA), 03:25Hz(6uA), 04:50Hz(11uA), 05:100Hz(20uA), 06:200Hz(38uA), 07:400Hz(73uA)
void ACC_setAOIFreq(uint8_t aSpeed){
//LIS2DH_writeRegister(LIS2DH_CTRL_REG1,(aSpeed<<4)|7); //normal mode (10 bit)
aoiFreq=aSpeed;	
};


void ACC_stop(void){
LIS2DH_writeRegister(LIS2DH_CTRL_REG1,0x00); //off axis disabled
};


void ACC_start(void){
LIS2DH_writeRegister(LIS2DH_CTRL_REG1,(aoiFreq<<4)|7);
};



void ACC_setACmode(bool mode){

if (mode)	
LIS2DH_writeRegister(LIS2DH_CTRL_REG2,0x09);//0xc0 autoreset filter //0x08-FIFO and out HP, 0x01 hpis1 trig. source - high pass filter
else
LIS2DH_writeRegister(LIS2DH_CTRL_REG2,0x00);
acMode=mode;

	
};


bool ACC_getACMode(void){
return acMode;
}	

// range (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
void ACC_setRange(uint8_t range){
LIS2DH_writeRegister(LIS2DH_CTRL_REG4,(range<<4)|0x80); //bdu on
accRange=range;
};	

// sTrigger: trigger sensitivity: 1LSb = 16mg @FS=2g, 1LSb = 32 mg @FS=4g, 1LSb = 62 mg @FS=8g, 1LSb = 186 mg @FS=16g
void ACC_setAOITrig(uint8_t trig){
LIS2DH_writeRegister(LIS2DH_INT1_THS,trig);
aoiTrig=trig;
};	



void ACC_setFIFODepth(uint8_t depth){
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x40|depth);	
fifoDepth=depth;	
};

void ACC_setFIFOFreq(uint8_t rate){
//LIS2DH_writeRegister(LIS2DH_CTRL_REG1,(rate<<4)|7);//fill FIFO at fifo_Rate
fifoFreq=rate;	
};




void LIS2DH_INT1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
if (!nrf_drv_gpiote_in_is_set(pin)) return; //we need lo-to-hi only, skip hi-to-lo
	
if (int_state==INT_AOI){      //AOI event
app_timer_stop(rest_timer_id);
app_timer_start(rest_timer_id,restTimerIntv,NULL);	//restart rest-timer on every AOI event

if (isAOIeventAllowed){	
m_registered_callback(G_AOI_EVENT);	//send AOI event
isAOIeventAllowed=false;
}
else  //FIFO
{	
LIS2DH_writeRegister(LIS2DH_CTRL_REG3,0x00); //no interrupts
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
LIS2DH_writeRegister(LIS2DH_CTRL_REG3,0x04); //WTM, INT1
LIS2DH_writeRegister(LIS2DH_CTRL_REG1,(fifoFreq<<4)|7);//fill FIFO at fifo_Rate
LIS2DH_writeRegister(LIS2DH_CTRL_REG5,0x40);//fifo enabled, 
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x40|fifoDepth);////FIFO trigger mode stops at FIFO full, wtm=10			
int_state=INT_WMK; //wait for watermark interrupt 	
}
return;	
};

if (int_state==INT_WMK){      //watermark event
ACC_stop();	
int_state=INT_IDLE;
while (LIS2DH_readRegister(LIS2DH_FIFO_SRC_REG)&0x80) {  // read fifo until WTM cleared
LIS2DH_getAxisX();    LIS2DH_getAxisY();    LIS2DH_getAxisZ();};	
m_registered_callback(G_SENSORS_EVENT_FIFO_READY);	//FIFO ready, passed to main thread
return;	
}	
};

void ACC_setSensorState(int_state_t state){
app_timer_stop(rest_timer_id);	
int_state=state;
};


void ACC_setTrigMode(uint8_t mode){
LIS2DH_writeRegister(LIS2DH_INT1_CFG,mode);  //OR (all axis) high/low threshold
trigMode=mode;	
}	

void ACC_callback_init(gsensor_event_callback_t callback){
m_registered_callback = callback;

}	

void ACC_setDefault(void){
LIS2DH_init(); 
}


void LIS2DH_INT1_init(nrf_drv_gpiote_pin_t pinIn)
{
    ret_code_t err_code;
	//DO NOT USE GPIOTE_CONFIG_IN_SENSE_LOTOHI(true), bug in this NRF51822 revision, individual pin inteerupts (true) uses HF and 1mA current!
    nrf_drv_gpiote_in_config_t in_config =  GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);//GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);//GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);//
    in_config.pull = NRF_GPIO_PIN_NOPULL;//NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(pinIn, &in_config, LIS2DH_INT1_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(pinIn, true);
	
}	

void ACC_setRestTimerIntv(uint32_t time){
app_timer_stop(rest_timer_id);
restTimerIntv=time;
}	

// aSpeed:0 - off, 01:1Hz(2uA), 02:10Hz(4uA), 03:25Hz(6uA), 04:50Hz(11uA), 05:100Hz(20uA), 06:200Hz(38uA), 07:400Hz(73uA)
// mode: fasle: output aX, aY, aZ (DC); true: output filtered HP (AC)
// range (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
// sTrigger: trigger sensitivity:
// 1LSb = 16mg @FS=2g
// 1LSb = 32 mg @FS=4g
// 1LSb = 62 mg @FS=8g
// 1LSb = 186 mg @FS=16g
// restTime, ms time after sensor considered to be at rest (record ready)
void ACC_init_Sensor(uint32_t restTime){
LIS2DH_INT1_init(0);
LIS2DH_init();
ACC_setACmode(acMode);	
ACC_setRange(accRange);
//ACC_setAOIFreq(aoiFreq); //100Hz //0- off
ACC_setAOITrig(aoiTrig); //4*16mg
ACC_setFIFODepth(fifoDepth); //max
//ACC_setFIFOFreq(fifoFreq); //fuill FIFO at 100 Hz
ACC_setTrigMode(trigMode); //default 0x2a	
LIS2DH_writeRegister(LIS2DH_CTRL_REG5,0x40);//0x00); //fifo disabled
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00); //bypass fifo

LIS2DH_writeRegister(LIS2DH_INT1_DURATION,0x00);//0x2f); //5x 1/10Hz
LIS2DH_writeRegister(LIS2DH_CTRL_REG3,0x40);//int1 aoi


app_timer_create(&rest_timer_id, APP_TIMER_MODE_SINGLE_SHOT, rest_timer_handler);
ACC_setRestTimerIntv(restTime);	

};	


	


uint8_t  ACC_getAccData(int16_t* ax, int16_t* ay, int16_t* az){//type_AccData* accData){
    *ax=LIS2DH_getAxisX()>>6;
    *ay=LIS2DH_getAxisY()>>6;
    *az=LIS2DH_getAxisZ()>>6;	
return LIS2DH_readRegister(LIS2DH_FIFO_SRC_REG);	
}	


void ACC_setAOI_int(){
LIS2DH_writeRegister(LIS2DH_CTRL_REG3,0x40); // AOI level int source
LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);
ACC_start();
int_state=INT_AOI;
}	


void ACC_setLiveMode(bool mode){
isAOIeventAllowed=true;
}	



uint8_t  ACC_getStatusReg(){
return LIS2DH_readRegister(LIS2DH_STATUS_REG2);	
}
