//*Mikhail Sharonov, aka obelix662000@yahoo.com, Copyright (c) 2016. All Rights Reserved.
//*
//*
/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "MISFIT.h"
#include "b_and_l.h"
#include "nrf_delay.h"
//DFU
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "app_util.h"  //rounded div and math
#define IS_SRVC_CHANGED_CHARACT_PRESENT 1
//DFU

#include <math.h>


#define DEVICE_NAME                     "mshdev"                                   /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */


#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1000, UNIT_0_625_MS) //64 //64 //MSEC_TO_UNITS(1000, UNIT_0_625_MS) //64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0 // forever                                      /** 0x3fff(max), 180 in limited discoverable mode, in seconds */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            9             															/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         8  	                            		        /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


//DFU


#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16 


static ble_dfu_t                         m_dfus;
static dm_application_instance_t         m_app_handle;
//DFU




#include "pstorage.h"
		
		
#include "nrf_drv_gpiote.h"
#include "flash_manage.h"	
		
//#define DEBUG_UART  //test via UART

#ifdef DEBUG_UART

//UART test
#include "app_uart.h"		
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#endif


								

#include "g_sensor.h"



static void reset_prepare(void);


static void power_manage(void);		
static uint8_t battery_level_get(void);

static void sleep_mode_enter(void);
static void advertising_init(void);

static void showLedRing(void);
static void liveTrigStart(void);
static void recordStart(void);
static void prgStop(void);
static void set_adv_packet(bool isAlarm);


static app_timer_id_t                    keepAlive_timer;
static uint32_t rtcKeepAliveIntv =  APP_TIMER_TICKS(500000, APP_TIMER_PRESCALER); //this timer is needed to have RTC1 always running, overfill is counted via ticks interrupts


static uint32_t liveTimerIntv =  APP_TIMER_TICKS(200, APP_TIMER_PRESCALER);
static app_timer_id_t                    live_timer;                 	
#define ADV_UPDATE_INTV  3             //adv comes every 1 sec, update comes every 3 sec  
static uint32_t advUpdateTimerIntv =  APP_TIMER_TICKS(ADV_UPDATE_INTV*1000, APP_TIMER_PRESCALER); //info via 3s upadtes
static app_timer_id_t                    advUpdate_timer; //update via adv timer

#define ADV_ALARM_OFF  3000             //how long to include offline alarm info to adv packets, 3 packets  
static uint32_t advAlarmOffTimerIntv =  APP_TIMER_TICKS(ADV_ALARM_OFF, APP_TIMER_PRESCALER); //info via 3s upadtes
static app_timer_id_t                    advAlarmOff_timer; 

static bool live_Info(bool isAlarm);

static bool isLiveTimerStarted=false;
static bool isOffAlarm=false;  //
static bool isOffAlarmSet=false; //comes with settings
static void setRealTime(uint32_t t);

static bool isOffAlarmAdvRequested=false;

static uint8_t countAdvLedOFF=5; //how many advUpdate_timer events until adv LED will be switched off (show only at startup for 3x5=15)

typedef struct {
  uint32_t realTimeRef; //sync realtime UNIX format
	uint32_t rtcCountRef; //RTC count at the momemt of synching
} type_time;


static type_time time;

typedef enum       //main thread state machine
{
    STATE_IDLE,
	  STATE_READ_FIFO,
	  STATE_READ_ORIENTATION,
	  STATE_ALARM,
	  STATE_CONNECT,
	  STATE_ADVSTOP,
	  STATE_READ_RECORD, 
} prg_state_t;

typedef enum       //modes of operation
{
	  MODE_STOP,
    MODE_LIVE,
	  MODE_LIVE_TRIGGER,
	  MODE_OFFLINE_ALARM,
	  MODE_RECORD,
	  
} mode_state_t;


static uint8_t dataArray[12]; //maximal size: DATA_RECORD_SIZE+4(fake)

static prg_state_t prg_state=STATE_IDLE;
static mode_state_t mode_state=MODE_STOP;



#define DATA_RECORD_SIZE      8 //uint32_t rtc + 10bit max acceler + 10 bit pitch + 10bit roll (-512-+512)
#define MAX_FLASH_REC_NUM    ((FLASH_PAGE_SIZE+END_DATA_ADDR-START_DATA_ADDR-DATA_RECORD_SIZE)/ DATA_RECORD_SIZE) // maximum record number in flash


static uint32_t currentRecordAddress;
static uint32_t readRecordAddress;
static uint32_t numberOfRecords;
static uint32_t currentRecordNum;
static uint16_t recNum=0; //number of records since start of recording, for adv info

static void recordDataToFlash(void);
static int32_t searchStartRecord(void);



typedef struct {
  int16_t Roll; 
  int16_t Pitch;
} type_Angles;

static type_Angles ang;

uint16_t absAcceleration(const int16_t ax,const int16_t ay,const int16_t az){
return	round(sqrt(ax*ax+ay*ay+az*az));
}


static type_Angles calculateAngles(const int16_t ax,const int16_t ay,const int16_t az){
type_Angles ang;
double const pi=3.141593;
	
ang.Roll=round((atan2(-ay,az)*180.0)/pi);
ang.Pitch=round((atan2(ax,sqrt(ay*ay+az*az))*180.0)/pi);	
	
	
return	ang;
}
// aSpeed:0 - off, 01:1Hz(2uA), 02:10Hz(4uA), 03:25Hz(6uA), 04:50Hz(11uA), 05:100Hz(20uA), 06:200Hz(38uA), 07:400Hz(73uA)
 
static void liveStart(void);
static void liveTrigStart(void);
static void recordStart(void);
static void prgStop(void);

bool isDataTransfer=false;

//#define VBAT_MAX_IN_MV                  3300




/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{

    bsp_indication_set(BSP_INDICATE_RCV_OK);

		if (strncmp((char*)p_data, "set0 ", 5) == 0) { 
			live_Info(false); //report connection fast
#ifdef DEBUG_UART				
			printf("set0 %s\r\n",p_data);
#endif
      char* token;
			uint32_t val=0;
      uint8_t i=0;	
      app_timer_stop(live_timer); //stop timer while new params are set
			token=strtok((char*)&p_data[5],",");
			while(token != NULL)
				{
				val=atol(token);
				switch (i){
					
					case 0: //live stream interval
#ifdef DEBUG_UART				
			printf("set liveTimerIntv= %d\r\n",val);
#endif						
					  liveTimerIntv =  APP_TIMER_TICKS(val, APP_TIMER_PRESCALER);
					break;
					case 1:  //AC filter false (0) true (1)
#ifdef DEBUG_UART				
			printf("set offAlarm= %d\r\n",(uint8_t)val);
#endif						
					isOffAlarmSet=(uint8_t)val; //1,0 true or false
					break;
					case 2:  //range  (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
#ifdef DEBUG_UART				
			printf("set Range= %d\r\n",(uint8_t)val);
#endif						
  					ACC_setRange((uint8_t)val);
					break;
					
					case 3:  //watch freqency 2,4,5,6 (10,50,100,200) Hz
#ifdef DEBUG_UART				
			printf("set aoiFreq= %d\r\n",(uint8_t)val);
#endif						
  					ACC_setAOIFreq((uint8_t)val);
					break;
					case 4: //fifo freqency 2,4,5,6 (10,50,100,200) Hz
#ifdef DEBUG_UART				
			printf("set fifoFreq= %d\r\n",(uint8_t)val);
#endif						
  					ACC_setFIFOFreq((uint8_t)val);
					break;

				};
       i++;				
			
				token=strtok(NULL,",");
				};
				
			return;
			};

	
   		if (strncmp((char*)p_data, "set1 ", 5) == 0) { //start measuring and send data

			
#ifdef DEBUG_UART				
			printf("set1 %s\r\n",p_data);
#endif
      char* token;
			uint32_t val=0;
      uint8_t i=0;				
			token=strtok((char*)&p_data[5],",");
			while(token != NULL)
				{
				val=atoi(token);
				switch (i){
					case 0: //trigger threshold
#ifdef DEBUG_UART				
			printf("set aoiTrig= %d\r\n",(uint8_t)val);
#endif						
          ACC_setAOITrig((uint8_t)val);
					break;
					case 1:  //trigger restTimerIntv
#ifdef DEBUG_UART				
			printf("set restTimerIntv= %d\r\n",val);
#endif						
					ACC_setRestTimerIntv(APP_TIMER_TICKS(val*1000, APP_TIMER_PRESCALER));
					break;
					case 2:  //trigger mode, OR/AND axes
#ifdef DEBUG_UART				
			printf("set trigMode= %d\r\n",(uint8_t)val);
#endif						
  					ACC_setTrigMode((uint8_t)val);
					break;
				};
       i++;				
					
				token=strtok(NULL,",");
				}; 
			 if (isLiveTimerStarted) //start timer if it was started
       app_timer_start(live_timer,liveTimerIntv, NULL); //start reading				
				return;	
		  };	
	
	
   		if (strncmp((char*)p_data, "live ", 5) == 0) { //start measuring and send data
#ifdef DEBUG_UART				
			printf("live start\n\r");
#endif
      liveStart();		
				return;	
		  };
   		if (strncmp((char*)p_data, "ltrig ", 6) == 0) { //start live trigger
#ifdef DEBUG_UART				
			printf("live trig. start\n\r");
#endif
      liveTrigStart();		
				return;	
		  };
   		if (strncmp((char*)p_data, "otrig ", 6) == 0) { //start offline alarm
#ifdef DEBUG_UART				
			printf("offline alarm. start\n\r");
#endif
      recordStart();
      isOffAlarm=true;
      mode_state=MODE_OFFLINE_ALARM;
			live_Info(false);	
			prg_state=STATE_ADVSTOP;
			return;	
		  };			
			
   		if (strncmp((char*)p_data, "rec ", 4) == 0) { //record to flash 
	    uint32_t t_stamp;
			t_stamp=atol((char *) &p_data[4]);	
#ifdef DEBUG_UART				
			printf("time got from android= %d\n\r",t_stamp);
#endif
			setRealTime(t_stamp);				
      recordStart();
			live_Info(false);	
	    *(uint32_t*)&dataArray[0]=0x00000000; //very first record, indicates beginning of the session 										
	    recordDataToFlash();				
			prg_state=STATE_ADVSTOP;	
			return;	
		  };
			

   		if (strncmp((char*)p_data, "read_rec ", 9) == 0) { //read flash records 
	           int32_t startAddr;
							 
						 startAddr=searchStartRecord();	 
#ifdef DEBUG_UART				
			        printf("read data from flash, start addr 0x%x\n\r",startAddr);
#endif
             
             readRecordAddress=startAddr+DATA_RECORD_SIZE;				
                     if (currentRecordAddress-readRecordAddress)
						            numberOfRecords=(currentRecordAddress-readRecordAddress)/DATA_RECORD_SIZE;
								     else
										    numberOfRecords=((VAR_ADDR-readRecordAddress)+(currentRecordAddress-START_DATA_ADDR))/DATA_RECORD_SIZE;             
#ifdef DEBUG_UART				
			        printf("num records= %d\n\r",numberOfRecords);
#endif       
						 currentRecordNum=numberOfRecords;				 
						 prg_state=STATE_READ_RECORD;
						  
						 return;
						 };					
			
   		if (strncmp((char*)p_data, "stop ", 5) == 0) { //stop 
#ifdef DEBUG_UART				
      printf("stop\n\r");
#endif
      prgStop();
			return;	
		  };			
   		if (strncmp((char*)p_data, "disc ", 5) == 0) { //disconnect
#ifdef DEBUG_UART					 
			printf("prg disconnect %d\r\n", mode_state);
#endif
			if ((mode_state==MODE_LIVE)||(mode_state==MODE_LIVE_TRIGGER)){
#ifdef DEBUG_UART					 
			printf("online mode, stop %d \r\n", mode_state);
#endif
				prgStop(); //stop if not waiting for offline alarms 
			};				
      prg_state=STATE_ADVSTOP;

				return;	
		  };	
   		if (strncmp((char*)p_data, "off ", 4) == 0) { //sleep until wake up with button
#ifdef DEBUG_UART					 
			printf("device off\r\n");
#endif
				
			prgStop();
 			reset_prepare(); //disconnect or (if advertising started, adv stop)
      sleep_mode_enter(); //sleep with no return, wake up by long button push				

				return;	
		  };				
}

static uint8_t battery_level_get(void)
{
    // Configure ADC
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
    
    while (!NRF_ADC->EVENTS_END)
    {
    }
    //2032 battery: 100% 3.0 V, 50% 2.75V 25% 2.625V 0% <2.5V
    uint16_t vbg_in_mv = 1200; //reference, prescaling=3
    uint8_t adc_max = 255;
    uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max; //in volts
    
    NRF_ADC->EVENTS_END     = 0;
    NRF_ADC->TASKS_STOP     = 1;
		
		if (vbat_current_in_mv>2750) return 100; //100%-50%
		if (vbat_current_in_mv>2625) return 50; //50%-25%
		if (vbat_current_in_mv>2500) return 25; //25%-0% 
		if (vbat_current_in_mv<=2500) return 0;
		return 100; //not reached
    //return (uint8_t) ((vbat_current_in_mv * 100) / VBAT_MAX_IN_MV); //%
}



/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by 
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */


static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        bsp_indication_set(BSP_INDICATE_IDLE);
		};	
    err_code = ble_conn_params_stop();
		if (err_code!=NRF_SUCCESS) bsp_indication_set(BSP_INDICATE_ERROR);
    APP_ERROR_CHECK(err_code);
	
}

/** @snippet [DFU BLE Reset prepare] */


static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();  //dfu required
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


//DFUe

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
	
// DFU 
	  ble_dfu_init_t   dfus_init;
    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);		
	//DFU	
	
}






/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
 uint32_t err_code; 
	  
	  bsp_indication_set(BSP_INDICATE_IDLE);
	  showLedRing();
    ACC_stop();
 	  err_code = bsp_btn_ble_sleep_mode_prepare();  //prepare wakeup buttons, hardware interrupt on in button0
    APP_ERROR_CHECK(err_code);
	  sd_power_system_off();

}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */


static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{

    switch (ble_adv_evt)
    {
			
        case BLE_ADV_EVT_FAST:
				    isDataTransfer=false;
#ifdef DEBUG_UART					 
			printf("adv fast\r\n");
#endif				
 				//bsp_indication_set(BSP_INDICATE_ADVERTISING); //indication is limited, see update timer
            break;
        case BLE_ADV_EVT_IDLE:  //after advertise timeout, never happens in this implementation
 #ifdef DEBUG_UART					 
			printf("adv idle\r\n");
#endif           
				    bsp_indication_set(BSP_INDICATE_IDLE);
				    break;
        default:
            break;
    }
}


/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
				 isDataTransfer=false;
				 bsp_indication_set(BSP_INDICATE_IDLE);
         m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
#ifdef DEBUG_UART					 
			printf("connected\r\n");
#endif				
				 break;
            
        case BLE_GAP_EVT_DISCONNECTED:
				 isDataTransfer=false;        //for general contionious mode adv starts automatically
				 //advertising_init();
         //ble_advertising_start(BLE_ADV_MODE_FAST);
  			set_adv_packet(false);
#ifdef DEBUG_UART					 
			printf("disconnected\r\n");
#endif				    
            break;
 
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
       case BLE_EVT_TX_COMPLETE:   // added by MSH to serve continiously sending data, next chunk must be send on TX complete
       if (isDataTransfer) {//wait for buffer empty
				    isDataTransfer=false;
						prg_state=STATE_READ_RECORD;};
            break;		
            
            		
        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    ble_advertising_on_sys_evt(sys_evt);
		pstorage_sys_event_handler(sys_evt);  //need
}

/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */

static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
//MSH		
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
}





void bsp_event_handler(bsp_event_t event)
{
isDataTransfer=false; //stop transfer if was in action
static 	bsp_event_t previous_event;
	 
    switch (event)
    {
        
			case BSP_BUTTON_LONG_PUSH: 

				bsp_indication_set(BSP_INDICATE_ALL);
				nrf_delay_ms(1);
				bsp_indication_set(BSP_INDICATE_IDLE);
			  prgStop();
			
				     break;
       case BSP_BUTTON_PUSH:
  //			    ble_advertising_start(BLE_ADV_MODE_FAST); //restore advertising, not needed we have auto forever
				     break;
       case BSP_BUTTON_RELEASE: //sleep with no return (restart)
           if (previous_event==BSP_BUTTON_LONG_PUSH) 
					 {
						 
					   sleep_mode_enter(); //prevent from wakup with contact bounce
			     }
						break;			 
			  default:
            break;
    }	;
previous_event=event;		
};	



/**@brief Function for initializing the Advertising functionality.
 */


static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //forever allowed,  (180 s max) for BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
		APP_ERROR_CHECK(err_code);
}



static void set_adv_packet(bool isAlarm){
//mshdev, flags=06
uint32_t err_code;
//0x0B length (including type, excluding length	
//0xFF  custom data	
//"MY"	0x4D,0x59, ID required for custom info 
//09 name
//01 flags	
uint8_t bat=battery_level_get();
uint8_t infoArray[6];	
*(uint16_t*)&infoArray[0]=recNum;
*(uint16_t*)&infoArray[2]=ang.Pitch;
*(uint16_t*)&infoArray[4]=ang.Roll;

//"mshdev"	6D,73,68,64,65,76
uint8_t p_data[23]={0x0b,0xFF,0x4D,0x59, bat,mode_state|(isAlarm<<7),infoArray[0],infoArray[1],infoArray[2],infoArray[3],infoArray[4],infoArray[5],0x07,0x09,0x6D,0x73,0x68,0x64,0x65,0x76,0x02,0x01,0x06}; //genral mode, allowed forever adv
err_code=sd_ble_gap_adv_data_set(p_data,23,NULL,0);

APP_ERROR_CHECK(err_code);	
#ifdef DEBUG_UART
printf("conn  adv packet set\r\n");
#endif	

}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}




uint32_t getRealTime()  //get real time in UNIX format
{
uint32_t over=getOverloadCounts();
uint32_t countsCurrent = (NRF_RTC1->COUNTER);
return time.realTimeRef+((int32_t)(countsCurrent-time.rtcCountRef))/32768*(1+APP_TIMER_PRESCALER)+512*(1+APP_TIMER_PRESCALER)*over;
}

void setRealTime(uint32_t ts) //set real time
{
time.realTimeRef=ts;
time.rtcCountRef=NRF_RTC1->COUNTER;
clearOverloadCounts();	
}	


static void live_timer_handler(void * p_context)
{
int16_t ax,ay,az;
ACC_getAccData(&ax,&ay,&az);

     *(int16_t*)&dataArray[0]=ax; 
     *(int16_t*)&dataArray[2]=ay;
     *(int16_t*)&dataArray[4]=az;
	
#ifdef DEBUG_UART		
	    printf("ax=%d, ay=%d, ay=%d\r\n",ax,ay,az);
#endif	
			if (ble_nus_string_send(&m_nus, (uint8_t *)&dataArray[0], 6)==NRF_SUCCESS)	
		         bsp_indication_set(BSP_INDICATE_SENT_OK);

	
}

static void advAlarmOff_timer_handler(void * p_context){
isOffAlarmAdvRequested=false;
}

static bool live_Info(bool isAlarm){

#ifdef DEBUG_UART					 
						printf("live info try send out\r\n");
#endif		
 	uint8_t bat=battery_level_get();
  *(int8_t*)&dataArray[0]=bat; 
	if ((mode_state==MODE_RECORD)&&!isOffAlarm) //do not send alarms in record mode when connected
		    *(int8_t*)&dataArray[1]=mode_state; 
	           else *(int8_t*)&dataArray[1]=mode_state|(isAlarm<<7);
	*(int16_t*)&dataArray[2]=recNum;	
	*(int16_t*)&dataArray[4]=ang.Pitch;
	*(int16_t*)&dataArray[6]=ang.Roll;	
	
// if connected	
if (ble_nus_string_send(&m_nus, (uint8_t *)&dataArray[0], 8)==NRF_SUCCESS)	
   {	
#ifdef DEBUG_UART					 
						printf("live info sent out\r\n");
#endif		 
   //bsp_indication_set(BSP_INDICATE_SENT_OK);	//no indication on updates
   return true;
	 };
	 return false;
}	

static void send_AdvInfo(bool isAlarm) //send live when connected or via adv packet
{
if (!live_Info(isAlarm)) //we are offline, send via adv packets              
{	
	
if (isOffAlarm&&isAlarm){isOffAlarmAdvRequested=true; app_timer_start(advAlarmOff_timer,advAlarmOffTimerIntv, NULL);};
    if	(isOffAlarmAdvRequested){ //valid until advAlarmOff_timer on and next update (3sec)
      set_adv_packet(true);
      isOffAlarmAdvRequested=false;	
} else set_adv_packet(false);
}

}


static void advUpdate_timer_handler(void * p_context)
{
send_AdvInfo(false);
if (countAdvLedOFF) {
	countAdvLedOFF--;
	if (countAdvLedOFF==0) {
#ifdef DEBUG_UART		
	    printf("adv LED off\r\n");
		  bsp_indication_set(BSP_INDICATE_IDLE);
#endif	
	};
};
}	


static void keepAlive_timer_handler(void * p_context)
{
//do nothing this timer is for RTC1 running
}	


static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
	 //included to prevent the situation when all timers stopped ( actually not needed in this implementation since we always have update timer running) 
		err_code = app_timer_create(&keepAlive_timer,
                                APP_TIMER_MODE_REPEATED,
                                keepAlive_timer_handler);
    APP_ERROR_CHECK(err_code);
    
	  err_code = app_timer_create(&live_timer,
                                APP_TIMER_MODE_REPEATED,
                                live_timer_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&advUpdate_timer,
                                APP_TIMER_MODE_REPEATED,
                                advUpdate_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&advAlarmOff_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                advAlarmOff_timer_handler);
    APP_ERROR_CHECK(err_code);	

	  // set overflow interrupt, works together with modified app_timer.c
	  NRF_RTC1->INTENSET = (NRF_RTC1->INTENSET)|RTC_INTENSET_OVRFLW_Msk;	
	

}

void liveTrigStart(void){
						app_timer_stop(live_timer);
            isLiveTimerStarted=false;
	          ACC_setACmode(true); //always AC!
            mode_state=MODE_LIVE_TRIGGER;
						live_Info(false);
	          ACC_setLiveMode(true); //G_AOI_EVENT
						LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
	          ACC_setAOI_int();// "on: starts here
	          isOffAlarm=false;
						ang.Pitch=0;
						ang.Roll=0;
 						prg_state=STATE_IDLE;	
};


void liveStart(void){
            app_timer_start(live_timer,liveTimerIntv, NULL); //start reading
            isLiveTimerStarted=true;
						ACC_setACmode(false); //always DC!
	          mode_state=MODE_LIVE;
	          live_Info(false);
	          ACC_setSensorState(INT_IDLE); //no acc interrupts
						ACC_setLiveMode(false); //no G_AOI_EVENT
						LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
						ACC_start();
						ang.Pitch=0;
						ang.Roll=0;
          	isOffAlarm=false;
 						prg_state=STATE_IDLE;	
};



void recordStart(void){
						app_timer_stop(live_timer);
            isLiveTimerStarted=false;
	          ACC_setACmode(true); //always AC!
            mode_state=MODE_RECORD;
						ACC_setLiveMode(false); //no G_AOI_EVENT
						ACC_setAOI_int(); //read freq and on here
	          isOffAlarm=isOffAlarmSet;
	          recNum=0;
						ang.Pitch=0;
						ang.Roll=0;
 						prg_state=STATE_IDLE;

};

static void prgStop(){
      						app_timer_stop(live_timer);
                  isLiveTimerStarted=false;
	                ACC_setLiveMode(false); //no G_AOI_EVENT
                  mode_state=MODE_STOP;
									live_Info(false);
									ACC_setSensorState(INT_IDLE);
									ACC_stop();// acc off
									isOffAlarm=false;
	                LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
									prg_state=STATE_IDLE;	
};
//test only
void testReadFlash(uint32_t address,uint32_t numrec){

uint32_t test[1]={0};
for (int32_t i=0;i<numrec;i++){
uint32_t raddr=address+i*DATA_RECORD_SIZE;
flash_read_array(raddr,(uint8_t*)&test[0],4);
printf("read, addr=0x%x, data:0x%x\r\n",raddr,test[0]);
fflush(stdout); 	
nrf_delay_ms(20);
};

};	
// test only
void testWriteFlash(uint32_t address,uint32_t numrec){
currentRecordAddress=address;	
for (int32_t i=0;i<numrec;i++){
*(uint32_t*)&dataArray[0]=0x12340000+i;
*(uint32_t*)&dataArray[4]=0x98765432;	
recordDataToFlash();
nrf_delay_ms(20);	

};

};



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
#ifdef DEBUG_UART

//@snippet [Handling the data received over UART] 
void uart_event_handle(app_uart_evt_t * p_event)
{

  	static char uart_RX_buffer[BLE_NUS_MAX_DATA_LEN];
	  static uint8_t index = 0;	      

	switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_RX_buffer[index]));					

            index++;
            if ((uart_RX_buffer[index - 1] == '\r')||(uart_RX_buffer[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
						 if (strncmp(uart_RX_buffer, "connect ", 8) == 0){
 						printf("connect received\r\n");
            prg_state=STATE_CONNECT; 
						} else
						if (strncmp(uart_RX_buffer, "disconn ", 8) == 0){
 						printf("discconnect received\r\n");
            prg_state=STATE_ADVSTOP; 
						} else	
						if (strncmp(uart_RX_buffer, "alarm ", 6) == 0){
 						printf("alarm received\r\n");
            prg_state=STATE_ALARM; 
						} else							
						 if (strncmp(uart_RX_buffer, "live ", 5) == 0){
            
						printf("livemode cmd received\r\n");
            liveStart(); 
						} else									
						 if (strncmp(uart_RX_buffer, "live_trig ", 10) == 0){
            
						printf("live_trig cmd received\r\n");
            liveTrigStart(); 
						} else
						 if (strncmp(uart_RX_buffer, "live ", 5) == 0){
            
						printf("livemode cmd received\r\n");
            liveStart(); 
						} else						 
						 if (strncmp(uart_RX_buffer, "record ", 7) == 0){
             printf("record cmd received\r\n");
             recordStart();
						 } else
						 if (strncmp(uart_RX_buffer, "stop ", 5) == 0){
						 printf("stop cmd received \r\n");
             prgStop();							 
						 } else
						 if (strncmp(uart_RX_buffer, "tr ", 3) == 0){
						 uint32_t address=strtol((char*)&uart_RX_buffer[3],NULL,0);
						 uint32_t numrec=atol((char*)&uart_RX_buffer[11]);
             testWriteFlash(address,numrec);							 
							 printf("address:0x%x num(dec): %d\r\n",address,numrec);
             							 
						 }
						  else
						 if (strncmp(uart_RX_buffer, "rr ", 3) == 0){
						 uint32_t address=strtol((char*)&uart_RX_buffer[3],NULL,0);
						 uint32_t numrec=atol((char*)&uart_RX_buffer[11]);
             testReadFlash(address,numrec);							 
							 printf("test read, address:0x%x num(dec): %d\r\n",address,numrec);
             							 
						 }						 
						 else
						 if (strncmp(uart_RX_buffer, "erase_page ", 11) == 0){
						 uint32_t page=atol((char*)&uart_RX_buffer[11]);
							 printf("page erased: %d\r\n",START_PAGE_NUM+page);
             	flash_page_erase(START_PAGE_NUM+page);						 
						 }						 
						 else
						 if (strncmp(uart_RX_buffer, "wf ", 3) == 0){
						 uint32_t address=strtol((char*)&uart_RX_buffer[3],NULL,0);
						 uint32_t data=atol((char*)&uart_RX_buffer[11]);
             currentRecordAddress=address;	
             *(uint32_t*)&dataArray[0]=data;
             *(uint32_t*)&dataArray[4]=0x98765432;	
             recordDataToFlash();							 
						 printf("written address:0x%x data(dec): %d\r\n",address,data);;						 
						 }
						  else
						 if (strncmp(uart_RX_buffer, "ts ", 3) == 0){
						 uint32_t address=strtol((char*)&uart_RX_buffer[3],NULL,0);
             currentRecordAddress=address;
						 int32_t found0=searchStartRecord();							 
						 printf("found0, address:0x%x\r\n",found0);
             							 
						 }	
						  else
						 if (strncmp(uart_RX_buffer, "read_rec ", 9) == 0){
	           int32_t startAddr;
							 
						 startAddr=searchStartRecord();	 
#ifdef DEBUG_UART				
			        printf("read data from flash, start addr 0x%x\n\r",startAddr);
#endif
             
             if (startAddr) {readRecordAddress=startAddr+DATA_RECORD_SIZE;				
                     if (currentRecordAddress-readRecordAddress)
						            numberOfRecords=(currentRecordAddress-readRecordAddress)/DATA_RECORD_SIZE;
								     else
										    numberOfRecords=((VAR_ADDR-readRecordAddress)+(currentRecordAddress-START_DATA_ADDR))/DATA_RECORD_SIZE;             
#ifdef DEBUG_UART				
			        printf("num records= %d\n\r",numberOfRecords);
#endif
             currentRecordNum=numberOfRecords;
						 prg_state=STATE_READ_RECORD;
						 } 
						 };								 
                index = 0;
            }            
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


///@snippet [UART Initialization] 
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


#endif //uart debug


static void showLedRing()
{
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
for(uint8_t i=0;i<LEDS_NUMBER;i++){	
	LEDS_ON(1<<leds_list[i]);
	nrf_delay_ms(1);
	LEDS_OFF(1<<leds_list[i]);
	nrf_delay_ms(10);
}
}


void sensor_event_handler(gsensor_event_t event){

switch (event)
{
	case G_SENSORS_EVENT_FIFO_READY:
#ifdef DEBUG_UART	
	printf("FIFO ready\r\n");
#endif
	prg_state=STATE_READ_FIFO;
	break;
	case G_SENSORS_EVENT_AT_REST:
#ifdef DEBUG_UART
	printf("at rest\r\n");
#endif		
	app_timer_stop(live_timer); //stop reading
  isLiveTimerStarted=false;
	if (mode_state==MODE_LIVE){  
			ACC_setAOI_int(); //continue
			prg_state=STATE_IDLE;	
	} else prg_state=STATE_READ_ORIENTATION; 

	break;
  case G_AOI_EVENT: 
#ifdef DEBUG_UART			
	printf("aoi event\r\n");	
#endif
  prg_state=STATE_ALARM;
	if (mode_state==MODE_LIVE_TRIGGER) {
	app_timer_start(live_timer,liveTimerIntv, NULL); //start reading
  isLiveTimerStarted=true;
	};
	break;
	
	default: //not implemented
	break;	
};

}



static void recordDataToFlash()
{
uint32_t page_to_erase;	
uint32_t page=currentRecordAddress/FLASH_PAGE_SIZE; //current flash page	
uint32_t rest_size=FLASH_PAGE_SIZE-(currentRecordAddress%FLASH_PAGE_SIZE); //size to the end of page
printf("page= %d, res=%d\r\n",page,rest_size);
if (rest_size<=DATA_RECORD_SIZE){ 	// does not fit, need to erase next page
       if ((page+1)>=(VAR_ADDR/FLASH_PAGE_SIZE)){ //last page for the data, last page is for some other variables (currently not used)
			 page_to_erase=START_DATA_ADDR/FLASH_PAGE_SIZE;
			 printf("page= 0\r\n");	 
			 } else page_to_erase=page+1;
						        flash_page_erase(page_to_erase);
#ifdef DEBUG_UART										
										printf("page erased= %d\n\r",page_to_erase);	
#endif
          };
flash_write_array(currentRecordAddress,&dataArray[0],DATA_RECORD_SIZE);			
#ifdef DEBUG_UART
							printf("recorded at addr= 0x%x\r\n",currentRecordAddress);
#endif
							currentRecordAddress+=DATA_RECORD_SIZE;			
              if (currentRecordAddress>(VAR_ADDR-DATA_RECORD_SIZE)) 
								                    currentRecordAddress=START_DATA_ADDR;										
}	



int32_t searchFlashEnd()  //search for 0xFFFFFFFF (empty flash follows...) 
{
uint32_t test[1]={0};
uint32_t offset=START_DATA_ADDR;
while ((test[0]!=0xFFFFFFFF)&&(offset<=(VAR_ADDR-DATA_RECORD_SIZE))) 
{
flash_read_array(offset,(uint8_t*)&test[0],4);
offset+=DATA_RECORD_SIZE;		
};
if (*(uint32_t*)&test[0]!=0xFFFFFFFF) return -1; //not found
  else return (offset-DATA_RECORD_SIZE); 
}



static int32_t searchStartRecord()  //search for 0x00000000 (backward from currentFlashRecordNum)
{
uint16_t reccnt=0;
uint16_t maxrecNum=(VAR_ADDR-START_DATA_ADDR)/DATA_RECORD_SIZE;	
uint32_t test[1]={0xffffffff};
int32_t addr=currentRecordAddress;
while ((test[0]!=0x00000000)&&(reccnt<maxrecNum))   //&&(offset<END_DATA_ADDR)) 
{
reccnt++;	
addr-=DATA_RECORD_SIZE; 	
if ((addr-START_DATA_ADDR)<0) addr=VAR_ADDR+(addr-START_DATA_ADDR);
	
flash_read_array(addr,(uint8_t*)&test[0],4);
}	
if (reccnt>=maxrecNum) return currentRecordAddress+1; //next, full range data will be returned (VAR_ADDR-START_DATA_ADDR)
else return addr;
}






/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	  int16_t ax,ay,az;
		int32_t sax, say, saz;
    static uint16_t maxAcc=0; //static: collect data from many calls
    uint16_t absAcc=0;
 

	  
	uint32_t uicrTemp =  *(uint32_t*)(0x10001014);	
    if (uicrTemp != 0x0003C000){	
    flash_word_write((uint32_t*)0x10001014,0x0003C000);  //fix bootloader address
    flash_word_write((uint32_t*)0x0003fc00,0x00000001);  //fix application valid	
    }	

	
#ifdef DEBUG_UART		
    uart_init();
		printf("\n\rSTART\r\n");
#endif	
    countAdvLedOFF=5;		//advertising led for 5x3s 
	  timers_init();
		app_timer_start(keepAlive_timer, rtcKeepAliveIntv, NULL); 
		
    leds_init(APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)); 
		
	  buttons_init(APP_TIMER_TICKS(10, APP_TIMER_PRESCALER),bsp_event_handler); 


		ACC_init_Sensor(APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)); //default rest time gap duration

		ACC_callback_init(sensor_event_handler);
		
		
		
    ble_stack_init();
	
		//DFU
		device_manager_init(erase_bonds);	
	  //DFUe
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		bsp_indication_set(BSP_INDICATE_ADVERTISING);
	  APP_ERROR_CHECK(err_code);
		set_adv_packet(false);
    currentRecordAddress=searchFlashEnd();
    app_timer_start(advUpdate_timer,advUpdateTimerIntv, NULL);
    
		sd_ble_gap_tx_power_set(4); //max 4dbm
		 
#ifdef DEBUG_UART		
		printf("currentRecordAddress 0x%x\n\r",currentRecordAddress);
		printf("current record# dec: %d\r\n",(currentRecordAddress-START_DATA_ADDR)/DATA_RECORD_SIZE);
#endif

		for (;;)
    {
		switch (prg_state)
    {			
		  case STATE_READ_FIFO: 
#ifdef DEBUG_UART
			printf("read_fifo\r\n");
#endif
		
		  while (!(ACC_getAccData(&ax,&ay,&az)&LIS2DH_EMPTY_MASK)){
#ifdef DEBUG_UART				
	    printf("ax=%d, ay=%d, az=%d\r\n",ax,ay,az);
			nrf_delay_ms(10);	
#endif	
     absAcc=absAcceleration(ax,ay,az);
		 if (absAcc>maxAcc) maxAcc=absAcc;	
}
		
	    ACC_setAOI_int(); //continue
			prg_state=STATE_IDLE;		
      break;

			case STATE_READ_ORIENTATION: //after at_rest event 
			ACC_setSensorState(INT_IDLE); //disable interrupts
			ACC_setACmode(false); //no filter
			ACC_start();
			LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);//clear fifo, bypass
			ACC_getAccData(&ax,&ay,&az);
      sax=0,say=0,saz=0;	
			for(uint8_t i=0; i<5; i++){
			while (!ACC_getStatusReg()){}; //new data have come
			ACC_getAccData(&ax,&ay,&az);
#ifdef DEBUG_UART					
			printf("ax=%d, ay=%d, ay=%d\r\n",ax,ay,az);
			nrf_delay_ms(10);	
#endif				
      sax+=ax; say+=ay; saz+=az;
			};
			ang=calculateAngles(sax/5,say/5,saz/5);
#ifdef DEBUG_UART				
			printf("pitch=%d, roll=%d\r\n",ang.Pitch,ang.Roll);
			nrf_delay_ms(10);
#endif	
			live_Info(false);

if (mode_state==MODE_RECORD){
     *(uint32_t*)&dataArray[0]=getRealTime();

	  *(uint32_t*)&dataArray[4]=((maxAcc&0x3FF)<<20)+((ang.Pitch&0x3FF)<<10)+(ang.Roll&0x3FF);	
	
	   recordDataToFlash();
	   recNum++;
#ifdef DEBUG_UART				
			printf("record to flash\r\n");
			printf("%d, %d\r\n",*(int32_t*)&dataArray[0],*(uint32_t*)&dataArray[4]);	
      nrf_delay_ms(10);
#endif	

     };
			
			
			maxAcc=0;
			ACC_setACmode(true); //always true in this version
			ACC_setAOI_int(); //continue
			prg_state=STATE_IDLE;	
      break;				
			
			case STATE_ALARM: 
#ifdef DEBUG_UART					
				printf("state_alarm\n\r");
			nrf_delay_ms(10);
#endif					
			prg_state=STATE_IDLE; 
#ifdef DEBUG_UART					
			printf("isOffAlarm %d\n\r",isOffAlarm);
			nrf_delay_ms(10);
#endif
      send_AdvInfo(true);			
	
      break;
	
			case STATE_ADVSTOP:
	
			reset_prepare(); //disconnect or (if advertising started, adv stop)
			
#ifdef DEBUG_UART					 
						printf("adv stopped\r\n");
#endif			

			
			prg_state=STATE_IDLE;
			break;
			
			case STATE_READ_RECORD: 

			  prg_state=STATE_IDLE;
				while (readRecordAddress!=currentRecordAddress){
				
				*(uint32_t*)&dataArray[8]=currentRecordNum;	
				flash_read_array(readRecordAddress,(uint8_t*)&dataArray[0],DATA_RECORD_SIZE);
				
					
			  err_code=ble_nus_string_send(&m_nus, (uint8_t *)&dataArray[0], DATA_RECORD_SIZE+4); //fake 4 bytes to get size=12 to recognize type by android and use 3x uint32
	          if (err_code == BLE_ERROR_NO_TX_BUFFERS ||
            err_code == NRF_ERROR_INVALID_STATE || 
            err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        {
				isDataTransfer=true;	
   			break;	// wait for complete in "on_ble_evnt" on BLE_EVT_TX_COMPLETE (should be added)
				}
        else if (err_code != NRF_SUCCESS) 
        {
         //bsp_indication_set(BSP_INDICATE_ERROR);
         //do nothing					
				};					
				bsp_indication_set(BSP_INDICATE_SENT_OK);	
					
				currentRecordNum--;	
#ifdef DEBUG_UART					
				printf("addr=0x%x data[0]=%d data[1]=%d data[2]=%d\r\n",readRecordAddress,*(uint32_t*)&dataArray[0],*(uint32_t*)&dataArray[4],*(uint32_t*)&dataArray[8]);	
        nrf_delay_ms(10);
#endif
				readRecordAddress+=DATA_RECORD_SIZE;
					
				if (readRecordAddress>=VAR_ADDR) readRecordAddress=START_DATA_ADDR+readRecordAddress-VAR_ADDR;					
			  
				
				}	//end while
      break;			
			
			case STATE_IDLE:
			power_manage();
      break;

			
      default:
			power_manage();  //any other state, not called	
      break;	
    };
			 	
 

		}	
			
	
    
}
