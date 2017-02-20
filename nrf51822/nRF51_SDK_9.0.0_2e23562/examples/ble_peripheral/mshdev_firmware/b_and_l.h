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
// modified by MSH


#ifndef BAL_H__
#define BAL_H__

#include "nrf_gpio.h"

//#define BSP_MS_TO_TICK(MS) (m_app_ticks_per_100ms * (MS / 100))
#define BSP_MS_TO_TICK(MS) (m_app_ticks_per_10ms * (MS / 10))

#define LEDS_OFF(leds_mask) do {  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                            NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask) do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                           NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)

#define ALERT_LED_MASK LED_B

#define BTN_ID_WAKEUP             0  /**< ID of button used to wake up the application. */
																					
																					
#define ADVERTISING_LED_ON_INTERVAL            10
#define ADVERTISING_LED_OFF_INTERVAL           1800
																					
																					
																		
																					
																					

//#define SENT_OK_INTERVAL                       100
//#define SEND_ERROR_INTERVAL                    500

//#define RCV_OK_INTERVAL                        100
//#define RCV_ERROR_INTERVAL                     500

//#define ALERT_INTERVAL                         200																					

#define BSP_BUTTON_ACTION_PUSH      (APP_BUTTON_PUSH)    /**< =1 defined in "app_button.h"*/
#define BSP_BUTTON_ACTION_RELEASE   (APP_BUTTON_RELEASE) /**< =0 defined in "app_button.h"*/
#define BSP_BUTTON_ACTION_LONG_PUSH (2)  																					
						
#define BSP_LONG_PUSH_TIMEOUT_MS (1000) /**< The time to hold for a long push (in milliseconds). */


typedef enum
{
    BSP_EVENT_NOTHING = 0,                  /**< Assign this event to an action to prevent the action from generating an event (disable the action). */
    BSP_BUTTON_PUSH,                // msh since we have 1 button, program action depends on program state, it will be easier handle simple button events   
    BSP_BUTTON_RELEASE,            
    BSP_BUTTON_LONG_PUSH,             
} bsp_event_t;

typedef struct
{
    bsp_event_t push_event;      /**< The event to fire on regular button press. */
    bsp_event_t long_push_event; /**< The event to fire on long button press. */
    bsp_event_t release_event;   /**< The event to fire on button release. */
} bsp_button_event_cfg_t;

typedef void (* bsp_event_callback_t)(bsp_event_t);  //msh this callback will be used to pass events to main function



typedef enum
{
    BSP_INDICATE_IDLE,
    BSP_INDICATE_ADVERTISING,
	  BSP_INDICATE_CONNECTED,
    BSP_INDICATE_SENT_OK,                    
    BSP_INDICATE_RCV_OK,                     
    BSP_INDICATE_ERROR,
	  BSP_INDICATE_LEDB,
    BSP_INDICATE_ALL=BSP_INDICATE_ERROR	
} led_indication_t;



//uint32_t led_indication(led_indication_t indicate);

uint32_t bsp_indication_set(led_indication_t indicate);
uint32_t leds_init(uint32_t ticks_per_10ms);

uint32_t buttons_init(uint32_t ticks_per_10ms, bsp_event_callback_t callback);
uint32_t bsp_btn_ble_sleep_mode_prepare(void);

#endif // BAL_H__


