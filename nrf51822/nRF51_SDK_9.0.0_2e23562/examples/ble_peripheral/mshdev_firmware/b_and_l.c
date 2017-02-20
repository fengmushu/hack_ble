//*Mikhail Sharonov, aka obelix662000@yahoo.com, Copyright (c) 2016. All Rights Reserved.
//*
//*

#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "MISFIT.h"
#include "app_timer.h"
#include "app_button.h"
#include "b_and_l.h"
#include "nrf_delay.h"




static bsp_event_callback_t   m_registered_callback         = NULL;

static app_timer_id_t   m_button_timer_id; //used to check long push
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action);
static const app_button_cfg_t app_buttons[BUTTONS_NUMBER] =
{
    #ifdef BUTTON_0
    {BUTTON_0, false, BUTTON_PULL, bsp_button_event_handler},  //pin, active state low, pullup, event handler
    #endif // BUTTON_0

    #ifdef BUTTON_1
    {BUTTON_1, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_1

    #ifdef BUTTON_2
    {BUTTON_2, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_2
};		

static const uint32_t m_buttons_list[BUTTONS_NUMBER] = BUTTONS_LIST;
static bsp_button_event_cfg_t m_events_list[BUTTONS_NUMBER] = {{BSP_BUTTON_PUSH, BSP_BUTTON_LONG_PUSH, BSP_BUTTON_RELEASE}}; //we deal with simple button actions
// we have only one button, so not much sense here, left for possible redefinition of bsp events


static app_timer_id_t   m_leds_timer_id;																					
static led_indication_t m_stable_state        = BSP_INDICATE_IDLE;
static uint32_t         m_app_ticks_per_10ms = 0;																			
																					

static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action)  //defined in "app_button.h" and called every time button change state (contact bounce already removed)
{
    bsp_event_t        event  = BSP_EVENT_NOTHING;
    uint32_t           button = 0;
    uint32_t           err_code;
    static uint8_t     current_long_push_pin_no;              /**< Pin number of a currently pushed button, that could become a long push if held long enough. */
    static bsp_event_t release_event_at_push[BUTTONS_NUMBER]; /**< Array of what the release event of each button was last time it was pushed, so that no release event is sent if the event was bound after the push of the button. */	
//static means they keep data for a lifetime	

//	if((pin_no==BUTTON_0)&&(button_action==BSP_BUTTON_ACTION_PUSH)) led_indication_set(BSP_INDICATE_CONNECTED);
//	if(pin_no==BUTTON_1) led_indication_set(BSP_INDICATE_ADVERTISING);

   while ((button < BUTTONS_NUMBER) && (m_buttons_list[button] != pin_no))
    {
        button++;
    }

    if (button < BUTTONS_NUMBER)
    {
        switch(button_action)
        {
            case APP_BUTTON_PUSH:
                event = m_events_list[button].push_event;
                if (m_events_list[button].long_push_event != BSP_EVENT_NOTHING)
                {
                    err_code = app_timer_start(m_button_timer_id, BSP_MS_TO_TICK(BSP_LONG_PUSH_TIMEOUT_MS), (void*)&current_long_push_pin_no);
                    if (err_code == NRF_SUCCESS)
                    {
                        current_long_push_pin_no = pin_no;
                    }
                }
                release_event_at_push[button] = m_events_list[button].release_event;
                break;
            case APP_BUTTON_RELEASE:
                (void)app_timer_stop(m_button_timer_id);
                if (release_event_at_push[button] == m_events_list[button].release_event)
                {
                    event = m_events_list[button].release_event;
                }
                break;
            case BSP_BUTTON_ACTION_LONG_PUSH:
                event = m_events_list[button].long_push_event;
        }
    }

    if ((event != BSP_EVENT_NOTHING) && (m_registered_callback != NULL))
    {
        m_registered_callback(event);
    }

	
}


static uint32_t led_indication(led_indication_t indicate)
{
    uint32_t err_code   = NRF_SUCCESS;
    uint32_t next_delay = 0;

    switch (indicate)
    {
        case BSP_INDICATE_IDLE:
            LEDS_OFF(LEDS_MASK & ~ALERT_LED_MASK);
            m_stable_state = indicate;  // will hold steady, if not, will be replaced  
            break;

        case BSP_INDICATE_ADVERTISING: //red led flashing
           LEDS_OFF(LEDS_MASK & ~LED_R_MASK);
 
            if (LED_IS_ON(LED_R_MASK))
            {
                LEDS_OFF(LED_R_MASK);
                next_delay=ADVERTISING_LED_OFF_INTERVAL;
            }
            else
            {
                LEDS_ON(LED_R_MASK);
                next_delay = ADVERTISING_LED_ON_INTERVAL;            }

            m_stable_state = indicate;
            err_code       = app_timer_start(m_leds_timer_id, BSP_MS_TO_TICK(next_delay), NULL);
            break;

        case BSP_INDICATE_CONNECTED:  //green steady
            LEDS_OFF(LEDS_MASK & ~LED_G_MASK);
            LEDS_ON(LED_G_MASK);
				    m_stable_state = indicate;
            break;

        case BSP_INDICATE_SENT_OK:    //when sending shortly invert led blue
						LEDS_ON(LED_R_MASK);
				    nrf_delay_ms(1);
				    LEDS_OFF(LED_R_MASK);
						
		           
           break;

        case BSP_INDICATE_LEDB:
            //         case BSP_INDICATE_SENT_OK:    //when sending shortly invert led blue
				    LEDS_ON(LED_B_MASK);
				    nrf_delay_ms(1);
				    LEDS_OFF(LED_B_MASK);
			
           break;
						
       case BSP_INDICATE_RCV_OK:    //when sending shortly invert led green
				 		LEDS_ON(LED_G_MASK);
				    nrf_delay_ms(1);
				    LEDS_OFF(LED_G_MASK);
     
           break;


        case BSP_INDICATE_ERROR:
            // on fatal error turn on all leds
            LEDS_ON(LEDS_MASK);
				   nrf_delay_ms(10);
				   m_stable_state = indicate;
            break;


        default:
            break;

}

    return err_code;
}

																					

static void leds_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    UNUSED_VARIABLE(led_indication(m_stable_state));
    
}
																					
static void button_timer_handler(void * p_context)
{
    bsp_button_event_handler(*(uint8_t *)p_context, BSP_BUTTON_ACTION_LONG_PUSH);
}



uint32_t leds_init(uint32_t ticks_per_10ms)
{
uint32_t err_code;
	
  //m_app_ticks_per_10ms = ticks_per_100ms;	
	if (m_app_ticks_per_10ms==0) m_app_ticks_per_10ms = ticks_per_10ms;	// if not defined
	//used by BSP_MS_TO_TICK
  LEDS_CONFIGURE(LEDS_MASK);
	LEDS_OFF(LEDS_MASK);
  //NRF_GPIO->DIRSET = LEDS_MASK;	

	err_code = app_timer_create(&m_leds_timer_id, APP_TIMER_MODE_SINGLE_SHOT, leds_timer_handler);


  return err_code;	
}



uint32_t buttons_init(uint32_t ticks_per_10ms, bsp_event_callback_t callback)
{
uint32_t err_code;
m_registered_callback = callback;
	
	if (m_app_ticks_per_10ms==0) m_app_ticks_per_10ms = ticks_per_10ms;	// if not defined
	
  err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                       BUTTONS_NUMBER,
                                       ticks_per_10ms /2 ); //  /2 50ms delay to detect button push
	
  if (err_code == NRF_SUCCESS)
        {
            err_code = app_button_enable();
        };	
	if (err_code == NRF_SUCCESS)
        {
      err_code = app_timer_create(&m_button_timer_id,
                                        APP_TIMER_MODE_SINGLE_SHOT,
                                        button_timer_handler); //in the bsp_button_event_handler start timer on push, stop on release
        }


  return err_code;	
}




uint32_t bsp_indication_set(led_indication_t indicate)
{
 uint32_t err_code = NRF_SUCCESS;

        err_code = led_indication(indicate);

    return err_code;
}


uint32_t bsp_wakeup_buttons_set(uint32_t wakeup_buttons)
{
#if (BUTTONS_NUMBER > 0) && !defined(BSP_SIMPLE)
    for (uint32_t i = 0; i < BUTTONS_NUMBER; i++)
    {
        uint32_t new_cnf = NRF_GPIO->PIN_CNF[m_buttons_list[i]];
        uint32_t new_sense = ((1 << i) & wakeup_buttons) ? GPIO_PIN_CNF_SENSE_Low : GPIO_PIN_CNF_SENSE_Disabled;
        new_cnf &= ~GPIO_PIN_CNF_SENSE_Msk;
        new_cnf |= (new_sense << GPIO_PIN_CNF_SENSE_Pos);
        NRF_GPIO->PIN_CNF[m_buttons_list[i]] = new_cnf;
    }
    return NRF_SUCCESS;
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

uint32_t bsp_btn_ble_sleep_mode_prepare(void)
{
    uint32_t err_code = bsp_wakeup_buttons_set((1 << BTN_ID_WAKEUP)); //button number 

   return err_code;
}

