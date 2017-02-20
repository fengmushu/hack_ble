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
#ifndef MISFIT_H
#define MISFIT_H

/*
#define LEDS_NUMBER    4

#define LED_START      21
#define LED_1          21
#define LED_2          22
#define LED_3          23
#define LED_4          24
#define LED_STOP       24

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }
*/
/*
#define LEDS_NUMBER    12

#define LED_0			     19
#define LED_1			     18
#define LED_2			     12
#define LED_3			     10
#define LED_4			     9
#define LED_5			     30
#define LED_6			     28
#define LED_7			     29
#define LED_8			     22
#define LED_9			     21
#define LED_10		     25
#define LED_11		     23

#define LEDS_LIST { LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9, LED_10, LED_11  }
*/

#define LEDS_NUMBER    3
#define LED_0          23  //D11
#define LED_1          19  //D0
#define LED_2          18  //D1 


#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1
#define BSP_LED_2      LED_2




/*
#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define BSP_LED_3_MASK (1<<BSP_LED_3)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
*/
/*
#define LED_0_MASK (1<<LED_0)
#define LED_1_MASK (1<<LED_1)
#define LED_2_MASK (1<<LED_2)
#define LED_3_MASK (1<<LED_3)
#define LED_4_MASK (1<<LED_4)
#define LED_5_MASK (1<<LED_5)
#define LED_6_MASK (1<<LED_6)
#define LED_7_MASK (1<<LED_7)
#define LED_8_MASK (1<<LED_8)
#define LED_9_MASK (1<<LED_9)
#define LED_10_MASK (1<<LED_10)
#define LED_11_MASK (1<<LED_11)

#define LEDS_MASK      (LED_0_MASK | LED_1_MASK | LED_2_MASK | LED_3_MASK | LED_4_MASK | LED_5_MASK | LED_6_MASK | LED_7_MASK | LED_8_MASK | LED_9_MASK | LED_10_MASK | LED_11_MASK)
*/
//#define LEDS_NUMBER    4
/* all LEDs are lit when GPIO is low */



/*
#define LED_R          19
#define LED_G          18
#define LED_B          12

//#define LEDS_LIST { LED_R, LED_G, LED_B}

#define LED_R_MASK (1<<LED_R)
#define LED_G_MASK (1<<LED_G)
#define LED_B_MASK (1<<LED_B)

#define LEDS_MASK      (LED_R_MASK | LED_G_MASK | LED_B_MASK)
*/
/* all LEDs are lit when GPIO is low */
//#define LEDS_INV_MASK  !LEDS_MASK

/*
#define BSP_LED_0      LED_G  //led which blinks while dfu advertising, and? transfer 
#define BSP_LED_1      LED_G //led which stay when connected, see dfu_transport_ble.c and main.c
*/

#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 1



#define BUTTONS_LIST { BUTTON_0}
#define BUTTON_0       17
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL //NRF_GPIO_PIN_PULLUP
#define BUTTON_0_MASK (1<<BUTTON_0)
#define BUTTONS_MASK (BUTTON_0_MASK) 


#define RX_PIN_NUMBER  15
#define TX_PIN_NUMBER  13
#define CTS_PIN_NUMBER 14
#define RTS_PIN_NUMBER 16
#define HWFC           true


#define SPIM0_SCK_PIN       5     //< SPI clock GPIO pin number. 
#define SPIM0_MOSI_PIN      4     //< SPI Master Out Slave In GPIO pin number. 
#define SPIM0_MISO_PIN      3    //< SPI Master In Slave Out GPIO pin number. 
#define SPIM0_SS_PIN        2     //< SPI Slave Select GPIO pin number.



#endif // MISFIT_H
