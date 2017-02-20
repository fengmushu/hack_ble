//*Mikhail Sharonov, aka obelix662000@yahoo.com, Copyright (c) 2016. All Rights Reserved.
//*
//*
#ifndef MISFIT_H
#define MISFIT_H

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


#define LEDS_INV_MASK  LEDS_MASK


// for simplicity to be in line with my other projects, lets call main 3 leds (used for indication)  R,G,B 
#define LED_R          LED_0
#define LED_G          LED_1
#define LED_B          LED_2


#define LED_R_MASK (1<<LED_R)
#define LED_G_MASK (1<<LED_G)
#define LED_B_MASK (1<<LED_B)


#define BSP_LED_0      LED_B
#define BSP_LED_1      LED_B


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
