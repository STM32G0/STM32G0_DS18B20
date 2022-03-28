/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32g071xx.h>
#include "ds18b20.h"
#include "delay.h"

#define LED_SetHigh()     (GPIOA->BSRR |= GPIO_BSRR_BS8) 
#define LED_SetLow()      (GPIOA->BSRR |= GPIO_BSRR_BR8) 
#define LED_Toggle()      ((GPIOA->ODR & GPIO_ODR_OD8)  ? (GPIOA->BSRR |= GPIO_BSRR_BR8) : (GPIOA->BSRR |= GPIO_BSRR_BS8))

#define MCU_CLK 16000000    /* 16 MHz */
#define DELAY_TIM_FREQUENCY 1000000 /* = 1MHZ -> timer runs in microseconds */

void delay_us(uint16_t us);
void delay_ms(uint16_t ms);

int main(void) {
SystemInit();
  /* zwoka na ustabilizowanie si zegarow / koniecznie musi by */
for (uint32_t i = 0; i < 5000; i++) {asm("nop");}
RCC->IOPENR |= RCC_IOPENR_GPIOAEN; //Open clock for GPIOA
RCC->IOPENR |= RCC_IOPENR_GPIOCEN; //Open clock for GPIOC
RCC->APBENR1 |= RCC_APBENR1_TIM6EN;  // Enable the timer6 clock for 1-Wire/DS18B20
/* PC6 set Output */
GPIOC->MODER |=  GPIO_MODER_MODE6_0; //MODE6 -> 0b01
GPIOC->MODER &= ~GPIO_MODER_MODE6_1; //MODE6 -> 0b01
/* PC6 set High */
GPIOC->BSRR |= GPIO_BSRR_BS6;
/* PA8 set Output for LED */
GPIOA->MODER |=  GPIO_MODER_MODE8_0; //MODE8 -> 0b01
GPIOA->MODER &= ~GPIO_MODER_MODE8_1; //MODE8 -> 0b01

/* Timer6 init */
TIM6->PSC = (MCU_CLK / DELAY_TIM_FREQUENCY) - 1;;  // 16MHz/16 = 1 MHz ~~ 1 uS per tick counter
TIM6->ARR = 0x0FFF; 
TIM6->CR1 |= TIM_CR1_CEN; // Enable the Counter
while (!(TIM6->SR & TIM_SR_UIF));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated


  
   while (1) {
  
  temperatura();
 
     
   };
}






