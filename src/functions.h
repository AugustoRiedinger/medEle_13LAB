/* Definicion del header:*/
#ifndef functions_H
#define functions_H

/* Librerias internas de la STM:*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stdio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"

/* Estructuras:*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  		TIM_OCInitStructure;
EXTI_InitTypeDef   		EXTI_InitStructure;
DAC_InitTypeDef 		DAC_InitStructure;

/* Declaracion funciones:*/
void INIT_DO(GPIO_TypeDef* Port, uint32_t Pin);
void INIT_ADC(GPIO_TypeDef* Port, uint16_t Pin);
int32_t READ_ADC(GPIO_TypeDef* Port, uint16_t Pin);
void INIT_DAC_CONT(GPIO_TypeDef* Port, uint16_t Pin);
void DAC_CONT(GPIO_TypeDef* Port, uint16_t Pin, int16_t MiliVolts);
void INIT_TIM3();
void SET_TIM3(uint32_t TimeBase, uint32_t Freq);

/* Cierre del header:*/
#endif
