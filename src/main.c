/********************************************************************************
  * @file    main.c
  * @author  A. Riedinger & G. Stang.
  * @version 0.1
  * @date    09-11-21.
  * @brief   Generacion de un filtro pasa algtos digital FIR con n = 10 y
  	  	  	 fc = 2kHz segun una ventana tipo Hamming.

  * SALIDAS:
  	  *	LCD  Conexion Estandar TPs

  * ENTRADAS:
  	  * TEC-MAT
********************************************************************************/

/*------------------------------------------------------------------------------
LIBRERIAS:
------------------------------------------------------------------------------*/
#include "functions.h"

/*------------------------------------------------------------------------------
DEFINICIONES LOCALES:
------------------------------------------------------------------------------*/
/*Pines del ADC - PC0:*/
#define ADC_Port GPIOC
#define ADC_Pin  GPIO_Pin_0

/*Base de tiempo para el TIM3:*/
#define TimeBase 200e3 //[mseg]
/*Frecuencia de interrupcion del TIM3:*/
#define FS  2500 //[Hz]

/*------------------------------------------------------------------------------
VARIABLES GLOBALES:
------------------------------------------------------------------------------*/

int main(void)
{
/*------------------------------------------------------------------------------
CONFIGURACION DEL MICRO:
------------------------------------------------------------------------------*/
	SystemInit();

	/*Inicializacion del ADC:*/
	INIT_ADC(ADC_Port, ADC_Pin);

	/*Inicialización del TIM3:*/
	INIT_TIM3();
	SET_TIM3(TimeBase, FS);

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
	while(1)
	{
	}
}
/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
/*Interrupcion al vencimiento de cuenta de TIM3 cada 1/FS:*/
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*------------------------------------------------------------------------------
TAREAS:
------------------------------------------------------------------------------*/
