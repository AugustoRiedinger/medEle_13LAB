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
/*Base de tiempo para el TIM3:*/
#define timeBase 200e3 //[mseg]

/*Frecuencia de muestreo segun el TIM3:*/
#define FS  5000 //[Hz]

/*Tareas del TS:*/
void ADC_PROCESSING(void);
void LCD(void);

/*Ticks del TS:*/
/*Se toma un dato del ADC en cada instante de muestreo:*/
#define ticksADC	1
/*Se clarea el display cada 200mseg:*/
#define ticksLCD	1000

/*------------------------------------------------------------------------------
VARIABLES GLOBALES:
------------------------------------------------------------------------------*/
/*Variables de manejo del TS:*/
uint32_t adc = 0;
uint32_t lcd = 0;

/*Variables de almacenamiento de datos del ADC:*/
float voltValue = 0.0f;
float currValue = 0.0f;

int main(void)
{
/*------------------------------------------------------------------------------
CONFIGURACION DEL MICRO:
------------------------------------------------------------------------------*/
	SystemInit();

	/*Inicializacion del DISPLAY LCD:*/
	INIT_LCD_2x16(LCD_2X16);

	/*Inicializacion del ADC:*/
	INIT_ADC();

	/*Inicialización del TIM3:*/
	INIT_TIM3();
	SET_TIM3(timeBase, FS);

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
	while(1)
	{
		/*Refresco del LCD cada 200mseg:*/
		if (lcd == ticksLCD)
			LCD();
	}
}
/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
/*Interrupcion al vencimiento de cuenta de TIM3 cada 1/FS:*/
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {

		/*Se aumenta la variable para controlar el refresco del LCD:*/
		lcd++;

		/*Se toma una muestra en el ADC:*/
		ADC_PROCESSING();

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*------------------------------------------------------------------------------
TAREAS:
------------------------------------------------------------------------------*/
void ADC_PROCESSING(void)
{

}
void LCD(void)
{

}
