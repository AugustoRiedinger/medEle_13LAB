/********************************************************************************
  * @file    main.c
  * @author  A. Riedinger & G. Stang.
  * @version 0.1
  * @date    09-11-21.
  * @brief   Generacion de un filtro pasa altos digital FIR con n = 10 y
  	  	  	 fc = 2kHz segun una ventana tipo Hamming.

  * SALIDAS:
  	  *	LCD  Conexion Estandar TPs

  * ENTRADAS:
  	  * TEC-MAT
********************************************************************************/

#include "functions.h"

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

		/*Actualizar flag TIM3:*/
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*------------------------------------------------------------------------------
TAREAS:
------------------------------------------------------------------------------*/
/*Mostrar datos en el LCD:*/
void LCD(void)
{
	/*Reseteo de la variable del TS:*/
	lcd = 0;

	/*Calculo de la potencia activa:*/
	P();

	/*Calculo de la potencia aparente:*/
	S();

	/*Buffers para mostrar valores de variables:*/
	char buffActivePow[lcdBufferLen];
	char buffApparentPow[lcdBufferLen];

	/*Refresco del LCD:*/
	CLEAR_LCD_2x16(LCD_2X16);

	/*Mostrar potencia activa:*/
	sprintf(buffActivePow, "P=%.1f W", activePow);
	PRINT_LCD_2x16(LCD_2X16, 0, 0, buffActivePow);

	/*Mostrar potencia aparente:*/
	sprintf(buffApparentPow, "S=%.1f VA", apparentPow);
	PRINT_LCD_2x16(LCD_2X16, 0, 1, buffApparentPow);

	/*Reseteo de las variables de calculo de potencia:*/
	activePow = 0.0f;
	apparentPow = 0.0f;
}

/*Tomar muestras de tension y corriente mediante los ADC:*/
void ADC_PROCESSING(void)
{
	/*Almacenar dato de tension digital instantanea:*/
	voltValueDig[instant] = READ_ADC1();

	/*Conversion y almacenamiento de dato de tension analogico:*/
	voltValueAna[instant] = (float) voltValueDig[instant] * maxVoltValue / maxDigValue;

	/*Almacenar dato de corriente digital instantanea:*/
	currValueDig[instant] = READ_ADC2();

	/*TODO: BUSCAR EL VALOR MAXIMO DE CORRIENTE PARA LA CONVERSION.*/

	/*Conversion y almacenamiento de dato de corriente analogica:*/
	currValueAna[instant] = (float) currValueDig[instant] * maxCurrValue / maxDigValue;

	/*Control de la variable para almacenar datos instantaneos:*/
	/*Si esta en el maximo instante de muestreo se resetea:*/
	if (instant == maxSampling)
		instant = 0;
	/*Sino, se sigue aumentando:*/
	else
		instant++;
}

/*Calculo de la potencia activa:*/
void P(void)
{
	/*Variable para el conteo de ciclos:*/
	uint32_t i = 0;

	/*P es la sumatoria del producto de valores instantaneos de tension y corriente:*/
	for (i = 0; i < maxSampling; i++)
		activePow = (float) activePow + voltValueAna[i]*currValueAna[i];
}

/*Calculo de la potencia aparente:*/
void S(void)
{
	/*Variable para el conteo de ciclos:*/
	uint32_t i = 0;

	/*Modulo de la tension:*/
	float	 voltMod = 0.0f;

	/*Modulo de la corriente:*/
	float 	 currMod = 0.0f;

	/*TODO: BUSCAR COMO CALCULAR EL MODULO DE UN ARREGLO.*/

	/*S es el producto de los modulos de tension y corriente:*/
	for (i = 0; i < maxSampling; i++)
	{
		/*Calculo del modulo de la tension:*/
		voltMod = voltMod + sqrt(voltValueAna[i]*voltValueAna[i]);
		/*Calculo del modulo de la corriente:*/
		currMod = currMod + sqrt(voltValueAna[i]*currValueAna[i]);
		/*Calculo potencia aparente:*/
		apparentPow = apparentPow + voltMod*currMod;
	}
}
