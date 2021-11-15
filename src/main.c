/********************************************************************************
  * @file    main.c
  * @author  M. Ibacache, A. Riedinger & G. Stang.
  * @version 0.1
  * @date    10-11-21.
  * @brief 	 Medidor de potencia real, instantanea y aparente con DMA.

  * SALIDAS:
  	  *	LCD  Conexion Estandar TPs

  * ENTRADAS:
  	  * UserButton  - PC13
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

	/*Buffers para mostrar valores de variables:*/
	char buffActivePow	[lcdBufferLen];
	char buffApparentPow[lcdBufferLen];
	char buffReactivePow[lcdBufferLen];
	char buffCosTheta	[lcdBufferLen];

	/*Refresco del LCD:*/
	CLEAR_LCD_2x16(LCD_2X16);

	/*Calculo de la potencia activa:*/
	P();

	/*Mostrar potencia activa:*/
	sprintf(buffActivePow, "P=%.1f W", activePow);
	PRINT_LCD_2x16(LCD_2X16, 0, 0, buffActivePow);

	/*Calculo de la potencia aparente:*/
	S();

	/*Mostrar potencia aparente:*/
	sprintf(buffApparentPow, "S=%.1f VA", apparentPow);
	PRINT_LCD_2x16(LCD_2X16, 0, 1, buffApparentPow);

	/*Calculo de la potencia reactiva:*/
	Q();

	/*Mostrar potencia reactiva:*/
	sprintf(buffReactivePow, "Q=%.1f VAR", reactivePow);
	PRINT_LCD_2x16(LCD_2X16, 6, 0, buffReactivePow);

	/*Calculo del cos(theta):*/
	COS_THETA();

	/*Mostrar cos(theta):*/
	sprintf(buffCosTheta, "T=%.1f VAR", cosTheta);
	PRINT_LCD_2x16(LCD_2X16, 6, 1, buffCosTheta);

	/*Reseteo de las variables de calculo de potencia:*/
	activePow 	= 0.0f;
	apparentPow = 0.0f;
	reactivePow = 0.0f;
	cosTheta	= 0.0f;
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
	uint32_t k = 0;

	/*Modulo de la tension:*/
	float	 voltMod = 0.0f;
	float 	 sumVoltElem = 0.0f;

	/*Modulo de la corriente:*/
	float 	 currMod = 0.0f;
	float 	 sumCurrElem = 0.0f;

	/*Suma de los cuadrados de cada elemento de tension y corriente:*/
	for (i = 0; i < maxSampling; i++)
	{
		sumVoltElem = sumVoltElem + voltValueAna[i]*voltValueAna[i];
		sumCurrElem = sumCurrElem + currValueAna[i]*currValueAna[i];
	}

	/*Se calculan los modulos y la potencia reactiva:*/
	for(k = 0; k < maxSampling; k++)
	{
		/*Modulo de la tension:*/
		voltMod = sqrt(sumVoltElem);

		/*Modulo de la corriente:*/
		currMod = sqrt(sumCurrElem);

		/*S es el producto de los modulos de tension y corriente:*/
		apparentPow = voltMod*currMod;
	}
}

/*Calculo de la potencia reactiva:*/
void Q(void)
{
	/*Q se calcula a partir del triangulo de potencias:*/
	/*S^2 = P^2 + Q^2*/
	reactivePow = sqrt(apparentPow*apparentPow - activePow*activePow);
}

/*Calculo del coseno de theta:*/
void COS_THETA(void)
{
	/*P = S * cos(theta):*/
	cosTheta = activePow / apparentPow;
}
