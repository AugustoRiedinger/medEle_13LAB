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

/*Frecuencia base de la red:*/
#define baseFreq 50 //[Hz]

/*Voltaje maximo analogico:*/
#define maxVoltValue 340 //[Vpp]

/*Corriente maxima analogica:*/
#define maxCurrValue 180 //[mA]

/*Valor maximo digital al voltaje o corriente maximo:*/
#define maxDigValue 4095

/*Ticks del TS:*/
/*Se toma un dato del ADC en cada instante de muestreo:*/
#define ticksADC	1
/*Se clarea el display cada 200mseg:*/
#define ticksLCD	1000

/*Tareas del TS:*/
void ADC_PROCESSING(void);
void LCD(void);

/*Funcion para calcular la potencia activa:*/
void P(void);

/*------------------------------------------------------------------------------
VARIABLES GLOBALES:
------------------------------------------------------------------------------*/
/*Variables de manejo del TS:*/
uint32_t adc = 0;
uint32_t lcd = 0;

/*Variable para controlar el almacenamiento de datos de los ADC:*/
uint32_t instant = 0;

/*Maximo de almacenamiento en un ciclo:*/
uint32_t maxSampling = FS / baseFreq;

/*Variables de almacenamiento de datos del ADC en forma digital:*/
uint32_t voltValueDig[maxSampling];
uint32_t currValueDig[maxSampling];

/*Variables de almacenamiento de datos del ADC en forma analogica:*/
float 	 voltValueAna[maxSampling];
float 	 currValueAna[maxSampling];

/*Variable para almacenar la potencia activa:*/
float 	 activePow = 0.0f;

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
/*Tomar muestras de tension y corriente mediante los ADC:*/
void ADC_PROCESSING(void)
{
	/*Almacenar dato de tension digital instantanea:*/
	voltValueDig[instant] = READ_ADC1();

	/*Conversion y almacenamiento de dato de tension analogico:*/
	voltValueAna[instant] = (float) voltValueDig[instant] * maxVoltValue / maxDigValue;

	/*Almacenar dato de corriente digital instantanea:*/
	currValueDig[instant] = READ_ADC2();

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

/*Mostrar datos en el LCD:*/
void LCD(void)
{
	/*Reseteo de la variable del TS:*/
	lcd = 0;

	/*Calculo de la potencia activa:*/
	P();
}

/*Calculo de la potencia activa:*/
void P(void)
{

}
