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
  	  * Tension AC  - PC0
  	  * Hall 		- PC3
********************************************************************************/

#include "functions.h"

/*------------------------------------------------------------------------------
DEFINICION PINES DEL HARDWARE:
------------------------------------------------------------------------------*/
/*Pines del ADC para medir tension - PC0:*/
#define _adcVoltPort	GPIOC
#define _adcVoltPin		GPIO_Pin_0

/*Pines del ADC para medir corriente - PC3:*/
#define _adcCurrPort	GPIOC
#define _adcCurrPin		GPIO_Pin_3

/*Pines del LCD:*/
LCD_2X16_t LCD_2X16[] = {
			// Name  , PORT ,   PIN      ,         CLOCK       ,   Init
			{ TLCD_RS, GPIOE, GPIO_Pin_2,  RCC_AHB1Periph_GPIOE, Bit_RESET },
			{ TLCD_E,  GPIOE, GPIO_Pin_4,  RCC_AHB1Periph_GPIOE, Bit_RESET },
			{ TLCD_D4, GPIOE, GPIO_Pin_5,  RCC_AHB1Periph_GPIOE, Bit_RESET },
			{ TLCD_D5, GPIOE, GPIO_Pin_6,  RCC_AHB1Periph_GPIOE, Bit_RESET },
			{ TLCD_D6, GPIOE, GPIO_Pin_3,  RCC_AHB1Periph_GPIOE, Bit_RESET },
			{ TLCD_D7, GPIOF, GPIO_Pin_8,  RCC_AHB1Periph_GPIOF, Bit_RESET }, };

/*------------------------------------------------------------------------------
DEFINICION DE VALORES PARA MAIN.C:
------------------------------------------------------------------------------*/
/*Base de tiempo para el TIM3:*/
#define timeBase 200e3 //[mseg]

/*Frecuencia de muestreo para configurar el TIM3:*/
#define FS  4000 //[Hz]

/*Frecuencia de interrupcion del systick:*/
#define systickIntTime 1 / (6*FS) //[seg]

/*Frecuencia base de la red:*/
#define baseFreq 50 //[Hz]

/*Maximo del buffer para el LCD:*/
#define lcdBufferLen 20

/*Voltaje maximo analogico:*/
#define maxVoltValue 622 //[Vpp]

/*Maximo voltaje del sensor efecto hall:*/
#define maxHallValue 3 //[V]

/*Resolucion del sensor de efecto hall:*/
#define hallRes 180e-3 //[mv/A]

/*Valor maximo digital al voltaje o corriente maximo:*/
#define maxDigValue 4095

/*Maximo almacenamiento en un ciclo:*/
#define maxSampling 6 * FS / baseFreq

/*Ingreso al clear del display cada 200mseg:*/
#define ticksLCD	500

/*Ingreso al procesamiento de datos del ADC cada 6 ciclos:*/
#define ticksADC 	26


/*------------------------------------------------------------------------------
DECLARACION FUNCIONES DE MAIN.C:
------------------------------------------------------------------------------*/
/*Procesamiento de datos del LCD:*/
void ADC_PROCESSING(void);

/*Manejo del LCD:*/
void LCD(void);

/*Calcular la potencia activa:*/
void P(void);

/*Calcular potencia aparente:*/
void S(void);

/*Calculo de la potencia reactiva:*/
void Q(void);

/*Calculo del coseno de theta:*/
void COS_THETA(void);

/*------------------------------------------------------------------------------
DECLARACION VARIABLES GLOBALES DE MAIN.C:
------------------------------------------------------------------------------*/
/*Control del refresco del LCD:*/
uint32_t lcd = 0;

/*Control de procesamiento de datos del LCD:*/
uint32_t adcProcessing = 0;

/*Registro para guardar los valores digitales del ADC:*/
uint32_t adcDigValues[maxSampling];

/*Variable para controlar el almacenamiento de datos de los ADC:*/
uint32_t instant = 0;

/*Variables de almacenamiento de datos del ADC en forma digital:*/
uint32_t voltValuesDig[maxSampling/2];
uint32_t currValuesDig[maxSampling/2];

/*Variables de almacenamiento de datos del ADC en forma analogica:*/
float 	 voltValuesAna[maxSampling/2];
float 	 currValuesAna[maxSampling/2];

/*Variable para almacenar la potencia activa:*/
float 	 activePow 	= 0.0f;

/*Variable para almacenar la potencia aparente:*/
float 	 apparentPow = 0.0f;

/*Variable para almacenar la potencia reactiva:*/
float 	 reactivePow = 0.0f;

/*Variable para almacenar el cos(theta):*/
float	 cosTheta 	= 0.0f;

int main(void)
{
/*------------------------------------------------------------------------------
CONFIGURACION DEL MICRO:
------------------------------------------------------------------------------*/
	SystemInit();

	/*Inicializacion del DISPLAY LCD:*/
	INIT_LCD_2x16(LCD_2X16);

	/*Inicializacion del ADC:*/
	INIT_ADC1DMA(adcDigValues, maxSampling);

	/*Inicialización del TIM3:*/
	INIT_TIM3(timeBase, FS);

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
	while(1)
	{
		/*Despachador de tareas:*/
		if (adcProcessing == ticksADC)
			ADC_PROCESSING();
		else if (lcd == ticksLCD)
			LCD();
	}
}
/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
//Interrupcion por tiempo - Systick cada 1mseg:
void SysTick_Handler()
{
	adcProcessing++;
	lcd++;
}

///*Interrupcion al vencimiento de cuenta de TIM3 cada 1/FS:*/
//void TIM3_IRQHandler(void) {
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
//
//		/*Se aumenta la variable para controlar el refresco del LCD:*/
//		lcd++;
//
//		/*Se toma una muestra en el ADC:*/
//		ADC_PROCESSING();
//
//		/*Actualizar flag TIM3:*/
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	}
//}

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
	PRINT_LCD_2x16(LCD_2X16, 8, 0, buffReactivePow);

	/*Calculo del cos(theta):*/
	COS_THETA();

	/*Mostrar cos(theta):*/
	sprintf(buffCosTheta, "fdp=%.1f", cosTheta);
	PRINT_LCD_2x16(LCD_2X16, 8, 1, buffCosTheta);

	/*Reseteo de las variables de calculo de potencia:*/
	activePow 	= 0.0f;
	apparentPow = 0.0f;
	reactivePow = 0.0f;
	cosTheta	= 0.0f;
}

/*Tomar muestras de tension y corriente mediante los ADC:*/
void ADC_PROCESSING(void)
{
	/*Control de ciclos:*/
	uint32_t i = 0;

	/*Pasaje de datos del arreglo general a los especificos:*/
	for (i = 0; i <= maxSampling ; i++)
	{
		/*Los datos de tension seran los pares:*/
		voltValuesDig[i] = adcDigValues[2*i];
		/*Los datos de corriente seran los impares:*/
		currValuesDig[i] = adcDigValues[(2*i)+1];

		/*Conversion y almacenamiento de dato de tension analogico de 0 a 310 Vpp:*/
		voltValuesAna[i] = (float)(voltValuesDig[i]*maxVoltValue/maxDigValue-maxVoltValue/2);
		/*Conversion y almacenamiento de corriente analogica:*/
		currValuesAna[i] = (float)((currValuesDig[i]*maxHallValue/maxDigValue)*hallRes);
	}
}

/*Calculo de la potencia activa:*/
void P(void)
{
	/*Variable para el conteo de ciclos:*/
	uint32_t i = 0;

	/*P es la sumatoria del producto de valores instantaneos de tension y corriente:*/
	for (i = 0; i < maxSampling; i++)
		activePow += (float) voltValuesAna[i]*currValuesAna[i];
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
		sumVoltElem += voltValuesAna[i]*voltValuesAna[i];
		sumCurrElem += currValuesAna[i]*currValuesAna[i];
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
