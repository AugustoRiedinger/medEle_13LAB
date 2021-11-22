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
/*Frecuencia de muestreo para configurar el TIM3:*/
#define FS  5000 //[Hz]

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
#define maxSampling 10 * FS / baseFreq

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
/*Control del despachador de tareas:*/
uint32_t adc = 0;

/*Registro para guardar los valores digitales del ADC:*/
uint16_t adcDigValues[maxSampling];

/*Variables de almacenamiento de datos del ADC en forma digital:*/
uint16_t voltValuesDig[maxSampling/2];
uint16_t currValuesDig[maxSampling/2];

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
	INIT_TIM3(FS);

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
	while(1)
	{
		if(adc == 1)
			ADC_PROCESSING();
	}
}

/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
/*Interrupcion por Transfer Request del DMA:*/
void DMA2_Stream0_IRQHandler(void)
{
	/* transmission complete interrupt */
	if (DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0))
	{
		/*Se para el timer:*/
		TIM_Cmd(TIM3, DISABLE);

		/*Se habilita el flag para procesar los datos:*/
		adc = 1;

		/*Resetear el flag:*/
		DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
	}
}

/*------------------------------------------------------------------------------
TAREAS:
------------------------------------------------------------------------------*/
/*Procesamiento de los datos otorgados por el DMA:*/
void ADC_PROCESSING(void)
{
	/*Clarear el flag:*/
	adc = 0;

	/*Control de ciclos:*/
	uint32_t i = 0;

	/*Valores medios de los arreglos de tension y corriente:*/
	float	voltMed = 0.0f;
	float	currMed = 0.0f;

	/*Pasaje de datos del arreglo general a los especificos:*/
	for (i = 0; i < maxSampling/2; i++) {

		/*Los datos de tension seran los pares:*/
		voltValuesDig[i] = adcDigValues[2 * i];
		/*Los datos de corriente seran los impares:*/
		currValuesDig[i] = adcDigValues[(2 * i) + 1];

		/*Adaptacion de 0 a 3V:*/
		voltValuesAna[i] =  ((float) voltValuesDig[i] * 2.8 / 4095.0);
		currValuesAna[i] =  ((float) currValuesDig[i] * 2.8 / 4095.0);

		/*Multiplicacion para adaptar al valor real de tension:*/
		/*3 * x = 622 => x = 207*/
		voltValuesAna[i] = voltValuesAna[i] * 207;

		/*Multiplicacion para adaptar al valor real de corriente:*/
		/*180e-3 * 3 = 16*/
		currValuesAna[i] = currValuesAna[i] * 16;

		/*Calculo del valor medio de los arreglos:*/
		voltMed += voltValuesAna[i];
		currMed += currValuesAna[i];
	}

	voltMed = voltMed / (maxSampling/2);
	currMed = currMed / (maxSampling/2);

	/*Quitar valores medios; llevar a cero:*/
	for (i = 0; i < maxSampling/2 ; i++)
	{
		voltValuesAna[i] = voltValuesAna[i] - voltMed;
		currValuesAna[i] = currValuesAna[i] - currMed;
	}

	/*Mostrar los valores en el LCD:*/
	LCD();

	/*Se vuelve a iniciar el timer:*/
	DMA_Cmd(DMA2_Stream0, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
/*Mostrar datos en el LCD:*/
void LCD(void)
{
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
	sprintf(buffActivePow, "P=%.1fW", activePow);
	PRINT_LCD_2x16(LCD_2X16, 0, 0, buffActivePow);

	/*Calculo de la potencia aparente:*/
	S();

	/*Mostrar potencia aparente:*/
	sprintf(buffApparentPow, "S=%.1fVA", apparentPow);
	PRINT_LCD_2x16(LCD_2X16, 0, 1, buffApparentPow);

	/*Calculo de la potencia reactiva:*/
	Q();

	/*Mostrar potencia reactiva:*/
	sprintf(buffReactivePow, "Q=%.1fVAR", reactivePow);
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

/*Calculo de la potencia activa:*/
void P(void)
{
	/*Variable para el conteo de ciclos:*/
	uint32_t i = 0;

	/*P es la sumatoria del producto de valores instantaneos de tension y corriente:*/
	for (i = 0; i < maxSampling/2; i++)
		activePow += (float) voltValuesAna[i]*currValuesAna[i];

	activePow = activePow / (maxSampling/2);
}

/*Calculo de la potencia aparente:*/
void S(void)
{
	/*Variable para el conteo de ciclos:*/
	uint32_t i = 0;

	/*Modulo de la tension:*/
	float	 voltMod = 0.0f;
	float 	 sumVoltElem = 0.0f;

	/*Modulo de la corriente:*/
	float 	 currMod = 0.0f;
	float 	 sumCurrElem = 0.0f;

	/*Suma de los cuadrados de cada elemento de tension y corriente:*/
	for (i = 0; i < maxSampling/2; i++)
	{
		sumVoltElem += voltValuesAna[i]*voltValuesAna[i];
		sumCurrElem += currValuesAna[i]*currValuesAna[i];
	}

	/*Modulo de la tension:*/
	voltMod = sqrt(sumVoltElem/(maxSampling/2));

	/*Modulo de la corriente:*/
	currMod = sqrt(sumCurrElem/(maxSampling/2));

	/*S es el producto de los modulos de tension y corriente:*/
	apparentPow = voltMod * currMod;
}

/*Calculo de la potencia reactiva:*/
void Q(void)
{
	/*Q se calcula a partir del triangulo de potencias:*/
	/*S^2 = P^2 + Q^2*/
	reactivePow = sqrt((apparentPow*apparentPow) - (activePow*activePow));
}

/*Calculo del coseno de theta:*/
void COS_THETA(void)
{
	/*P = S * cos(theta):*/
	cosTheta = activePow / apparentPow;
}
