#include "functions.h"

/*------------------------------------------------------------------------------
DECLARACION DE FUNCIONES INTERNAS:
------------------------------------------------------------------------------*/
/*Encontrar el CLOCK de un determinado pin:*/
uint32_t FIND_CLOCK(GPIO_TypeDef* Port);

/*Control del ADC:*/
ADC_TypeDef* FIND_ADC_TYPE(GPIO_TypeDef* Port, uint32_t Pin);
uint32_t FIND_RCC_APB(ADC_TypeDef* ADCX);
uint8_t FIND_CHANNEL(GPIO_TypeDef* Port, uint32_t Pin);

/*Control del DAC:*/
uint32_t FIND_DAC_CHANNEL(GPIO_TypeDef* Port, uint32_t Pin);

/*****************************************************************************
INIT_DO

	* @author	A. Riedinger.
	* @brief	Inicialiiza salidas digitales.
	* @returns	void
	* @param
		- Port	Puerto del pin a inicializar. Ej: GPIOX.
		- Clock Clock del pin a inicializar. Ej: RCC_AHB1Periph_GPIOX.
		- Pin	Pin a inicializar. Ej: GPIO_Pin_X
	* @ej
		- INIT_DO(GPIOX, RCC_AHB1Periph_GPIOX, GPIO_Pin_X);
******************************************************************************/
void INIT_DO(GPIO_TypeDef* Port, uint32_t Pin)
{
	//Estructura de configuracion
	GPIO_InitTypeDef GPIO_InitStructure;

	//Habilitacion de la senal de reloj para el periferico:
	uint32_t Clock;
	Clock = FIND_CLOCK(Port);
	RCC_AHB1PeriphClockCmd(Clock, ENABLE);

	//Se configura el pin como entrada (GPI0_MODE_IN):
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	//Se aplica la configuracion definida anteriormente al puerto:
	GPIO_Init(Port, &GPIO_InitStructure);
}

/*****************************************************************************
INIT_ADC

	* @author	Catedra UTN-BHI TDII / A. Riedinger.
	* @brief	Inicializa una entrada analogica como ADC.
	* @returns	void
	* @param
		- Port		Puerto del ADC a inicializar. Ej: GPIOX.
		- Pin		Pin del ADC a inicializar. Ej: GPIO_Pin_X
	* @ej
		- INIT_ADC(GPIOX, GPIO_Pin_X);
******************************************************************************/
void INIT_ADC(GPIO_TypeDef* Port, uint16_t Pin)
{
	uint32_t Clock;
	Clock = FIND_CLOCK(Port);

	ADC_TypeDef* ADCX;
	ADCX = FIND_ADC_TYPE(Port, Pin);

	uint32_t RCC_APB;
	RCC_APB = FIND_RCC_APB(ADCX);

	uint8_t Channel;
	Channel = FIND_CHANNEL(Port, Pin);

    GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;

    //Habilitacion del Clock para el puerto donde esta conectado el ADC:
    RCC_AHB1PeriphClockCmd(Clock, ENABLE);

    //Configuracion del PIN del ADC como entrada ANALOGICA.
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = Pin;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL ;
    GPIO_Init(Port, &GPIO_InitStructure);

    //Activar ADC:
    RCC_APB2PeriphClockCmd(RCC_APB, ENABLE);

    //ADC Common Init:
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode                = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler           = ADC_Prescaler_Div4; // max 36 MHz
    ADC_CommonInitStructure.ADC_DMAAccessMode       = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay    = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    //ADC Init:
    ADC_StructInit (&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution             = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode           = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode     = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge   = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign              = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion        = 1;
    ADC_Init(ADCX, &ADC_InitStructure);

    //Establecer la configuración de conversion:
    ADC_InjectedSequencerLengthConfig(ADCX, 1);
    ADC_SetInjectedOffset(ADCX, ADC_InjectedChannel_1, 0);
    ADC_InjectedChannelConfig(ADCX, Channel, 1, ADC_SampleTime_480Cycles);

    /* Poner en marcha ADC ----------------------------------------------------*/
    ADC_Cmd(ADCX, ENABLE);
}

/*****************************************************************************
READ_ADC

	* @author	Catedra UTN-BHI TDII / A. Riedinger.
	* @brief	Prende un LED por un determinado tiempo y luego lo apaga.
	* @returns
		- ADC_DATA	Devuelve el valor DIGITAL de la lectura en el ADCX.
	* @param
		- ADCX		Nombre del ADC a inicializar. Ej: ADCX.
	* @ej
		- READ_ADC(ADCX);
******************************************************************************/
int32_t READ_ADC(GPIO_TypeDef* Port, uint16_t Pin)
{
    uint32_t ADC_DATA;

    ADC_TypeDef* ADCX;
    ADCX = FIND_ADC_TYPE(Port, Pin);

    ADC_ClearFlag(ADCX, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConv(ADCX);
    while (ADC_GetFlagStatus(ADCX, ADC_FLAG_JEOC) == RESET);

    ADC_DATA = ADC_GetInjectedConversionValue(ADCX, ADC_InjectedChannel_1);
    return ADC_DATA;
}

/*****************************************************************************
INIT_DAC_CONT
	* @author	A. Riedinger.
	* @brief	Inicializa una salida como DAC de continua.
	* @returns	void
	* @param
		- Port		Puerto del timer a inicializar. Ej: GPIOX.
		- Pin		Pin del LED. Ej: GPIO_Pin_X

	* @ej
		- INIT_DAC_CONT(GPIOX, GPIO_Pin_X); //Inicialización del Pin PXXX como DAC.
******************************************************************************/
void INIT_DAC_CONT(GPIO_TypeDef* Port, uint16_t Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	uint32_t Clock;
	Clock = FIND_CLOCK(Port);
	RCC_AHB1PeriphClockCmd(Clock, ENABLE);

	/* Configura el Pin como salida Analogica */
	GPIO_InitStructure.GPIO_Pin = Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Port, &GPIO_InitStructure);

	/* DAC: activar clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* DAC configuracion canal */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(FIND_DAC_CHANNEL(Port,Pin), &DAC_InitStructure);
	//**************************************************

	//DAC ON Channel 1
	DAC_Cmd(FIND_DAC_CHANNEL(Port,Pin), ENABLE);
}

/*****************************************************************************
DAC_CONT
	* @author	A. Riedinger.
	* @brief	Genera una señal DAC de continua.
	* @returns	void
	* @param
		- Port		Puerto del timer a inicializar. Ej: GPIOX.
		- Pin		Pin del LED. Ej: GPIO_Pin_X
		- Value		El valor de continua a generar en milivolts.

	* @ej
		- INIT_DAC_CONT(GPIOX, GPIO_Pin_X); //Inicialización del Pin PXXX como DAC.
******************************************************************************/
void DAC_CONT(GPIO_TypeDef* Port, uint16_t Pin, int16_t MiliVolts)
{
	DAC_SetChannel2Data(DAC_Align_12b_R, MiliVolts);
}


/*****************************************************************************
INIT_TIM3

	* @author	A. Riedinger.
	* @brief	Inicializa las interrupciones del TIM3.
	* @returns	void
	* @param
	* @ej
		- INIT_TIM4();
******************************************************************************/
void INIT_TIM3()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*****************************************************************************
SET_TIM3
	* @author	A. Riedinger.
	* @brief	Setea el TIM1 a una determinada frecuencia.
	* @returns	void
	* @param
		- Port		Puerto del timer a inicializar. Ej: GPIOX.
		- Pin		Pin del LED. Ej: GPIO_Pin_X

	* @ej
		- INIT_TIM4(GPIOX, GPIO_Pin_X); //Inicialización del Pin PXXX como TIMER4.
******************************************************************************/
void SET_TIM3(uint32_t TimeBase, uint32_t Freq)
{
	uint16_t PrescalerValue = 0;

	//Actualización de los valores del TIM4:
	SystemCoreClockUpdate();
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TimeBase) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TimeBase / Freq - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

/*------------------------------------------------------------------------------
 FUNCIONES INTERNAS:
------------------------------------------------------------------------------*/
uint32_t FIND_CLOCK(GPIO_TypeDef* Port)
{
	uint32_t Clock;

	if		(Port == GPIOA) Clock = RCC_AHB1Periph_GPIOA;
	else if (Port == GPIOB) Clock = RCC_AHB1Periph_GPIOB;
	else if (Port == GPIOC) Clock = RCC_AHB1Periph_GPIOC;
	else if (Port == GPIOD) Clock = RCC_AHB1Periph_GPIOD;
	else if (Port == GPIOE) Clock = RCC_AHB1Periph_GPIOE;
	else if (Port == GPIOF) Clock = RCC_AHB1Periph_GPIOF;
	else if (Port == GPIOG) Clock = RCC_AHB1Periph_GPIOG;
	return Clock;
}

ADC_TypeDef* FIND_ADC_TYPE (GPIO_TypeDef* Port, uint32_t Pin)
{
	ADC_TypeDef* ADCX;

	if 		((Port == GPIOA && (Pin == GPIO_Pin_0 || Pin == GPIO_Pin_1   || Pin == GPIO_Pin_2 ||
								Pin == GPIO_Pin_3 || Pin == GPIO_Pin_4   || Pin == GPIO_Pin_5 ||
								Pin == GPIO_Pin_6 || Pin == GPIO_Pin_7)) ||

			 (Port == GPIOB && (Pin == GPIO_Pin_0 || Pin == GPIO_Pin_1)) ||

			 (Port == GPIOC && (Pin == GPIO_Pin_0 || Pin == GPIO_Pin_1   || Pin == GPIO_Pin_2 ||
					  	  	  	Pin == GPIO_Pin_3 || Pin == GPIO_Pin_4   || Pin == GPIO_Pin_5)))
		ADCX = ADC1;

	else if ((Port == GPIOF && (Pin == GPIO_Pin_3 || Pin == GPIO_Pin_4   || Pin == GPIO_Pin_5 ||
								Pin == GPIO_Pin_6 || Pin == GPIO_Pin_7   || Pin == GPIO_Pin_8 ||
								Pin == GPIO_Pin_9 || Pin == GPIO_Pin_10)))
		ADCX = ADC3;

	else
		ADCX = NULL;

	return ADCX;
}

uint32_t FIND_RCC_APB(ADC_TypeDef* ADCX)
{
	uint32_t RCC_APB;

	if 		(ADCX == ADC1) RCC_APB = RCC_APB2Periph_ADC1;
	else if (ADCX == ADC3) RCC_APB = RCC_APB2Periph_ADC3;
	else 				   RCC_APB = 0;

	return RCC_APB;
}

uint8_t FIND_CHANNEL(GPIO_TypeDef* Port, uint32_t Pin)
{
	uint8_t Channel;

	if 		(Port == GPIOA && Pin == GPIO_Pin_0)  Channel = ADC_Channel_0;  else if (Port == GPIOA && Pin == GPIO_Pin_1)  Channel = ADC_Channel_1;
	else if (Port == GPIOA && Pin == GPIO_Pin_2)  Channel = ADC_Channel_2;  else if (Port == GPIOA && Pin == GPIO_Pin_3)  Channel = ADC_Channel_3;
	else if (Port == GPIOA && Pin == GPIO_Pin_4)  Channel = ADC_Channel_4;  else if (Port == GPIOA && Pin == GPIO_Pin_5)  Channel = ADC_Channel_5;
	else if (Port == GPIOA && Pin == GPIO_Pin_6)  Channel = ADC_Channel_6;  else if (Port == GPIOA && Pin == GPIO_Pin_7)  Channel = ADC_Channel_7;
	else if (Port == GPIOB && Pin == GPIO_Pin_0)  Channel = ADC_Channel_8;  else if (Port == GPIOB && Pin == GPIO_Pin_1)  Channel = ADC_Channel_9;
	else if (Port == GPIOC && Pin == GPIO_Pin_0)  Channel = ADC_Channel_10; else if (Port == GPIOC && Pin == GPIO_Pin_1)  Channel = ADC_Channel_11;
	else if (Port == GPIOC && Pin == GPIO_Pin_2)  Channel = ADC_Channel_12; else if (Port == GPIOC && Pin == GPIO_Pin_3)  Channel = ADC_Channel_13;
	else if (Port == GPIOC && Pin == GPIO_Pin_4)  Channel = ADC_Channel_14;	else if (Port == GPIOC && Pin == GPIO_Pin_5)  Channel = ADC_Channel_15;
	else if (Port == GPIOF && Pin == GPIO_Pin_3)  Channel = ADC_Channel_9;	else if (Port == GPIOF && Pin == GPIO_Pin_4)  Channel = ADC_Channel_14;
	else if (Port == GPIOF && Pin == GPIO_Pin_5)  Channel = ADC_Channel_15;	else if (Port == GPIOF && Pin == GPIO_Pin_6)  Channel = ADC_Channel_4;
	else if (Port == GPIOF && Pin == GPIO_Pin_7)  Channel = ADC_Channel_5;	else if (Port == GPIOF && Pin == GPIO_Pin_8)  Channel = ADC_Channel_6;
	else if (Port == GPIOF && Pin == GPIO_Pin_9)  Channel = ADC_Channel_7;	else if (Port == GPIOF && Pin == GPIO_Pin_10) Channel = ADC_Channel_8;
	else 										  Channel = 0;

	return Channel;
}

uint32_t FIND_DAC_CHANNEL(GPIO_TypeDef* Port, uint32_t Pin)
{
	if(Port == GPIOA && Pin == GPIO_Pin_5) return DAC_Channel_2;
	else return 0;
}
