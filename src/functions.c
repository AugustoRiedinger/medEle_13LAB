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

//LCD:
void P_LCD_2x16_InitIO(LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_PinLo(TLCD_NAME_t lcd_pin, LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_PinHi(TLCD_NAME_t lcd_pin, LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_Delay(volatile uint32_t nCount);
void P_LCD_2x16_InitSequenz(LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_Clk(LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_Cmd(uint8_t wert, LCD_2X16_t* LCD_2X16);
void P_LCD_2x16_Cursor(LCD_2X16_t* LCD_2X16, uint8_t x, uint8_t y);
void P_LCD_2x16_Data(uint8_t wert, LCD_2X16_t* LCD_2X16);


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
/*Inicializacion de dos ADC:*/
void INIT_ADC(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Puerto C -------------------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* PC1 para entrada analógica */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Activar ADC1 ----------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* Activar ADC2 ----------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	/* ADC Common Init -------------------------------------------------------*/
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // max 36 MHz segun datasheet
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC Init ---------------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);

	/* Establecer la configuración de conversión ------------------------------*/
	ADC_InjectedSequencerLengthConfig(ADC1, 1);
	ADC_SetInjectedOffset(ADC1, ADC_InjectedChannel_1, 0);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1,
			ADC_SampleTime_480Cycles);

	/* Establecer la configuración de conversión ------------------------------*/
	ADC_InjectedSequencerLengthConfig(ADC2, 1);
	ADC_SetInjectedOffset(ADC2, ADC_InjectedChannel_1, 0);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_13, 1,
			ADC_SampleTime_480Cycles);

	/* Poner en marcha ADC ----------------------------------------------------*/
	ADC_Cmd(ADC1, ENABLE);

	/* Poner en marcha ADC ----------------------------------------------------*/
	ADC_Cmd(ADC2, ENABLE);
}

/*Lectura del ADC1:*/
int32_t READ_ADC1(void) {

	uint32_t valor_adc;

	ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);      // borrar flag de fin conversion

	ADC_SoftwareStartInjectedConv(ADC1);    // iniciar conversion

	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET)
		; // Espera fin de conversion

	valor_adc = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1); // obtiene Valor A-D

	return valor_adc;
}

/*Lectura del ADC2:*/
int32_t READ_ADC2(void) {

	uint32_t valor_adc;

	ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);      // borrar flag de fin conversion

	ADC_SoftwareStartInjectedConv(ADC2);    // iniciar conversion

	while (ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == RESET)
		; // Espera fin de conversion

	valor_adc = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1); // obtiene Valor A-D

	return valor_adc;
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

/*****************************************************************************
INIT_LCD_2x16

	* @author	A. Riedinger.
	* @brief	Inicializa los pines del display.
	* @returns	void
	* @param
		- LCD_2x16	Arreglo tipo LCD_2X16_t con los pines del LCD. Ej:
					LCD_2X16_t LCD_2X16[] = {
 	 	 	 	 	// Name  , PORT,    PIN     ,          CLOCK     , Init
  	  	  	  	  	{TLCD_RS ,GPIOC,GPIO_Pin_10 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_E  ,GPIOC,GPIO_Pin_11 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D4 ,GPIOC,GPIO_Pin_12 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D5 ,GPIOD,GPIO_Pin_2  ,RCC_AHB1Periph_GPIOD,Bit_RESET},
  	  	  	  	  	{TLCD_D6 ,GPIOF,GPIO_Pin_9  ,RCC_AHB1Periph_GPIOF,Bit_RESET},
  	  	  	  	  	{TLCD_D7 ,GPIOF,GPIO_Pin_7  ,RCC_AHB1Periph_GPIOF,Bit_RESET},};
	* @ej
		- INIT_LCD_2x16(LCD_2X16);
******************************************************************************/
void INIT_LCD_2x16(LCD_2X16_t* LCD_2X16)
{
	  //Inicialización de los pines del LCD:
	  P_LCD_2x16_InitIO(LCD_2X16);
	  // kleine Pause
	  P_LCD_2x16_Delay(TLCD_INIT_PAUSE);
	  // Init Sequenz starten
	  P_LCD_2x16_InitSequenz(LCD_2X16);
	  // LCD-Settings einstellen
	  P_LCD_2x16_Cmd(TLCD_CMD_INIT_DISPLAY, LCD_2X16);
	  P_LCD_2x16_Cmd(TLCD_CMD_ENTRY_MODE, LCD_2X16);
	  // Display einschalten
	  P_LCD_2x16_Cmd(TLCD_CMD_DISP_M1, LCD_2X16);
	  // Display l�schen
	  P_LCD_2x16_Cmd(TLCD_CMD_CLEAR, LCD_2X16);
	  // kleine Pause
	  P_LCD_2x16_Delay(TLCD_PAUSE);
}



/*****************************************************************************
CLEAR_LCD_2x16

	* @author	A. Riedinger.
	* @brief	Refresca la pantalla del LCD.
	* @returns	void
	* @param
		- LCD_2x16	Arreglo tipo LCD_2X16_t con los pines del LCD. Ej:
					LCD_2X16_t LCD_2X16[] = {
 	 	 	 	 	// Name  , PORT,    PIN     ,          CLOCK     , Init
  	  	  	  	  	{TLCD_RS ,GPIOC,GPIO_Pin_10 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_E  ,GPIOC,GPIO_Pin_11 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D4 ,GPIOC,GPIO_Pin_12 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D5 ,GPIOD,GPIO_Pin_2  ,RCC_AHB1Periph_GPIOD,Bit_RESET},
  	  	  	  	  	{TLCD_D6 ,GPIOF,GPIO_Pin_9  ,RCC_AHB1Periph_GPIOF,Bit_RESET},
  	  	  	  	  	{TLCD_D7 ,GPIOF,GPIO_Pin_7  ,RCC_AHB1Periph_GPIOF,Bit_RESET},};
	* @ej
		- UB_LCD_2x16_Clear(LCD_2X16);
******************************************************************************/
void CLEAR_LCD_2x16(LCD_2X16_t* LCD_2X16)
{
  // Display l�schen
  P_LCD_2x16_Cmd(TLCD_CMD_CLEAR, LCD_2X16);
  // kleine Pause
  P_LCD_2x16_Delay(TLCD_PAUSE);
}



/*****************************************************************************
PRINT_LCD_2x16

	* @author	A. Riedinger.
	* @brief	Imprime una string en el LCD.
	* @returns	void
	* @param
		- LCD_2x16	Arreglo tipo LCD_2X16_t con los pines del LCD. Ej:
					LCD_2X16_t LCD_2X16[] = {
 	 	 	 	 	// Name  , PORT,    PIN     ,          CLOCK     , Init
  	  	  	  	  	{TLCD_RS ,GPIOC,GPIO_Pin_10 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_E  ,GPIOC,GPIO_Pin_11 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D4 ,GPIOC,GPIO_Pin_12 ,RCC_AHB1Periph_GPIOC,Bit_RESET},
  	  	  	  	  	{TLCD_D5 ,GPIOD,GPIO_Pin_2  ,RCC_AHB1Periph_GPIOD,Bit_RESET},
  	  	  	  	  	{TLCD_D6 ,GPIOF,GPIO_Pin_9  ,RCC_AHB1Periph_GPIOF,Bit_RESET},
  	  	  	  	  	{TLCD_D7 ,GPIOF,GPIO_Pin_7  ,RCC_AHB1Periph_GPIOF,Bit_RESET},};
  	  	- x			Indicador de fila. Ej: 0 ... 1.
  	  	- y 		Indicador de columna. Ej: 0 .. 16.
  	  	- ptr		Puntero a la string a imprimir.

	* @ej
		- PRINT_LCD_2x16(LCD_2X16, 0, 0, STR);
******************************************************************************/
void PRINT_LCD_2x16(LCD_2X16_t* LCD_2X16, uint8_t x, uint8_t y, char *ptr)
{
  // Cursor setzen
  P_LCD_2x16_Cursor(LCD_2X16,x,y);
  // kompletten String ausgeben
  while (*ptr != 0) {
    P_LCD_2x16_Data(*ptr, LCD_2X16);
    ptr++;
  }
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

//LCD:
void P_LCD_2x16_InitIO(LCD_2X16_t* LCD_2X16)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TLCD_NAME_t lcd_pin;

	for (lcd_pin = 0; lcd_pin < TLCD_ANZ; lcd_pin++)
	{
		//Habilitacion del Clock para cada PIN:
		RCC_AHB1PeriphClockCmd(LCD_2X16[lcd_pin].TLCD_CLK, ENABLE);

		//Configuracion como salidas digitales:
		GPIO_InitStructure.GPIO_Pin = LCD_2X16[lcd_pin].TLCD_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(LCD_2X16[lcd_pin].TLCD_PORT, &GPIO_InitStructure);

		//Default Wert einstellen
		if(LCD_2X16[lcd_pin].TLCD_INIT == Bit_RESET)
			P_LCD_2x16_PinLo(lcd_pin, LCD_2X16);
		else
			P_LCD_2x16_PinHi(lcd_pin, LCD_2X16);
	}
}

void P_LCD_2x16_PinLo(TLCD_NAME_t lcd_pin, LCD_2X16_t* LCD_2X16)
{
  LCD_2X16[lcd_pin].TLCD_PORT->BSRRH = LCD_2X16[lcd_pin].TLCD_PIN;
}

void P_LCD_2x16_PinHi(TLCD_NAME_t lcd_pin, LCD_2X16_t* LCD_2X16)
{
  LCD_2X16[lcd_pin].TLCD_PORT->BSRRL = LCD_2X16[lcd_pin].TLCD_PIN;
}

void P_LCD_2x16_Delay(volatile uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void P_LCD_2x16_InitSequenz(LCD_2X16_t* LCD_2X16)
{
  //Inicializacion de la secuencia:
  P_LCD_2x16_PinHi(TLCD_D4, LCD_2X16);
  P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16);
  P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  // Erster Init Impuls
  P_LCD_2x16_Clk(LCD_2X16);
  P_LCD_2x16_Delay(TLCD_PAUSE);
  // Zweiter Init Impuls
  P_LCD_2x16_Clk(LCD_2X16);
  P_LCD_2x16_Delay(TLCD_PAUSE);
  // Dritter Init Impuls
  P_LCD_2x16_Clk(LCD_2X16);
  P_LCD_2x16_Delay(TLCD_PAUSE);
  // LCD-Modus einstellen (4Bit-Mode)
  P_LCD_2x16_PinLo(TLCD_D4, LCD_2X16);
  P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16);
  P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  P_LCD_2x16_Clk(LCD_2X16);
  P_LCD_2x16_Delay(TLCD_PAUSE);
}

void P_LCD_2x16_Clk(LCD_2X16_t* LCD_2X16)
{
  // Pin-E auf Hi
  P_LCD_2x16_PinHi(TLCD_E, LCD_2X16);
  // kleine Pause
  P_LCD_2x16_Delay(TLCD_CLK_PAUSE);
  // Pin-E auf Lo
  P_LCD_2x16_PinLo(TLCD_E, LCD_2X16);
  // kleine Pause
  P_LCD_2x16_Delay(TLCD_CLK_PAUSE);
}

void P_LCD_2x16_Cmd(uint8_t wert, LCD_2X16_t* LCD_2X16)
{
  // RS=Lo (Command)
  P_LCD_2x16_PinLo(TLCD_RS, LCD_2X16);
  // Hi-Nibble ausgeben
  if((wert&0x80)!=0) P_LCD_2x16_PinHi(TLCD_D7, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  if((wert&0x40)!=0) P_LCD_2x16_PinHi(TLCD_D6, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  if((wert&0x20)!=0) P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D5, LCD_2X16);
  if((wert&0x10)!=0) P_LCD_2x16_PinHi(TLCD_D4, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D4, LCD_2X16);
  P_LCD_2x16_Clk(LCD_2X16);
  // Lo-Nibble ausgeben
  if((wert&0x08)!=0) P_LCD_2x16_PinHi(TLCD_D7, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  if((wert&0x04)!=0) P_LCD_2x16_PinHi(TLCD_D6, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  if((wert&0x02)!=0) P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D5, LCD_2X16);
  if((wert&0x01)!=0) P_LCD_2x16_PinHi(TLCD_D4, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D4, LCD_2X16);
  P_LCD_2x16_Clk(LCD_2X16);
}

void P_LCD_2x16_Cursor(LCD_2X16_t* LCD_2X16, uint8_t x, uint8_t y)
{
  uint8_t wert;

  if(x>=TLCD_MAXX) x=0;
  if(y>=TLCD_MAXY) y=0;

  wert=(y<<6);
  wert|=x;
  wert|=0x80;
  P_LCD_2x16_Cmd(wert,LCD_2X16);
}

void P_LCD_2x16_Data(uint8_t wert, LCD_2X16_t* LCD_2X16)
{
  // RS=Hi (Data)
  P_LCD_2x16_PinHi(TLCD_RS, LCD_2X16);
  // Hi-Nibble ausgeben
  if((wert&0x80)!=0) P_LCD_2x16_PinHi(TLCD_D7, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  if((wert&0x40)!=0) P_LCD_2x16_PinHi(TLCD_D6, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  if((wert&0x20)!=0) P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D5, LCD_2X16);
  if((wert&0x10)!=0) P_LCD_2x16_PinHi(TLCD_D4, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D4, LCD_2X16);
  P_LCD_2x16_Clk(LCD_2X16);
  // Lo-Nibble ausgeben
  if((wert&0x08)!=0) P_LCD_2x16_PinHi(TLCD_D7, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D7, LCD_2X16);
  if((wert&0x04)!=0) P_LCD_2x16_PinHi(TLCD_D6, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D6, LCD_2X16);
  if((wert&0x02)!=0) P_LCD_2x16_PinHi(TLCD_D5, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D5, LCD_2X16);
  if((wert&0x01)!=0) P_LCD_2x16_PinHi(TLCD_D4, LCD_2X16); else P_LCD_2x16_PinLo(TLCD_D4, LCD_2X16);
  P_LCD_2x16_Clk(LCD_2X16);
}

