/**
*****************************************************************************
**
**  Cookie - wersja performance 1.01.b
**  by Mateo de Jo Vinco and Marcino el Banano
**
*****************************************************************************
*/

/* Includes */

#include "stm32f10x.h"

//stan wejæia IR
#define IR_Input ((GPIOB->IDR & GPIO_Pin_5)?(1):(0))

#define IR_BIT_HALF 45
#define IR_BIT 90
#define IR_Tolerance 10

//konwersja liczby do cyfry heksadecymalnej
#define HEX(x) ((x >= 15)?(0x46):((x >= 10)?(0x41 + (x - 10)):(0x30 + x)))

//definicje kodów poszczególnych klawiszy
#define RC5_Code_0 0x0000
#define RC5_Code_1 0x000C
#define RC5_Code_2 0x0002
#define RC5_Code_3 0x0003
#define RC5_Code_4 0x0004
#define RC5_Code_5 0x0005
#define RC5_Code_6 0x0006
#define RC5_Code_7 0x0007
#define RC5_Code_8 0x0008
#define RC5_Code_9 0x0009

/* Private typedef */

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
ADC_InitTypeDef ADC_InitStruct;

/* Private define  */



#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define SampleTime ADC_SampleTime_7Cycles5


/* Private macro */

/* Private variables */
/* signed 16bit int -32 768 to +32 767  */
__IO int16_t P, D, error, previous_error, uchyb, poprzedni_uchyb, PD_value, motor_l, motor_p;

__IO int16_t W[10]={-15,-10,-6,-4,-2,2,4,6,10,15};
int16_t Kp=300;
int16_t Kd=1500;
int16_t Ki=0;
__IO int16_t motor=1600
		;

__IO int16_t motor_1=1600;
__IO int16_t motor_2=1800;
__IO int16_t motor_3=1600;
__IO int16_t motor_initial=1000;
__IO int16_t rdyfzm=000;

__IO uint8_t dyf=1; //dyferencja³ na zewnêtrznym kole
__IO int16_t rdyf=000;

//Tablica wyboru:
int16_t liczba[5]={0,0,0,0,0};
int16_t ilosc=0;
int16_t licz=0;
int16_t zmienna=0;
int16_t tryb=0;

/* unsigned 16bit int 0 to +65 535 */
__IO uint16_t WYPP=0;
__IO uint16_t WYPL=0;
__IO uint16_t ADCVal[10];
__IO uint16_t Treshold=300;

//zmienne stanu
__IO uint8_t start=0;
__IO uint8_t mode=0;
__IO uint8_t speed=3;
__IO uint8_t Kp_step=3;
__IO uint8_t Kd_step=3;
__IO uint8_t tresh=3;
__IO uint8_t cz=0;

//zmienne pomocnicze
__IO uint8_t i=0;
__IO int16_t k=0;
__IO int16_t e=0;

uint16_t licznik1 = 0;
uint8_t stan = 0;
uint16_t rc5 = 0;
uint16_t rc5_code = 0;
uint8_t rc5_ok = 0;
uint8_t rc5_toogle = 2;
uint8_t rc5_toogle_last = 2;
uint8_t temp = 0;
int czas=3;
/* Private function prototypes */

/* Private functions */

void RCC_Conf(void);
void NVIC_Conf(void);
void EXTI_Conf(void);
void GPIO_Conf(void);
void DMA_Conf(void);
void PWM_Init(void);
void ADC_On(void);
void TIM2_IRQHandler(void);

void error_calc(void);
void PD_calc(void);
void straight(void);
void start_stop(void);
void czujnik_check(void);

void speed_up();
void speed_down();
void Kp_up();
void Kp_down();
void Kd_up();
void Kd_down();
void tresh_up();
void tresh_down();

void delay_ms(int ms);
void menu();
void sterowanie();
int wartosc = 2500;
double szyfr=0;
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{




	RCC_Conf();
	NVIC_Conf();
	FLASH_Unlock();
	EXTI_Conf();
	GPIO_Conf();
	PWM_Init();
	DMA_Conf();
	ADC_On();


	//LED 1 i 4
	GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);

	//SILNIKI W SPOCZYNKU
	GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	GPIO_SetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_6);
	GPIO_ResetBits(GPIOC,GPIO_Pin_7|GPIO_Pin_9);

	PD_value=0;
	previous_error=0;
	uchyb=0;
	poprzedni_uchyb=0;

  /* Infinite loop */
  while (1)
  {
	  if(mode==4||mode==5) czujnik_check();
	  error_calc();
	  menu();
	  //sterowanie();
  }
}


/* Functions */
void DMA_Conf(void)
{
	DMA_InitTypeDef DMA_InitStruct;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr = (u32)&ADCVal;
	// Kierunek: zrodlem jest ADC
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;

	// Rozmiar burora: dwa kanaly = rozmiar bufora 2
	DMA_InitStruct.DMA_BufferSize = 10;

	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	// Dane beda przesylane ciagle
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;

	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	// Wlacz DMA
	DMA_Cmd(DMA1_Channel1, ENABLE);
}
void GPIO_Conf(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    /* LED */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* SWITCH */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* RC5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* SILNIKI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9
                            |GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* PWM */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15 );
}
void RCC_Conf(void)
{
  ErrorStatus HSEStartUpStatus;

  // Reset ustawien RCC
  RCC_DeInit();

  // Wlacz HSE
  RCC_HSEConfig(RCC_HSE_ON);

  // Czekaj za HSE bedzie gotowy
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // zwloka dla pamieci Flash
    FLASH_SetLatency(FLASH_Latency_2);

    // HCLK = SYSCLK
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    // PCLK2 = HCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);

    // PCLK1 = HCLK/2
    RCC_PCLK1Config(RCC_HCLK_Div2);

    // ADCCLK = PCLK2/6
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    // PLLCLK = 8MHz * 9 = 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    // Wlacz PLL
    RCC_PLLCmd(ENABLE);

    // Czekaj az PLL poprawnie sie uruchomi
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    // PLL bedzie zrodlem sygnalu zegarowego
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Czekaj az PLL bedzie sygnalem zegarowym systemu
    while(RCC_GetSYSCLKSource() != 0x08);

	// Wlacz sygnal zegarowy dla ADC1, GPIOC i funkcji alternatywnych
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Wlacz DMA
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  	// SysTick bedzie taktowany z f = 72MHz/8 = 9MHz
  	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	// Przerwanie ma byc co 1ms, f = 9MHz / 1000, czyli liczy od 9000
	if (SysTick_Config(90000*czas))
	{
		// W razie bledu petla nieskonczona
		while(1);
	}
  }
}
void ADC_On(void)
{
	// Jeden przetwornik, pracujacy niezaleznie
		ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
		// Pomiar jednego kanalu, wylacz opcje skanowania
		ADC_InitStruct.ADC_ScanConvMode = ENABLE;
		// Wlacz pomiar w trybie ciaglym
		ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
		// Nie bedzie wyzwalania zewnetrznego
		ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		// Dane wyrownane do prawej - znaczacych bedzie 12 mlodszych bitow
		ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
		// Dwa kanaly
		ADC_InitStruct.ADC_NbrOfChannel = 10;
		// Inicjuj przetwornik
		ADC_Init(ADC1, &ADC_InitStruct);

		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 10, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 9, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 8, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, SampleTime);


		// Wlaczenie DMA
		ADC_DMACmd(ADC1, ENABLE);
		// Wlacz ADC1
		ADC_Cmd(ADC1, ENABLE);

		// Resetuj rejestry kalibracyjne
		ADC_ResetCalibration(ADC1);
		// Czekaj, az skonczy resetowac
		while(ADC_GetResetCalibrationStatus(ADC1));

		// Start kalibracji ADC1
		ADC_StartCalibration(ADC1);
		// Czekaj na zakonczenie kalibracji ADC1
		while(ADC_GetCalibrationStatus(ADC1));


		// Start przetwarzania
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void PWM_Init(void)
{
	// Ustawienia ukladu podstawy czasu
	TIM_TimeBaseStructure.TIM_Period = 3000; //fpwm = 72MHz/3000 = 24kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);



	// Konfiguracja kanalu 2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);

	// Konfiguracja kanalu 3
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);

	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 1399;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	TIM_Cmd(TIM2, ENABLE);

}
void SysTick_Handler(void)
{
	PD_calc();
	straight();

	/*motor_l = motor_1+PD_value;
	motor_p = motor_1-PD_value;
	if (motor_l>(dyf*motor_1+rdyf)) motor_l=dyf*motor_1+rdyf;
	if (motor_l<0) motor_l=0;
	if (motor_p>(dyf*motor_1+rdyf)) motor_p=dyf*motor_1+rdyf;
	if (motor_p<0) motor_p=0;*/


	motor_l = motor_initial+PD_value;
	motor_p = motor_initial-PD_value;

	if (motor_l>(dyf*motor_initial+rdyf)) motor_l=dyf*motor_initial+rdyf;
	if (motor_l<0) motor_l=0;
	if (motor_p>(dyf*motor_initial+rdyf)) motor_p=dyf*motor_initial+rdyf;
	if (motor_p<0) motor_p=0;



	WYPL=motor_l;
	WYPP=motor_p;
	TIM1->CCR2 = WYPL;//4/3
	TIM1->CCR3 = WYPP;
}
void error_calc(void)
{
	i=0;
	error=0;
	if(ADCVal[0]>Treshold)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[0];
		i++;
		//e=-25;
		e=-35;
		k=150;
	}
	if(ADCVal[1]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[1];
		i++;
		k=150;
	}
	if(ADCVal[2]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[2];
		i++;
		k=150;
	}
	if(ADCVal[3]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[3];
		i++;
	}
	if(ADCVal[4]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[4];
		i++;
	}
	if(ADCVal[5]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[5];
		i++;
	}
	if(ADCVal[6]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[6];
		i++;
	}
	if(ADCVal[7]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[7];
		i++;
		k=150;
	}
	if(ADCVal[8]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[8];
		i++;
		k=150;
	}
	if(ADCVal[9]>Treshold)
	{	GPIO_SetBits(GPIOC,GPIO_Pin_10);
		error+=W[9];
		i++;
		//e=25;
		e=35;
		k=150;
	}

	if(i>2)
	{
		//error = 5*error;
		//motor_1=motor-200;
	}
	else if(i==0)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	if(e>0)
		error=e-15;
	else error=e+15;

		//motor_1=motor-100;
	}
	else
	{	//motor_1=motor;
		error = error/i;
	}
	/*if(i==0)
		{
			if (previous_error>5) error = 12;
			else if (previous_error<-5) error = -12;
			else error = previous_error;

			//error=previous_error;
			GPIO_ResetBits(GPIOA,GPIO_Pin_12);
		}
		else
		{
			error = error/i;
			GPIO_SetBits(GPIOA,GPIO_Pin_12);
		}*/
	//if (error<-7 || error > 7) rdyf=rdyfzm;
	//else rdyf=0;
	if(start==1 || mode==4)
	{
		/*if(error<-10)GPIO_SetBits(GPIOA,GPIO_Pin_11);
		if(error<0){GPIO_SetBits(GPIOA,GPIO_Pin_10);GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);}
		if(error==0)GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
		if(error>0){GPIO_SetBits(GPIOA,GPIO_Pin_9);GPIO_ResetBits(GPIOA,GPIO_Pin_10|GPIO_Pin_11);}
		if(error>10)GPIO_SetBits(GPIOA,GPIO_Pin_8);*/
	}
	uchyb=error;
	previous_error=error;


}
void PD_calc(void)
{
	P = uchyb;

	D = uchyb - poprzedni_uchyb;

	PD_value = (Kp*P) + (Kd*D);
	poprzedni_uchyb=uchyb;
}
void straight(void)
{


	if(ADCVal[4]>Treshold)
	{
		k=k-1;
	}
	if(ADCVal[5]>Treshold)
	{
		k=k-1;
	}
	if(k<-30000)k=30000;
	if (k<50) motor_initial=motor_2;
	else motor_initial=motor_1;
}
void czujnik_check(void)
{
	if(ADCVal[cz]>Treshold) GPIO_SetBits(GPIOA,GPIO_Pin_12);
	else GPIO_ResetBits(GPIOA,GPIO_Pin_12);
}
void NVIC_Conf(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
	  // Jezeli tablica wektorow w RAM, to ustaw jej adres na 0x20000000
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
	  // W przeciwnym wypadku ustaw na 0x08000000
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif


	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Wlacz przerwanie od TIM2
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);




}
void EXTI_Conf(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

  	// Poinformowanie uC o zrodle przerwania
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

  	// SWITCH START
  	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	// SWITCH DOWN
  	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	// SWITCH MODE
  	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	// SWITCH UP
  	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Enable EXTIx to detect the start bit of the RC5 frame */
  	  EXTI_ClearITPendingBit(EXTI_Line5);
  	  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	  EXTI_Init(&EXTI_InitStructure);


}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
	{
	start_stop();
	EXTI_ClearITPendingBit(EXTI_Line8);
	}


	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		//zapis stanu licznika oraz jego wyzerowanie
		licznik1 = TIM_GetCounter(TIM2);
		TIM_SetCounter(TIM2, 0);

		//odbioru pierwszego bitu
		if((stan == 0) && (IR_Input == 0))
		{
			TIM_SetCounter(TIM2, 0);
			TIM_Cmd(TIM2, ENABLE);

			stan = 1;
			rc5 = 0;
			rc5_ok = 0;
		}
		else if(stan > 0)
		{
			//aktualizacja stanu w którym sie znajduje dekodowanie rc5
			//wraz ze sprawdzaniem czy czas trwania impulsu jest poprawny
			if((licznik1 > (75)) && (licznik1 < (105)))
				{stan += 2;}
			else if((licznik1 > (35)) && (licznik1 < (55)))
				{stan += 1;}
			//w przypadku b³êdnego czasu trwania stanu niskiego lub wysokiego nastêpuje
			//wyzerowanie zmiennych oraz zablokowanie dekodowania sygna³u na okres³ony czas
			else
			{
				//wy³¹czenie licznika
				TIM_Cmd(TIM2, DISABLE);
				TIM_SetCounter(TIM2, 0);
				//b³êdne zakoñczenie odbioru ramki kodu RC5
				stan = 0;
				rc5 = 0;
				rc5_ok = 0;
			}
		}
		else
		{
			//wy³¹czenie licznika
			TIM_Cmd(TIM2, DISABLE);
			TIM_SetCounter(TIM2, 0);
			//b³êdne zakoñczenie odbioru ramki kodu RC5
			stan = 0;
			rc5 = 0;
			rc5_ok = 0;
		}
		switch(stan)
		{
			case 27 :
				//ostatnie próbkowanie po zboczu synchronizacji
				if(IR_Input == 0)
					{rc5 |= 0x01;}
				//wy³¹czenie licznika
				TIM_Cmd(TIM2, DISABLE);
				TIM_SetCounter(TIM2, 0);
				//udan zakoñczenie odbioru ramki kodu RC5
				rc5_ok = 1;
				stan = 0;
			break;
			//próbkowanie stanów chwile po zboczu synchronizacji
			case 25 : if(IR_Input == 0){rc5 |= 0x0002;} break;
			case 23 : if(IR_Input == 0){rc5 |= 0x0004;} break;
			case 21 : if(IR_Input == 0){rc5 |= 0x0008;} break;
			case 19 : if(IR_Input == 0){rc5 |= 0x0010;} break;
			case 17 : if(IR_Input == 0){rc5 |= 0x0020;} break;
			case 15 : if(IR_Input == 0){rc5 |= 0x0040;} break;
			case 13 : if(IR_Input == 0){rc5 |= 0x0080;} break;
			case 11 : if(IR_Input == 0){rc5 |= 0x0100;} break;
			case 9 : if(IR_Input == 0){rc5 |= 0x0200;} break;
			case 7 : if(IR_Input == 0){rc5 |= 0x0400;} break;
			case 5 : if(IR_Input == 0){rc5 |= 0x0800;} break;
			case 3 : if(IR_Input == 0){rc5 |= 0x1000;} break;
			case 1 : if(IR_Input == 0){rc5 |= 0x2000;} break;
			default : break;
		}
		//wyzerowanie flagi przerwania
		EXTI_ClearITPendingBit(EXTI_Line5);
	}

}
void EXTI15_10_IRQHandler(void)
{
	/* SWITCH MODE */
	if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
		if(mode==0)
		{
			mode=1;
			GPIO_SetBits(GPIOA,GPIO_Pin_8);
		}
		else if(mode==1)
		{
			mode=2;
			GPIO_SetBits(GPIOA,GPIO_Pin_9);
		}
		else if(mode==2)
		{
			mode=3;
			GPIO_SetBits(GPIOA,GPIO_Pin_10);
		}
		else if(mode==3)
		{
			mode=4;
			GPIO_SetBits(GPIOA,GPIO_Pin_11);
		}
		else if(mode==4)
		{
			mode=5;
		}
		else if(mode==5)
		{
			mode=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
		}

		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	/* SWITCH UP */
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
	{
		if (mode==1) speed_up();
		else if(mode==2) Kp_up();
		else if(mode==3) Kd_up();
		else if(mode==4) //czujnik_check
		{
			if (cz<13) cz=cz+1;
		}
		else if(mode==5) tresh_up();

		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	/* SWITCH DOWN */
	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
	{
		if (mode==1) speed_down();
		else if(mode==2) Kp_down();
		else if(mode==3) Kd_down();
		else if(mode==4) //czujnik_check
		{
			if (cz>0) cz=cz-1;
		}
		else if(mode==5) tresh_down();

		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_Cmd(TIM2, DISABLE);
		TIM_SetCounter(TIM2, 0);

		rc5 = 0;
		rc5_ok = 0;
		stan = 0;

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	}
}

	void start_stop(void)
	{
		if(start==0)
		{
			start=1;
			delay_ms(1000);
			GPIO_SetBits(GPIOC,GPIO_Pin_10);
			GPIO_ResetBits(GPIOC,GPIO_Pin_7|GPIO_Pin_9);
		}
		else if(start==1)
		{
			start=0;
			GPIO_SetBits(GPIOC,GPIO_Pin_7|GPIO_Pin_9);
			GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
			mode=0;
			delay_ms(500);
			GPIO_ResetBits(GPIOC,GPIO_Pin_7|GPIO_Pin_9);
			GPIO_ResetBits(GPIOC,GPIO_Pin_10);
		}
	}

void speed_up()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int16_t speed_profile[7]={-210,-140,-70,0,70,140,210};
	if (speed<6)
	{
		speed++;
		motor_1=motor_initial + speed_profile[speed];
		motor_2=motor_1+200;
		motor_3=motor_1+300;
	}
	if(speed==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	else if(speed==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	else if(speed==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	else if(speed==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	else if(speed==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	else if(speed==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	else if(speed==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void speed_down()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int16_t speed_profile[7]={-210,-140,-70,0,70,140,210};
	if (speed>0)
	{
		speed--;
		motor_1=motor_initial + speed_profile[speed];
		motor_2=motor_1+200;
		motor_3=motor_1+300;
	}
	if(speed==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	else if(speed==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	else if(speed==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	else if(speed==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	else if(speed==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	else if(speed==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	else if(speed==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void Kp_up()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int8_t Kp_profile[7]={-9,-6,-3,0,3,6,9};
	if (Kp_step<6)
	{
		Kp_step++;
		//Kp+=Kp_profile[Kp_step];
	}
	Kp++;
	if(Kp_step==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(Kp_step==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(Kp_step==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(Kp_step==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(Kp_step==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(Kp_step==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(Kp_step==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void Kp_down()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int8_t Kp_profile[7]={-9,-6,-3,0,3,6,9};
	if (Kp_step>0)
	{
		Kp_step--;
		//Kp+=Kp_profile[Kp_step];
	}
	Kp--;
	if(Kp_step==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(Kp_step==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(Kp_step==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(Kp_step==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(Kp_step==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(Kp_step==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(Kp_step==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void Kd_up()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int8_t Kd_profile[7]={-90,-60,-30,0,30,60,90};
	if (Kd_step<6)
		{
			Kd_step++;
			//Kd+=Kd_profile[Kd_step];
		}
		Kd+=10;
	if(Kd_step==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(Kd_step==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(Kd_step==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(Kd_step==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(Kd_step==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(Kd_step==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(Kd_step==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void Kd_down()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	int8_t Kd_profile[7]={-90,-60,-30,0,30,60,90};
	if (Kd_step>0)
		{
			Kd_step--;
			//Kd+=Kd_profile[Kd_step];
		}
		Kd-=10;
	if(Kd_step==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(Kd_step==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(Kd_step==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(Kd_step==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(Kd_step==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(Kd_step==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(Kd_step==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void tresh_up()
{
	//GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	uint16_t tresh_profile[7]={210,240,270,300,350,400,450};
	if (tresh<6) tresh++;
	Treshold=tresh_profile[tresh];
	/*if(tresh==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(tresh==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(tresh==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(tresh==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(tresh==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(tresh==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(tresh==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);*/
}
void tresh_down()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	uint16_t tresh_profile[7]={210,240,270,300,350,400,450};
	if (tresh>0) tresh--;
	Treshold=tresh_profile[tresh];
	if(tresh==0)GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
	if(tresh==1)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11);
	if(tresh==2)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
	if(tresh==3)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	if(tresh==4)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);
	if(tresh==5)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	if(tresh==6)GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
}
void delay_ms(int ms)
{
	int i, tms;
	tms = 5000*ms;
	for(i=0;i<tms;i++);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif
void menu()
{
	//sprawdzenie flagi odbioru danych
	if(rc5_ok != 0)
	{	int i=0;
		int j=0;
		//wykonanie kopii zdekodowanego kodu RC5
		rc5_code = rc5;

		//starts
		temp = (rc5_code  & 0x3000) >> 12;

		//toggle bit
		temp = (rc5_code  & 0x0800) >> 11;

		//adres
		temp = (rc5_code  & 0x0400) >> 10;
		temp = (rc5_code  & 0x03C0) >> 6;

		//rozkaz
		temp = (rc5_code & 0x0030) >> 4;
		temp = rc5_code & 0x000F;


		//sprawdzenie bitu toggle
		if((rc5_code & 0x0800) == 0)
			{rc5_toogle = 0;}
		else
			{rc5_toogle = 1;}


		//wykonanie kodu jesli wcisnieto "nowy" klawisz
		if(rc5_toogle != rc5_toogle_last)
		{
			//aktualizacja poprzedniej flagi toogle
			rc5_toogle_last = rc5_toogle;
			//klawisz 1
			if(szyfr<3)
			{
		if((rc5_code & 0x003f) == 5)
				{if(szyfr==0)szyfr=szyfr+1;
				}
				else if((rc5_code & 0x003f) == 8)
				{if(szyfr==1) szyfr=szyfr+1;
				else szyfr=0;
				}
				else if((rc5_code & 0x003f) == 6)
				{if(szyfr==2)
					{szyfr=szyfr+1;
					GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
					}
				else szyfr=0;
				}
			}
			else
		{
		if(tryb==2)
			{
			if((rc5_code & 0x003F) == 0 && (ilosc<5))
						{
						liczba[ilosc] = 0;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 1 && (ilosc<5))
						{
						liczba[ilosc] = 1;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 2 && (ilosc<5))
						{
						liczba[ilosc] = 2;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 3 && (ilosc<5))
						{
						liczba[ilosc] = 3;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 4 && (ilosc<5))
						{
						liczba[ilosc] = 4;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 5 && (ilosc<5))
						{
						liczba[ilosc] = 5;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 6 && (ilosc<5))
						{
						liczba[ilosc] = 6;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 7 && (ilosc<5))
						{
						liczba[ilosc] = 7;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 8 && (ilosc<5))
						{
						liczba[ilosc] = 8;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 9 && (ilosc<5))
						{
						liczba[ilosc] = 9;
						ilosc++;
						}
			if((rc5_code & 0x003F) == 14 || (ilosc==5))
			{
			if(ilosc==1) licz = licz + 1*liczba[0];
			else if(ilosc==2) licz = licz + 10*liczba[0] + 1*liczba[1];
			else if(ilosc==3) licz = licz + 100*liczba[0] + 10*liczba[1] + 1*liczba[2];
			else if(ilosc==4) licz = licz + 1000*liczba[0] + 100*liczba[1] + 10*liczba[2] + 1*liczba[3];
			else if(ilosc==5) licz = licz + 10000*liczba[0] + 1000*liczba[1] + 100*liczba[2] + 10*liczba[3] + 1*liczba[4];
			else if(ilosc==0) zmienna = 0;
			tryb=0;
			if(zmienna==1)
				{motor = licz;
				zmienna=0;
				if(motor > 3000){
					motor = 3000;
				}
				}
			if(zmienna==7)
						{motor_2 = licz;
						zmienna=0;
						if(motor_2 > 3000){
							motor_2 = 3000;
						}
						}
			if(zmienna==6)
						{motor_initial = licz;
						zmienna=0;
						if(motor_initial > 3000){
							motor_initial = 3000;
						}
						}
			if(zmienna==8)
						{rdyfzm = licz;
						zmienna=0;
						}
			if(zmienna==2)
			{Kp=licz;
			zmienna=0;
			if(Kp>30000)
				{Kp=30000;}
			}
			if(zmienna==3)
			{Kd=licz;
			zmienna=0;
			if(Kd>30000)
				{Kd=30000;}

			}
			if(zmienna==4)
				{Treshold=licz;
				if(Treshold>8000)
				{Treshold=8000;}
			}
			if(zmienna==5)
			{czas=licz;
			}
			}
			}
		else if(tryb==1)
			{GPIO_SetBits(GPIOA,GPIO_Pin_8);
			if((rc5_code & 0x003F) == 1)
				{zmienna = 1;
				tryb=2;
				GPIO_SetBits(GPIOA,GPIO_Pin_9);
				}//Predkosc <0;3000>
			else if((rc5_code & 0x003F) == 2)
				{zmienna = 2;
				tryb=2;
				GPIO_SetBits(GPIOA,GPIO_Pin_10);
				}//Kp <0;500>
			else if((rc5_code & 0x003F) == 3)
				{zmienna = 3;
				tryb=2;
				GPIO_SetBits(GPIOA,GPIO_Pin_11);
				}//Kd <0;100>
			else if((rc5_code & 0x003F) == 4)
				{zmienna = 4;
				tryb=2;
				}//Ki <0;10>
			else if((rc5_code & 0x003F) == 5)
			{zmienna=5;
			tryb=2;
			}
			else if((rc5_code & 0x003F) == 6)
			{zmienna=6;
			tryb=2;
			}

			else if((rc5_code & 0x003F) == 7)
			{zmienna=7;
			tryb=2;
			}
			else if((rc5_code & 0x003F) == 8)
			{zmienna=8;
			tryb=2;
			}

			else if((rc5_code & 0x003F) == 14)//Ok
				{zmienna = 0;
				tryb=0;
				}
			}
		else if(tryb==0){
			GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);
			if((rc5_code & 0x003F) == 54)//Start- green
				{start_stop();}
			else if((rc5_code & 0x003F) == 16)//Zwiekszenie o 10- volume up
				{motor+=50;
				if(motor>3000)
					{motor=3000;}
				}
			else if((rc5_code & 0x003F) == 17)
				{
					motor-=50;
				if(motor<0)
					{motor=0;}
				}
			else if((rc5_code & 0x003F) == 15)
				{
					tryb=1;
					licz=0;
					for(i=0;i<5;i++)
						liczba[i]=0;
					zmienna=0;
					ilosc=0;
					}
				}
			}
	}
	}
};

void sterowanie()
{//sprawdzenie flagi odbioru danych
	if(rc5_ok != 0)
	{
		//wykonanie kopii zdekodowanego kodu RC5
		rc5_code = rc5;

		//starts
		temp = (rc5_code  & 0x3000) >> 12;

		//toggle bit
		temp = (rc5_code  & 0x0800) >> 11;

		//adres
		temp = (rc5_code  & 0x0400) >> 10;
		temp = (rc5_code  & 0x03C0) >> 6;

		//rozkaz
		temp = (rc5_code & 0x0030) >> 4;
		temp = rc5_code & 0x000F;


		//sprawdzenie bitu toggle
		if((rc5_code & 0x0800) == 0)
			{rc5_toogle = 0;}
		else
			{rc5_toogle = 1;}


		//wykonanie kodu jesli wcisnieto "nowy" klawisz
		if(rc5_toogle != rc5_toogle_last)
		{
			//aktualizacja poprzedniej flagi toogle
			rc5_toogle_last = rc5_toogle;
			//klawisz 1
			if((rc5_code & 0x003F) == 12)//Start- duzy czerwony
							{start_stop();}
			else if((rc5_code & 0x003F) == 2)
				{WYPL=1.5*wartosc;
				WYPP=wartosc;}
			else if((rc5_code & 0x003F) == 5)
			{WYPL=0;
			WYPP=0;
			}
			else if((rc5_code & 0x003F) == 4)
			{WYPL=0;
			WYPP=wartosc;
			}
			else if((rc5_code & 0x003F) == 6)
			{WYPL=1.5*wartosc;
			WYPP=0;
			}
			else if((rc5_code & 0x003F) == 16)
							{wartosc = wartosc + 500;
							WYPL=WYPL+500;
							WYPP=WYPP+500;
							if(wartosc>3000)
								{wartosc=3000;}
							if(WYPL>3000)
								{WYPL=3000;}
							if(WYPP>3000)
								{WYPP=3000;}
							}
			else if((rc5_code & 0x003F) == 17)
							{
							wartosc = wartosc - 500;
							WYPL=WYPL-500;
							WYPP=WYPP-500;
							if(wartosc<0)
								{wartosc=0;}
							if(WYPL<0)
								{WYPL=0;}
							if(WYPP<0)
								{WYPP=0;}
							}
			TIM1->CCR2 = WYPL;
			TIM1->CCR3 = WYPP;
		}
	}
}



/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

