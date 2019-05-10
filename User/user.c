/**
  ******************************************************************************
  * @file    user.c
  * @author  Phan Van Hai
  * @version V3
  * @date    01/08/2018
  * @brief   This file contains all the functions prototypes for user.
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "user.h"
#include "Blue_Uart.h"

const uint8_t arrResponseOK[] = "000194E1";
const uint8_t arrResponseERROR[] = "000084C0";
/* Private typedef -----------------------------------------------------------*/
/** 
  * @brief: defines the alarm structure
  */ 
typedef struct
{
	uint8_t	dim;
	uint8_t pwm1;
	uint8_t pwm2;
	uint8_t hour;
	uint8_t min;
	uint8_t reserve;
} PACKET_AlarmTypeDef;

/* Private define ------------------------------------------------------------*/
			/************** define Format Packet ***************/								
#define MAX_ALARM						50						// max number of alarms
#define	SIZE_PACKET_ALARM				6						// packet size alarm

#define MAX_NODE						8
#define INDEX_CMD						0
/* Packet Notifi */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_NOTIFI_CONNECT_MAC1 = 1,
	INDEX_NOTIFI_CONNECT_MAC2,
	INDEX_NOTIFI_CONNECT_MAC3,
	INDEX_NOTIFI_CONNECT_MAC4,
	INDEX_NOTIFI_CONNECT_MAC5,
	INDEX_NOTIFI_CONNECT_MAC6,
	INDEX_NOTIFI_STATUS_CONNECT
} PACKET_INDEX_NotifiConnectTypeDef;

/* Packet Set Time */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_SETTIME_HOUR = 1,
	INDEX_SETTIME_MIN,
	INDEX_SETTIME_SEC
} PACKET_INDEX_SetTimeTypeDef;

/* Packet Start Alarm */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_STARTALARM_SIZE	= 1
} PACKET_INDEX_StartAlarmTypeDef;

/* Packet Set Alarm */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_SETALARM_DIM = 1,
	INDEX_SETALARM_PWM1,
	INDEX_SETALARM_PWM2,
	INDEX_SETALARM_HOUR,
	INDEX_SETALARM_MIN
} PACKET_INDEX_SetAlarmTypeDef;

/* Packet signIn/Set PIN */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_MAC1 = 1,
	INDEX_MAC2,
	INDEX_MAC3,
	INDEX_MAC4,
	INDEX_MAC5,
	INDEX_MAC6,
	INDEX_PIN1,
	INDEX_PIN2,
	INDEX_PIN3,
	INDEX_PIN4
} PACKET_INDEX_PinMacTypeDef;

/* Packet Illuminate */
typedef enum
{
	/* INDEX_CMD = 0, */
	INDEX_ILL_DIM = 1,
	INDEX_ILL_PWM1,
	INDEX_ILL_PWM2
} PACKET_INDEX_IlluminateTypeDef;

/* Private macro -------------------------------------------------------------*/
#define BCD2HEX(a) ((uint8_t)((a>>4)*10 + (a & 0x0F)))


/* Private variables ---------------------------------------------------------*/
typedef struct
{
	PACKET_AlarmTypeDef   		arrAlarm[MAX_ALARM];	
	FlagStatus			Flag_ReceiveAllAlarm ;
	uint8_t 			size ;
	uint8_t				i_update;
	uint8_t				i_setAlarm;
} Alarm_Infor_TypeDef;

static __IO Alarm_Infor_TypeDef alarm_t = { {}, SET, 0, 0, 0xff };

typedef struct
{
	uint8_t arrDataForward[MAX_LEN_PAYLOAD_FORMART_HEX];
	uint8_t arrDataForward_len;
	uint8_t arrMACInGroup_indexForward;
	FlagStatus Flag_ForwardComplete;
	FlagStatus	Flag_RestartForward;
} ForwardData_TypeDef;

static ForwardData_TypeDef forward_t 	= { {0}, 0, 0xFF, SET, SET };
const MAC_TypeDef	MAC_ZERO			= {0, 0};
MAC_TypeDef 		MyMAC 				= {0, 0};
MAC_TypeDef			arrIdNode[MAX_NODE] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
uint8_t 			arrIdNode_len  		= 0;
MAC_TypeDef 		arrMACInGroup[MAX_NODE] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
FlagStatus			flag_ControlForIt	= SET;

uint32_t  PIN;

__IO	uint32_t 			countTick;
__IO 	uint32_t 			LsiFreq 			= 0;
__IO 	uint32_t 			CaptureNumber 		= 0;
__IO	uint32_t			PeriodValue 		= 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t 		Flash_SearchBlock(void);
static uint32_t 	Flash_SearchPinMACAddr(void);
static uint32_t 	GetLSIFrequency(void);
static uint8_t 		FindIdFromMAC(MAC_TypeDef idMAC);
static uint8_t 		GATE_ForwardFindNode(ErrorStatus flag);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get Command in the packet data.
  * @param  The array contains the packet data.
  * @retval Code command.
  */
PACKET_CmdTypeDef GetCMD(uint8_t* arrData)
{
	return (PACKET_CmdTypeDef)arrData[INDEX_CMD];
}

/**
  * @brief  Configures TIM14 to measure the LSI oscillator frequency. 
  * @param  None
  * @retval LSI Frequency
  */
static uint32_t GetLSIFrequency(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	RCC_ClocksTypeDef  RCC_ClockFreq;

	/* TIM14 configuration *******************************************************/ 
	/* Enable TIM14 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Reset TIM14 registers */
	TIM_DeInit(TIM14);

	/* Configure TIM14 prescaler */
	TIM_PrescalerConfig(TIM14, 0, TIM_PSCReloadMode_Immediate);

	/* Connect internally the TIM14_CH1 to the RTC clock output */
	TIM_RemapConfig(TIM14, TIM14_RTC_CLK);

	/* TIM14 configuration: Input Capture mode ---------------------
	 The reference clock(LSE or external) is connected to TIM14 CH1
	 The Rising edge is used as active edge,
	 The TIM14 CCR1 is used to compute the frequency value 
	------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter 	= 0x0;
	TIM_ICInit(TIM14, &TIM_ICInitStructure);

	/* Enable the TIM14 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel  		= TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable TIM14 counter */
	TIM_Cmd(TIM14, ENABLE);

	/* Reset the flags */
	TIM14->SR = 0;

	/* Enable the CC1 Interrupt Request */  
	TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);


	/* Wait until the TIM14 get 2 LSI edges (refer to TIM14_IRQHandler() in 
	stm32F0xx_it.c file) ******************************************************/
	while(CaptureNumber != 2)
	{
	}
	/* Deinitialize the TIM14 peripheral registers to their default reset values */
	TIM_DeInit(TIM14);


	/* Compute the LSI frequency, depending on TIM14 input clock frequency (PCLK1)*/
	/* Get SYSCLK, HCLK and PCLKx frequency */
	RCC_GetClocksFreq(&RCC_ClockFreq);

	/* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
	return ((RCC_ClockFreq.PCLK_Frequency / PeriodValue) * 8);
}


/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void _RTC_Config(void)
{
	RTC_InitTypeDef   	RTC_InitStructure;
	RTC_TimeTypeDef 	RTC_TimeStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure; 
	EXTI_InitTypeDef 	EXTI_InitStructure;

	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);

	/* LSI used as RTC source clock */
	/* The RTC Clock may varies due to LSI frequency dispersion. */   
	/* Enable the LSI OSC */ 
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* EXTI configuration *******************************************************/
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Wakeup Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Disable the Alarm A */
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	/* Enable the RTC Alarm A Interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);


	LsiFreq = GetLSIFrequency();

	/* Calendar Configuration */
	RTC_InitStructure.RTC_AsynchPrediv 	= 49;
	RTC_InitStructure.RTC_SynchPrediv	= (LsiFreq / 50) - 1;
	RTC_InitStructure.RTC_HourFormat 	= RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);

	  /* Set the time to 00h 00mn 00s AM */
	RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours   = 0x00;
	RTC_TimeStructure.RTC_Minutes = 0x00;
	RTC_TimeStructure.RTC_Seconds = 0x00;  

	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
}


/**
  * @brief  Search for the next block to save the alarm array.
  * @param  None
  * @retval Location of the block found.
  */
static uint8_t Flash_SearchBlock(void)
{
	uint8_t i,j;
	uint32_t addr = FLASH_START_ARR_BLOCK_ADDR;
	
	// find the block used
	for( i = 0; i < FLASH_NUMBER_BLOCK; i++ )
	{
		addr = FLASH_START_ARR_BLOCK_ADDR + (uint32_t)(i * FLASH_BLOCK_SIZE);
		if( (uint16_t)(*((uint16_t*)addr)) != 0xFFFF )
			break;
	}
	for( j = i + 1; j < FLASH_NUMBER_BLOCK; j++)
	{
		addr = FLASH_START_ARR_BLOCK_ADDR + (uint32_t)(j * FLASH_BLOCK_SIZE);
		if ( (uint16_t)(*((uint16_t*)addr)) == 0xFFFF )
			break;
	}
	if( j >= FLASH_NUMBER_BLOCK) 
		j = 0;
	return j;
}
/**
  * @brief  Search for the next address to save the PIN.
  * @param  None
  * @retval Address found.
  * @note 	The scope of saving a PIN is in the following page: FLASH_PIN_START_ADDR to FLASH_PIN_END_ADDR
  */
static uint32_t Flash_SearchPinMACAddr(void)
{
	uint32_t addr = FLASH_PIN_START_ADDR;
	
	while( addr < FLASH_PIN_END_ADDR)
	{
		if( (uint32_t)(*((uint32_t*)addr)) == 0xFFFFFFFF )
			break;
		addr = addr + 16U;
	}
	return addr;
}

/* Public functions ---------------------------------------------------------*/
///**
//  * @brief  Restart Timeout.
//  * @param  Min: timeout period (ms).
//  * @retval None
//  */
//void TimeOut_Restart(uint32_t ms_timeout)
//{
//	SysTick_Config( SystemCoreClock / 1000 );
//	countTick =  ms_timeout;
//}
///**
//  * @brief  Stop Timeout.
//  * @param  None
//  * @retval None
//  */
//void TimeOut_Stop(void)
//{
//	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk ;
//}
///**
//  * @brief  Stop Timeout.
//  * @param  None
//  * @retval None
//  */
//FlagStatus TimeOut_CheckFlag(void)
//{
//	if(countTick == 0)
//		return SET;
//	return RESET;
//}
/**
  * @brief  Decrease count Timeout.
  * @param  None
  * @retval value count Timeout.
  * @note   This function is used in the SysTick interrupt.
  */
uint32_t DecCountTick(void)
{
	countTick--;
	return countTick;
}

/**
  * @brief  Configuration Timer for generating PWM
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
	uint16_t 					TimerPeriod = 0;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	GPIO_InitTypeDef 			GPIO_InitStructure;

	/* GPIOA, GPIOB and GPIOE Clocks enable */
	RCC_AHBPeriphClockCmd( TIM_GPIO_CLK, ENABLE);


	/* GPIOA Configuration: Channel 1, 2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin 	= TIM_CH1_PIN | TIM_CH2_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP ;
	GPIO_Init(TIM_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(TIM_GPIO_PORT, TIM_CH1_SOURCE, TIM_CH1_AF);
	GPIO_PinAFConfig(TIM_GPIO_PORT, TIM_CH2_SOURCE, TIM_CH1_AF);


	/* Compute the value to be set in ARR regiter to generate signal frequency at 10 Khz */
	TimerPeriod = (SystemCoreClock / F_PWM ) - 1;

	/* TIM3 clock enable */
	TIM_AHBPERIPHCLOCK(TIM_CLK, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler 	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period 		= TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/* Channel 1, 2 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_Pulse 		= 0;
	TIM_OC1Init(TIMx, &TIM_OCInitStructure);
	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIMx, ENABLE);
}


/*
return:
*/
void TIM_SetPWM_CH1_CH2(uint8_t Channel1Pulse, uint8_t Channel2Pulse)
{
	TIM_Cmd(TIMx, DISABLE);
	uint16_t TimerPeriod	= (SystemCoreClock / F_PWM ) - 1;
	
	/* Set the Capture Compare Register value */
	TIMx->CCR1	= (uint16_t) (((uint32_t) Channel1Pulse * (TimerPeriod - 1)) / 255);	// quy doi tu dai 4 byte thanh dai 1 byte
	TIMx->CCR2	= (uint16_t) (((uint32_t) Channel2Pulse * (TimerPeriod - 1)) / 255);
	TIM_Cmd(TIMx, ENABLE);
}
/**
  * @brief  Create PWM pulses.
  * @param  The array contains data indicating duty cycle PWM.
  * @retval None
  */
void Illuminate_Normal(uint8_t* arrData)
{
	uint16_t TimerPeriod	= (SystemCoreClock / F_PWM ) - 1;
	uint16_t pwm1 	= (uint16_t) (BCD2HEX(arrData[INDEX_ILL_DIM]) * BCD2HEX(arrData[INDEX_ILL_PWM1]));
	uint16_t pwm2 	= (uint16_t) (BCD2HEX(arrData[INDEX_ILL_DIM]) * BCD2HEX(arrData[INDEX_ILL_PWM2]));
	TIM_Cmd(TIMx, DISABLE);
	/* Set the Capture Compare Register value */
	TIMx->CCR1		= (uint16_t) (((uint32_t) pwm1 * (TimerPeriod - 1)) / (99*99));	// quy doi tu dai 4 byte thanh dai 1 byte
	TIMx->CCR2		= (uint16_t) (((uint32_t) pwm2 * (TimerPeriod - 1)) / (99*99));
	TIM_Cmd(TIMx, ENABLE);
}
/**
  * @brief  Create PWM pulse used in Alarm interrupt.
  * @param  None
  * @retval None
  */
void Illuminate_Alarm(void)
{
	uint16_t TimerPeriod	= (SystemCoreClock / F_PWM ) - 1;
	uint16_t pwm1 = (uint16_t) (BCD2HEX(alarm_t.arrAlarm[alarm_t.i_setAlarm].dim) * BCD2HEX(alarm_t.arrAlarm[alarm_t.i_setAlarm].pwm1));
	uint16_t pwm2 = (uint16_t) (BCD2HEX(alarm_t.arrAlarm[alarm_t.i_setAlarm].dim) * BCD2HEX(alarm_t.arrAlarm[alarm_t.i_setAlarm].pwm2));
	TIM_Cmd(TIMx, DISABLE);
	/* Set the Capture Compare Register value */
	TIMx->CCR1	  = (uint16_t) (((uint32_t) pwm1 * (TimerPeriod - 1)) / (99*99));	// quy doi tu dai 4 byte thanh dai 1 byte
	TIMx->CCR2	  = (uint16_t) (((uint32_t) pwm2 * (TimerPeriod - 1)) / (99*99));
	TIM_Cmd(TIMx, ENABLE);
}


/**
  * @brief  Real time for RTC.
  * @param  The array contains real-time data.
  * @retval None
  */
void SetTimeCurrent(uint8_t* arrData)
{
	RTC_TimeTypeDef	RTC_TimeStruct;
	
	RTC_TimeStruct.RTC_H12			= RTC_H12_AM;
	RTC_TimeStruct.RTC_Seconds	= arrData[INDEX_SETTIME_SEC];
	RTC_TimeStruct.RTC_Minutes	= arrData[INDEX_SETTIME_MIN];
	RTC_TimeStruct.RTC_Hours		= arrData[INDEX_SETTIME_HOUR];
	
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStruct);
	alarm_t.i_setAlarm	= 0xff;
}

/**
  * @brief  Get about the number of alarms to be sent.
  * @param  The array contains data about the number of alarm packets.
  * @retval None
  */
void SetSizeArrAlarm(uint8_t* arrData)
{
	alarm_t.size	  = BCD2HEX(arrData[INDEX_STARTALARM_SIZE]);
	alarm_t.i_update = 0;
	alarm_t.Flag_ReceiveAllAlarm = RESET;
}

/**
  * @brief  Get a new alarm packet and save it to the array.
  * @param  The array contains the alarm packet.
  * @retval None
  */
void UpdateArrAlarm(uint8_t* arrData)
{
	if( alarm_t.i_update < MAX_ALARM )
	{
		alarm_t.arrAlarm[alarm_t.i_update].dim	 = arrData[INDEX_SETALARM_DIM];
		alarm_t.arrAlarm[alarm_t.i_update].pwm1 = arrData[INDEX_SETALARM_PWM1];
		alarm_t.arrAlarm[alarm_t.i_update].pwm2 = arrData[INDEX_SETALARM_PWM2];
		alarm_t.arrAlarm[alarm_t.i_update].hour = arrData[INDEX_SETALARM_HOUR];
		alarm_t.arrAlarm[alarm_t.i_update].min  = arrData[INDEX_SETALARM_MIN];
		
		alarm_t.i_update++;
		if( alarm_t.i_update == alarm_t.size )
			alarm_t.Flag_ReceiveAllAlarm = SET;
	}
}
 
FlagStatus CheckReceiveAllAlarm(void)
{
	return alarm_t.Flag_ReceiveAllAlarm;
}
/**
  * @brief  Rearrange the alarm array in ascending order.
  * @param  None
  * @retval None
  */
void SortAlarm(void)
{
	PACKET_AlarmTypeDef tempAlarm;
	uint8_t i, j, pos;
	uint16_t compare1, compare2;
	
	if( alarm_t.size == 0 )
		return ;
	
	for( i = 0; i < alarm_t.size; i++ )
	{
		pos = i;
		compare1	= (uint16_t) ((alarm_t.arrAlarm[i].hour << 8) | alarm_t.arrAlarm[i].min);
		for( j = i + 1; j < alarm_t.size; j++)
		{
			compare2 = (uint16_t) ((alarm_t.arrAlarm[j].hour << 8) | alarm_t.arrAlarm[j].min);
			if( compare2 < compare1 )
			{
				compare1	= compare2;
				pos 		= j;
			}
		}
		tempAlarm 		= alarm_t.arrAlarm[i];
		alarm_t.arrAlarm[i] 	= alarm_t.arrAlarm[pos];
		alarm_t.arrAlarm[pos]   = tempAlarm;
	}
	
	alarm_t.i_setAlarm	= 0xff;
}
/**
  * @brief  Update and set up a new alarm.
  * @param  None
  * @retval None
  */
void UpdateSetAlarm(void)
{
	uint8_t 			i;
	uint16_t 			compare1, compare2;
	RTC_TimeTypeDef 	RTC_TimeStruct;
	RTC_AlarmTypeDef  	RTC_AlarmStructure;
	
	if( alarm_t.size == 0 )
	{
		/* Disable the Alarm A */
		RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
		
		return;
	}
	
	RTC_GetTime(RTC_Format_BCD, &RTC_TimeStruct);
	compare1	= (uint16_t) ((RTC_TimeStruct.RTC_Hours << 8) | RTC_TimeStruct.RTC_Minutes);
	
	if( alarm_t.i_setAlarm < 0xff )
	{
		alarm_t.i_setAlarm++;
	}
	else
	{
		for( i= 0; i < alarm_t.size; i++ )
		{
			compare2 = (uint16_t) ((alarm_t.arrAlarm[i].hour << 8) | alarm_t.arrAlarm[i].min);
			if( compare2 > compare1 )
			{
				alarm_t.i_setAlarm	= i;
				break;
			}
		}
	}
	if( alarm_t.i_setAlarm >= alarm_t.size )
		alarm_t.i_setAlarm = 0;
	//--------------------------------------------------------------------------------------
	/* Disable the Alarm A */
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	
	/* Set the alarm*/
	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     	= RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours  	= alarm_t.arrAlarm[alarm_t.i_setAlarm].hour;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes 	= alarm_t.arrAlarm[alarm_t.i_setAlarm].min ;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds 	= 0x00;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay 		= RTC_Weekday_Monday;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel 		= RTC_AlarmDateWeekDaySel_WeekDay;
	RTC_AlarmStructure.RTC_AlarmMask 				= RTC_AlarmMask_DateWeekDay ;
	
	/* Configure the RTC Alarm A register */
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
	
	/* Enable the Alarm A */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}
/**
  * @brief  Put the user data storage area to the default value.
  * @note   The saved area is completely erased. Create a default PIN of "0000".
  * @param  The starting address of the data area.
  * @param  End address of data area.
  * @param  The size of each page.
  * @retval None.
  */
void Flash_Default(uint32_t addr_start, uint32_t addr_end, uint32_t sizepage)
{
	uint32_t NbrOfPage = 0x00;
	uint32_t EraseCounter = 0x00;
	__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

	 /* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	/* Define the number of page to be erased */
	NbrOfPage = (addr_end - addr_start) / sizepage;

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
	if (FLASH_ErasePage(addr_start + (sizepage * EraseCounter))!= FLASH_COMPLETE)
	{
		/* Error occurred while sector erase. 
		 User can add here some code to deal with this error  */
		while (1)
		{
		}
	}
	}
	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	if (FLASH_ProgramWord(FLASH_PIN_START_ADDR, 0x00000000) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
		 User can add here some code to deal with this error */
		while (1)
		{
		}
	}
	if (FLASH_ProgramWord(FLASH_PIN_START_ADDR + 4, 0x00000000) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
		 User can add here some code to deal with this error */
		while (1)
		{
		}
	}
	
	if (FLASH_ProgramWord(FLASH_PIN_START_ADDR + 8, 0x00000000) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
		 User can add here some code to deal with this error */
		while (1)
		{
		}
	}

	if (FLASH_ProgramWord(FLASH_PIN_START_ADDR + 12, 0x00000000) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
		 User can add here some code to deal with this error */
		while (1)
		{
		}
	}
	
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
}

/**
  * @brief  Get the alarm array from Flash saved to an array on RAM.
  * @param  None
  * @retval None
  */
void Flash2Arr(void)
{
	uint8_t 	i, i_block;
	uint16_t* 	pArrAlarm	= (uint16_t*)alarm_t.arrAlarm;
	uint32_t 	addr_data ;
	
	i_block = Flash_SearchBlock(); 
	if( i_block == 0 )
		i_block = FLASH_NUMBER_BLOCK - 1U;
	else
		i_block--;
	
	addr_data = FLASH_START_ARR_BLOCK_ADDR + (uint32_t)( i_block * FLASH_BLOCK_SIZE) ;
	
	alarm_t.size = (*((uint16_t*)addr_data)) & 0x00ff;
	if( alarm_t.size == 0xff )
		alarm_t.size = 0;
	
	addr_data		= addr_data + 2;
	
	for( i = 0; (i < (SIZE_PACKET_ALARM * alarm_t.size / 2)) && (i < MAX_ALARM); i++ )
	{
		*((uint16_t*)pArrAlarm) = *((uint16_t*)addr_data);
		pArrAlarm++;
		addr_data		= addr_data + 2;
	}
}

/**
  * @brief  Save the alarm array (sorted) into Flash.
  * @param  None
  * @retval Flag error or success.
  */
ErrorStatus Arr2Flash(void)
{
	uint8_t i, i_block;
	uint32_t addr_erase, addr_data;
	uint16_t* pArrAlarm	= (uint16_t*)alarm_t.arrAlarm;

	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	i_block = Flash_SearchBlock();  
	if( (i_block % 2) == 0 )
	{
		// erase the pre-page
		if( i_block == 0 )
			addr_erase =  FLASH_START_ARR_BLOCK_ADDR + (uint32_t)( (FLASH_NUMBER_BLOCK - 2U) * FLASH_BLOCK_SIZE);
		else 
			addr_erase =  FLASH_START_ARR_BLOCK_ADDR + (uint32_t)( (i_block - 2U) * FLASH_BLOCK_SIZE);

		/* Erase the user Flash area */
		/* Clear pending flags (if any) */  
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
		/* Erase the 1 FLASH pages */
		if (FLASH_ErasePage(addr_erase) != FLASH_COMPLETE)
		{	
			while(1);
			//return ERROR;
		}
	}

	/* Program the user Flash area word by word */

	addr_data = FLASH_START_ARR_BLOCK_ADDR + (uint32_t)( i_block * FLASH_BLOCK_SIZE);

	if (FLASH_ProgramHalfWord(addr_data, (uint16_t) alarm_t.size) == FLASH_COMPLETE)
		addr_data 	=	addr_data + 2;
	else
		{	
			while(1);
			//return ERROR;
		}

	for( i = 0; (i < (SIZE_PACKET_ALARM * alarm_t.size / 2)) && (i < MAX_ALARM); i++ )
	{
		if (FLASH_ProgramHalfWord(addr_data, (uint16_t)(*((uint16_t*)pArrAlarm))) == FLASH_COMPLETE)
		{
			addr_data 	=		addr_data + 2;
			pArrAlarm++;
		}
		else
		{	
			while(1);
			//return ERROR;
		}
	}
	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	return SUCCESS;
}
/**
  * @brief  Save PIN and MAC to Flash.
  * @param  The array contains the PIN_MAC packet.
  * @retval Flag error or success.
  */
ErrorStatus PinMAC2Flash(uint8_t* arrData)
{
	uint32_t addr;
	MAC_TypeDef mac;
	
	mac.MACLow = (uint16_t) ((arrData[INDEX_MAC5] << 8) + arrData[INDEX_MAC6]);
	mac.MACHigh = (uint32_t) ( (arrData[INDEX_MAC1] << 24)+ (arrData[INDEX_MAC2] << 16) + (arrData[INDEX_MAC3] << 8) + arrData[INDEX_MAC4]);
	
	MyMAC = mac;
	arrIdNode[0] = MyMAC;
	
	PIN = (uint32_t)((arrData[INDEX_PIN1] << 24) | (arrData[INDEX_PIN2] << 16) |
				(arrData[INDEX_PIN3] << 8) | arrData[INDEX_PIN4]);
	addr = Flash_SearchPinMACAddr();
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	if( addr >= FLASH_PIN_END_ADDR )
	{
		/* Erase the user Flash area
			(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

		/* Clear pending flags (if any) */  
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

		/* Erase the 1 FLASH pages */
		if (FLASH_ErasePage(FLASH_PIN_START_ADDR )!= FLASH_COMPLETE)
		{
			/* Error occurred while sector erase. 
				 User can add here some code to deal with this error  */
			return ERROR;
		}
		
		addr = FLASH_PIN_START_ADDR;
	}
	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/	
	if (FLASH_ProgramWord(addr, PIN) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
			 User can add here some code to deal with this error */
		return ERROR;
	}
	addr = addr + 4U;
	if (FLASH_ProgramWord(addr, mac.MACHigh) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
			 User can add here some code to deal with this error */
		return ERROR;
	}
	addr = addr + 4U;
	if (FLASH_ProgramHalfWord(addr, mac.MACLow) != FLASH_COMPLETE)
	{ 
		/* Error occurred while writing data in Flash memory. 
			 User can add here some code to deal with this error */
		return ERROR;
	}

	 /* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	return SUCCESS;
}
/**
  * @brief  Get PIN and MAC from Flash.
  * @param  None
  * @retval None
  */
void Flash2PinMAC(void)
{
	uint32_t addr;
	
	addr 	= Flash_SearchPinMACAddr();
	
	addr 	= addr - 16U;
	PIN		= (uint32_t) (*( (uint32_t*)addr ) );
	if( PIN == 0xFFFFFFFF )
		PIN = 0;
	addr 	= addr + 4U;
	MyMAC.MACHigh   = (uint32_t) (*((uint32_t*)addr));
	addr 			= addr + 4U;
	MyMAC.MACLow   	= (uint16_t) (*((uint16_t*)addr));
	
	arrIdNode[0] 	= MyMAC;
	
}

/**
  * @brief  Check the login.
  * @param  The array contains the login PIN from the user.
  * @retval Flag error or success login.
  */
ErrorStatus CheckSignIn(uint8_t* arrData)
{
	if( PIN == (uint32_t)((arrData[INDEX_PIN1] << 24) | (arrData[INDEX_PIN2] << 16) |
		 (arrData[INDEX_PIN3] << 8) | arrData[INDEX_PIN4]) )
		return SUCCESS;
	return ERROR;
}


/**
  * @brief  Add new Node to array IdNode
  * @param  Id of the node just connected 
  * @retval None
  * @Note   arrIdNode[id] = 0  -> Node is not connected to Gate
  *						  = 0xFFFFFFFFFF	-> Node connected to Gate but unknow MAC of Node
  *						  = MAC -> Node have MAC connected to Gate
  */
void GATE_AddIdNode(uint8_t id)
{
	arrIdNode[id].MACLow = 0xFFFF;
	arrIdNode[id].MACHigh = 0xFFFFFFFF;
}

/**
  * @brief  Remove Node just lost connected
  * @param  Id of the node
  * @retval None
  */
void GATE_RemoveIdNode(uint8_t id)
{
	arrIdNode[id].MACLow 		= 0;
	arrIdNode[id].MACHigh 		= 0;
	
	arrMACInGroup[id].MACLow 	= 0;
	arrMACInGroup[id].MACHigh 	= 0;
}
/**
  * @brief  Find ID of Node will be next forwarded
  * @param  flag = ERROR: find Id again recent forwarded
  *              = SUCCESS   : find next new Id 
  * @retval id that was found
  */
uint8_t GATE_ForwardFindNode(ErrorStatus flag)
{
	uint8_t i;
	if( flag == ERROR )
		i = forward_t.arrMACInGroup_indexForward;
	else
		i = forward_t.arrMACInGroup_indexForward + 1;
	
	for( ; i < MAX_NODE; i++)
		if( (arrMACInGroup[i].MACLow != MAC_ZERO.MACLow) && (arrMACInGroup[i].MACHigh != MAC_ZERO.MACHigh) )
		{
			forward_t.arrMACInGroup_indexForward = i;
			return i;
		}
		
	forward_t.arrMACInGroup_indexForward = 0xFF;
	return 0xFF;		// not found
}
/**
  * @brief  Start forward to all Node
  * @param  data will be forwarded
  * @retval None
  */
void GATE_StartForward2Node(uint8_t len, uint8_t* arrData)
{
	uint8_t i;
	
	forward_t.arrDataForward_len = len;
	for(i = 0; i < len; i++)
		forward_t.arrDataForward[i] = arrData[i];
	
	forward_t.arrMACInGroup_indexForward = 1;		// 0 - ung voi MAC cua chinh GATE, se dc bo qua
	
	forward_t.Flag_ForwardComplete = RESET;
	forward_t.Flag_RestartForward	 = SET;
	
	GATE_Forward2Node(ERROR);
}
/**
  * @brief  Get flag Flag_RestartForward
  * @param  None
  * @retval None
  * @Note   This flag indicates that the previous packet forwarding 
  *          has to be skipped to start forwarding the new packet
  */
FlagStatus GetFlagRestartForward(void)
{
	return forward_t.Flag_RestartForward;
}

/**
  * @brief  Set flag FlagRestartForward
  * @param  SET/RESET
  * @retval None
  * @Note   This flag indicates that the previous packet forwarding 
  *          has to be skipped to start forwarding the new packet
  */
void SetFlagRestartForward(FlagStatus flag)
{
	forward_t.Flag_RestartForward = flag;
}
/**
  * @brief  Forward data to Node.
  * @param  flag = ERROR: forward Node again recent forwarded
  *              = SUCCESS   : forward next new Node 
  * @retval None
  */
void GATE_Forward2Node(ErrorStatus flag)
{
	uint8_t idNode;
	if( forward_t.Flag_ForwardComplete == SET )
		return;
	
	idNode = GATE_ForwardFindNode(flag);
	// end if
	if( idNode == 0xFF )
		forward_t.Flag_ForwardComplete = SET;
	else
	{
		BLE_Send2Node(idNode, forward_t.arrDataForward_len, forward_t.arrDataForward);	
	}
}
/**
  * @brief  Add MAC of Node to arrIdNode
  * @param  array data contain MAC
  * @retval None
  */
void GATE_AddMAC(uint8_t idNode, uint8_t* arrData)
{
	arrIdNode[idNode].MACLow = (uint16_t)((arrData[INDEX_NOTIFI_CONNECT_MAC5] << 8) + arrData[INDEX_NOTIFI_CONNECT_MAC6]);
	arrIdNode[idNode].MACHigh = (uint32_t)((arrData[INDEX_NOTIFI_CONNECT_MAC1] << 24) + (arrData[INDEX_NOTIFI_CONNECT_MAC2] << 16)+(arrData[INDEX_NOTIFI_CONNECT_MAC3] << 8) + arrData[INDEX_NOTIFI_CONNECT_MAC4]);
}

/**
  * @brief  Create a Group
  * @param  cmd = CMD_CREATE_GROUP: create a new group
  *	            = CMD_ADD_TO_GROUP: add new Node to group
  *         arrData: contain MACs will be added to group
  * @retval ERROR if there is a Node in list not connected to Gate
  *         SUCCESS if all Node in list connected to Gate
  */
ErrorStatus GATE_CreateGroup(uint8_t cmd, uint8_t len, uint8_t* arrData)
{
	uint8_t i, idNode ;
	MAC_TypeDef idMAC;
	ErrorStatus flag = SUCCESS;
	
	if( cmd == CMD_CREATE_GROUP)
	{
		for( i = 0; i < MAX_NODE; i++ )
			arrMACInGroup[i] = MAC_ZERO;
		flag_ControlForIt = RESET;
	}
	
	len = len - 3U;
	
	for(i = 0 ; i < len/6; i++)
	{
		idMAC.MACHigh = (arrData[6*i + 1] << 24) + (arrData[6*i + 2] << 16) + (arrData[6*i + 3] << 8) + arrData[6*i+4];
		idMAC.MACLow = (arrData[6*i + 5] << 8) + arrData[6*i+6];
		
		idNode = FindIdFromMAC(idMAC);
		if( idNode != 0xFF)
		{
			if( idNode == 0 )
				flag_ControlForIt = SET;
			// end if
			arrMACInGroup[idNode] = idMAC;
		}
		else
			flag = ERROR;
	}
	return flag;
}
/**
  * @brief  Find Id of Node from MAC of Node
  * @param  MAC of the node
  * @retval Id that was found
  */
uint8_t FindIdFromMAC(MAC_TypeDef idMAC)
{
	uint8_t i;
	for(i = 0; i < MAX_NODE; i++)
		if((arrIdNode[i].MACLow == idMAC.MACLow) && (arrIdNode[i].MACHigh == idMAC.MACHigh))
			return i;
	return 0xFF;
}

/**
  * @brief  Find MAC of Node from Id of Node
  * @param  Id of the node
  * @retval MAC that was found
  */
MAC_TypeDef FindMACFromId(uint8_t idNode)
{
	return arrIdNode[idNode] ;
}
/**
  * @brief  Check if the packet is sent to Gate
  * @param  None
  * @retval SET/RESET
  */
FlagStatus CheckControlForIt(void)
{
	return flag_ControlForIt;
}

/**
  * @brief  get MAC of device
  * @param  None
  * @retval MAC of device
  */
MAC_TypeDef GetMAC(void)
{
	return MyMAC;
}
/**
  * @brief  send a notification to central about state connect
  * @param  mac :MAC of Node
  * @param  flag_connect : satae connect
  * @retval None
  */
void NotifiConnect2Central(MAC_TypeDef mac, FlagStatus flag_connect)
{
	uint8_t arrData[9];
	arrData[0] = CMD_NOTIFI_CONNECT_FROMM_NODE;
	
	arrData[1] = (uint8_t)(mac.MACHigh >> 24);
	arrData[2] = (uint8_t)((mac.MACHigh >> 16) & 0x00FF);
	arrData[3] = (uint8_t)((mac.MACHigh >> 8) & 0x0000FF);
	arrData[4] = (uint8_t)(mac.MACHigh  & 0x000000FF);
	
	arrData[5] = (uint8_t)(mac.MACLow >> 8);
	arrData[6] = (uint8_t)(mac.MACLow  & 0x00FF);
	
	if( flag_connect == RESET)
		arrData[7] = 0x00;
	else
		arrData[7] = 0x01;
	*((uint16_t*)(arrData+8)) = CRCCalculator(8, arrData);
	
	BLE_Send2Central(9, arrData);
}

/**
  * @brief  check if the forwarded is complete
  * @param  Id of the node
  * @retval None
  */
FlagStatus CheckForwardComplete(void)
{
	return forward_t.Flag_ForwardComplete;
}

/**
  * @brief  Set time to reset system
  * @param  time_ms
  * @retval None
  */
void TimeOut_Reset(uint32_t time_ms)
{
	/* If IWDG reset orcur, flag will be set */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
		RCC_ClearFlag();

	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/4 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* set Reload value is 1 => reset period = 1*4/(LSI frequency) */
	IWDG_SetReload(time_ms);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
	
}
/*****************************END OF FILE*****************************/

