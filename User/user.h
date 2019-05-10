/**
  ******************************************************************************
  * @file    user.h
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   This file contains all the functions prototypes for the RTC firmware 
  *          library.
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_H
#define __USER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint32_t MACHigh;
	uint16_t MACLow;
} MAC_TypeDef;
/* Exported constants --------------------------------------------------------*/
typedef enum	
{
	CMD_NOTIFI_CONNECT_FROMM_NODE 	= 	0x01,
	CMD_CREATE_GROUP				= 	0x02,
	CMD_ADD_TO_GROUP				= 	0x12,
	CMD_SIGNIN 						=	0x0A,
	CMD_SETTIME						=	0x0B,
	CMD_SETPIN_MAC					=	0x0C,
	CMD_DISPLAY						=	0x0D,
	CMD_STARTALARM					=	0x0E,
	CMD_SETALARM					=	0x1E
	
	
} PACKET_CmdTypeDef;


			/************define Address Flash **************/


#define FLASH_PAGE_SIZE         		((uint32_t)0x00000400)  	/* FLASH Page Size */
#define FLASH_USER_START_ADDR   		((uint32_t)0x08003800)   	/* Start address of user Flash area */
#define FLASH_USER_END_ADDR     		((uint32_t)0x08004000)   	/* End address of user Flash area */

#define FLASH_START_ARR_BLOCK_ADDR		((uint32_t)0x08003800)		/* Start address of the alarm block */
#define FLASH_BLOCK_SIZE				((uint32_t)0x00000200)		/* Size of block*/
#define FLASH_NUMBER_BLOCK				2U							/* Number of block used. Must be multiples of 2 */

#define FLASH_PIN_START_ADDR			((uint32_t)0x08003C00)		/* Page address saved PIN */
#define FLASH_PIN_END_ADDR				((uint32_t)0x08004000)		/* The address at the bottom of the page  */

			/************define periph PWM **************/
#define F_PWM							1000		/* Hz	*/
#define TIMx							TIM3
#define TIM_AHBPERIPHCLOCK				RCC_APB1PeriphClockCmd
#define	TIM_CLK							RCC_APB1Periph_TIM3

#define	TIM_GPIO_CLK					RCC_AHBPeriph_GPIOA
#define TIM_GPIO_PORT					GPIOA

#define	TIM_CH1_PIN						GPIO_Pin_6
#define TIM_CH1_SOURCE					GPIO_PinSource6
#define	TIM_CH1_AF						GPIO_AF_1

#define	TIM_CH2_PIN						GPIO_Pin_7
#define TIM_CH2_SOURCE					GPIO_PinSource7
#define	TIM_CH2_AF						GPIO_AF_1

			/************ define Source CLK RTC **************/
#define RTC_CLOCK_SOURCE_LSI  	// LSI used as RTC source clock. The RTC Clock
                                // may varies due to LSI frequency dispersion
																

/* Exported functions ------------------------------------------------------- */ 

PACKET_CmdTypeDef 	GetCMD(uint8_t* arrData);
										/* Illuminate */
void 				TIM_Config(void);
void 				TIM_SetPWM_CH1_CH2(uint8_t Channel1Pulse, uint8_t Channel2Pulse);
void 				Illuminate_Default(void);
void 				Illuminate_Normal(uint8_t* arrData);
void 				Illuminate_Alarm(void);
										/* RTC - Alarm */
void 				_RTC_Config(void);																	
void 				SetTimeCurrent(uint8_t* arrData);
void 				SetSizeArrAlarm(uint8_t* arrData);
FlagStatus 			CheckReceiveAllAlarm(void);
void 				UpdateArrAlarm(uint8_t* arrData);
void 				UpdateSetAlarm(void);
void 				SortAlarm(void);
										/* Flash */
void 				Flash_Default(uint32_t addr_start, uint32_t addr_end, uint32_t sizepage);
void 				Flash2Arr(void);
ErrorStatus 		Arr2Flash(void);
void 				Flash2PinMAC(void);
ErrorStatus 		PinMAC2Flash(uint8_t* arrData);
										/* SignIn */
ErrorStatus 		CheckSignIn(uint8_t* arrData);
										/* TimeOut */
void 				TimeOut_Reset(uint32_t time_ms);
//void 				TimeOut_Restart(uint32_t min_timeout);
//void 				TimeOut_Stop(void);
//FlagStatus 		TimeOut_CheckFlag(void);
//uint32_t 			DecCountTick(void);
										/* Relate MAC - IdNode*/
void 				GATE_AddMAC(uint8_t idNode, uint8_t* arrData);
void 				GATE_AddIdNode(uint8_t id);
void 				GATE_RemoveIdNode(uint8_t id);
MAC_TypeDef 		GetMAC(void);
MAC_TypeDef 		FindMACFromId(uint8_t idNode);
void 				NotifiConnect2Central(MAC_TypeDef mac, FlagStatus flag_connect);
										/* Create Group */
ErrorStatus 		GATE_CreateGroup(uint8_t cmd, uint8_t len, uint8_t* arrData);
FlagStatus 			CheckControlForIt(void);
										/* Forward to Node*/
void 				GATE_StartForward2Node(uint8_t len, uint8_t* arrData);
void 				GATE_Forward2Node(ErrorStatus flag);
FlagStatus 			GetFlagRestartForward(void);
void 				SetFlagRestartForward(FlagStatus flag);
FlagStatus 			CheckForwardComplete(void);


#ifdef __cplusplus
}
#endif

#endif /* __USER_H */

/*****************************END OF FILE*****************************/

