#include "stm32f0xx_rcc.h"

#ifndef __BLUE_UART_H__
#define __BLUE_UART_H__

#include "main.h"

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>
#include <string.h>


/* ================ USART Communication boards Interface ================ */
#define BLE_USARTx						USART1
#define BLE_USARTx_IRQn       			USART1_IRQn
#define BLE_USARTx_IRQHandler			USART1_IRQHandler

#define BLE_USARTx_TX_PIN				GPIO_Pin_9                
#define BLE_USARTx_TX_GPIO_PORT			GPIOA                       
#define BLE_USARTx_TX_GPIO_CLK			RCC_AHBPeriph_GPIOA
#define BLE_USARTx_TX_SOURCE			GPIO_PinSource9
#define BLE_USARTx_TX_AF				GPIO_AF_1

#define BLE_USARTx_RX_PIN				GPIO_Pin_10               
#define BLE_USARTx_RX_GPIO_PORT			GPIOA              
#define BLE_USARTx_RX_GPIO_CLK			RCC_AHBPeriph_GPIOA
#define BLE_USARTx_RX_SOURCE			GPIO_PinSource10
#define BLE_USARTx_RX_AF				GPIO_AF_1

#define BLE_MD_RESET_PIN				GPIO_Pin_1
#define BLE_SWDCLK_PIN					GPIO_Pin_0
#define BLE_MD_RESET_PORT				GPIOA              
#define BLE_MD_RESET_PORT_CLK			RCC_AHBPeriph_GPIOA

#define CRC_16_CCITT_POLYNOMIAL			0x1021


/* =============== Contanst indicate States or Processes ================ */

#define MAX_LEN_PAYLOAD_FORMART_HEX		18
#define MAX_LEN_PACKET_FORMAT_CHAR	54
#define LEN_NOTIFI_BLE	9

typedef enum { 
	NONE, 			//0
	NCMGR, 			//1
	NPMGR, 			//2
	NNCONN, 		//3
	NNDISCON, 		//4
	NMODE,			//5
	OK,				//6
	ERROr			//7
} RecentNotification_TypeDef;

typedef enum {
	MODE_CONFIG, 	//0
	MODE_NODE,	 	//1
	MODE_GATE		//2
} ModeID_TypeDef;

typedef enum {
	DONE,			//0
	NOTIFICATION,	//1
	LENGTHRX,		//2
	DATA,			//3
	MODEID,			//4
	NODEID,			//5
} processOfBLE;

typedef enum {
	STOPRECEIVING = 0, 
	RECEIVING = !STOPRECEIVING 
} stateOfReceiver;

typedef enum{
	TRANS_AT_NONE,
	TRANS_AT_FOR_NODE,
	TRANS_AT_FOR_CENTRAL,
	TRANS_AT_FOR_BLE
} TransAT_TypeDef;

/* ========================== Declare Functions ========================== */
void BLE_USARTConfig(void);
void BLE_USART_RX_IRQ(void);
void BLE_USART_TX_IRQ(void);

void BLE_Parsing(void);
void String2Hex(void);
void resetParsingProcess(void);

RecentNotification_TypeDef BLE_ReturnRecentNotificationAT(void);
uint8_t BLE_GetNodeID(void);
ModeID_TypeDef BLE_GetModeID(void);
void BLE_ReceivedFromCentral(uint8_t* BLE_lengthData, uint8_t* arrData);
void BLE_ReceivedFromNode(uint8_t* BLE_nodeID, uint8_t* BLE_lengthData, uint8_t* arrData);
uint16_t CRCCalculator(uint8_t len, uint8_t* arrCRC);

void BLE_SendNewAT(void);
void BLE_ReSendAT(void);
void BLE_SendReset(void);
void BLE_Send2Node(uint8_t nodeID, uint8_t lengthPayload, uint8_t* arrayPayload);
void BLE_Send2Central(uint8_t lengthPayload, uint8_t* arrayPayload);
void BLE_Response(ErrorStatus flag);
void BLE_SetFlagTransATComplete(void);
void BLE_QueryCurrentMode(void);
void Hex2String(uint8_t lengthPayload, uint8_t* arrayPayload);
TransAT_TypeDef GetStatusRecentAT(void);
#endif
