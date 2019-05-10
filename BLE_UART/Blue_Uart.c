#include "Blue_Uart.h"
#include "user.h"
extern const uint8_t arrResponseOK[];
extern const uint8_t arrResponseERROR[];


/* Receiver */
uint8_t	dataUSART;
FlagStatus Flag_TxComplete 		= SET;
char		BLE_notificationAT[LEN_NOTIFI_BLE];
char*		BLE_pNotificationAT = BLE_notificationAT;

stateOfReceiver	BLE_stateOfReceiver = STOPRECEIVING;
uint8_t	BLE_CRLFCounter = 2;

processOfBLE	BLE_processOfBLE;

/* TX - Transmit payload */
uint8_t	BLE_lengthPayloadTX;
uint8_t	BLE_lengthBufferPayloadTX;
uint8_t	BLE_payloadTXCounter;

FlagStatus Flag_TransATComplete = SET;
TransAT_TypeDef Flag_NewTransAT		= TRANS_AT_NONE;
TransAT_TypeDef flag_recentSendAT;
//char		BLE_payloadTXHex[18];
char		BLE_payloadTXString[MAX_LEN_PAYLOAD_FORMART_HEX*2];

//char*		BLE_pPayloadTX = BLE_payloadTXHex;
char*		BLE_pPayloadTXString = BLE_payloadTXString;

char		BLE_arrayBufferForSendingCompletely[MAX_LEN_PACKET_FORMAT_CHAR];
char		BLE_stringBufferSendNewAT[MAX_LEN_PACKET_FORMAT_CHAR];
char*		BLE_pArrayBufferForSendingCompletely = BLE_arrayBufferForSendingCompletely;
	
/* RX - Receive payload */
uint8_t	BLE_lengthPayloadRX;
uint8_t	BLE_payloadRXCounter;


char		BLE_payloadRXHex[MAX_LEN_PAYLOAD_FORMART_HEX];
char		BLE_payloadRXString[MAX_LEN_PAYLOAD_FORMART_HEX+1];		// why +1 ?

char*		BLE_pPayloadRX = BLE_payloadRXHex;
char*		BLE_pPayloadRXString = BLE_payloadRXString;

char		BLE_arrayWaitingForStopReceiving[MAX_LEN_PACKET_FORMAT_CHAR + 1];	// why + 1
char*		BLE_pArrayWaitingForStopReceiving = BLE_arrayWaitingForStopReceiving;

/* Node ID/Mode ID */
ModeID_TypeDef	BLE_modeID = MODE_CONFIG;
uint8_t	BLE_nodeID;

/* Flags */
uint8_t newNotificaion; 		/*if new AT Notification was received, this flag will be RESET*/
RecentNotification_TypeDef BLE_recentNotificationAT; /* the lastest AT Notification*/


/**
  * @brief  Configure USART1, RCC, GPIO, NVIC peripheral for control module Bluetooth
  * @param  None
  * @retval None
  */

void BLE_USARTConfig(void){
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

	
/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
	
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(BLE_USARTx_TX_GPIO_CLK | BLE_USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	
/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
 
 
  /* Connect PXx to reset pins of module Bluetooth */  
  RCC_AHBPeriphClockCmd(BLE_MD_RESET_PORT_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = BLE_MD_RESET_PIN | BLE_SWDCLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(BLE_MD_RESET_PORT, &GPIO_InitStructure);

  /* Write Reset pin of Module Bluetooth to Low */
  GPIO_WriteBit(BLE_MD_RESET_PORT, BLE_MD_RESET_PIN, Bit_RESET);
  /* While Module during reset, SWDCLK pin must be kept Low */
  GPIO_WriteBit(BLE_MD_RESET_PORT, BLE_SWDCLK_PIN, Bit_RESET);
  
  
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(BLE_USARTx_TX_GPIO_PORT, BLE_USARTx_TX_SOURCE, BLE_USARTx_TX_AF);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(BLE_USARTx_RX_GPIO_PORT, BLE_USARTx_RX_SOURCE, BLE_USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = BLE_USARTx_TX_PIN | BLE_USARTx_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(BLE_USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */

  /* NVIC configuration: Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = BLE_USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Prepare uart to receive a data packet */
	USART_Init(BLE_USARTx, &USART_InitStructure); 
   /* Enable 8xUSARTs Receive interrupts */
   USART_ITConfig(BLE_USARTx, USART_IT_RXNE, ENABLE);
	/* Enable the 8xUSARTs */
   USART_Cmd(BLE_USARTx, ENABLE);

   
   /* After USART Configuration Write RESET pin and SWDCLK of module to High to finish reset module BLE_process */
	GPIO_WriteBit(BLE_MD_RESET_PORT, BLE_MD_RESET_PIN, Bit_SET);
	GPIO_WriteBit(BLE_MD_RESET_PORT, BLE_SWDCLK_PIN, Bit_SET);
}


/**
  * @brief  USART1 Receiver Interrupt Request
  * @param  None
  * @retval None
  */
void BLE_USART_RX_IRQ(void){
	dataUSART = USART_ReceiveData(USART1);
	
	if(BLE_stateOfReceiver == RECEIVING){
		*BLE_pArrayWaitingForStopReceiving = dataUSART;
		BLE_pArrayWaitingForStopReceiving++;
		
		if((dataUSART == 'R' && *(BLE_pArrayWaitingForStopReceiving - 2) == 'O')|| dataUSART == 'K'){
			BLE_stateOfReceiver = STOPRECEIVING;
		}
		/* End if */
	}
	else{	
		/* BLE_stateOfReceiver == STOPRECEIVING; */
		if(dataUSART == '\r' || dataUSART == '\n')
			BLE_CRLFCounter++;
		if(BLE_CRLFCounter == 2)
			BLE_Parsing();
		if(BLE_CRLFCounter == 4){
			BLE_stateOfReceiver = RECEIVING;
			BLE_CRLFCounter = 0;
			resetParsingProcess();

		}
		/* End if */
	}
}


/**
  * @brief  Parse syntaxs of Notification AT Response were received from module Bluetooth
  * @param  None
  * @retval None
  */
void BLE_Parsing(void){
	if(dataUSART == 'R')
	/* NRF52 notify ERROR to MCU */
		BLE_recentNotificationAT = ERROr;
		
	else{
	/* NRF52 notify OK to MCU */
		
		if(BLE_arrayWaitingForStopReceiving[0] == 'O')
			BLE_recentNotificationAT = OK;
		else {
			BLE_pArrayWaitingForStopReceiving = BLE_arrayWaitingForStopReceiving;
			while(BLE_processOfBLE != DONE) {
				BLE_pArrayWaitingForStopReceiving++;
				switch(*BLE_pArrayWaitingForStopReceiving) {
					case ':': {
					/*Parse characters in String AT Notification Responses*/
						if(strcmp("NCMGR",BLE_notificationAT) == 0) {
							BLE_recentNotificationAT = NCMGR;
							BLE_processOfBLE = LENGTHRX;
							break;
						}
						
						if(strcmp("NPMGR",BLE_notificationAT) == 0) {
							BLE_recentNotificationAT = NPMGR;
							BLE_processOfBLE = NODEID;
							break;
						}
						
						if(strcmp("NNCONN",BLE_notificationAT) == 0) {
							BLE_recentNotificationAT = NNCONN;
							BLE_processOfBLE = NODEID;
							break;
						}
						
						if(strcmp("NNDISCON",BLE_notificationAT) == 0) {
							BLE_recentNotificationAT = NNDISCON;
							BLE_processOfBLE = NODEID;
							break;
						}
						
						if(strcmp("NMODE",BLE_notificationAT) == 0) {
							BLE_recentNotificationAT = NMODE;
							BLE_processOfBLE = MODEID;
							break;
						}
					}
					
					case ',': {
						if (BLE_processOfBLE == LENGTHRX)
							BLE_processOfBLE = DATA;
						else if (BLE_processOfBLE == NODEID)
							BLE_processOfBLE = LENGTHRX;
						/* End if */
						break;
					}
						
					default: {
						switch(BLE_processOfBLE) {
							
							/* MCU is receiving Notification AT from NRF52 */
							case NOTIFICATION: {
								*BLE_pNotificationAT = *BLE_pArrayWaitingForStopReceiving;
								BLE_pNotificationAT++;
								break;
							}
							
							/* MCU is receiving Length of Data from NRF52 */
							case LENGTHRX: {
								if (BLE_lengthPayloadRX == 0)
									BLE_lengthPayloadRX = *BLE_pArrayWaitingForStopReceiving - '0';
								else BLE_lengthPayloadRX = BLE_lengthPayloadRX * 10 + *BLE_pArrayWaitingForStopReceiving - '0';
								break;
							}
							
							/* MCU is receiving Data as String from NRF52 */
							case DATA: {
								*BLE_pPayloadRXString = *BLE_pArrayWaitingForStopReceiving;
								BLE_pPayloadRXString++;
								BLE_payloadRXCounter++;
								if (BLE_payloadRXCounter >= (2 * BLE_lengthPayloadRX)) {
									String2Hex();
									BLE_processOfBLE = DONE;
									//uint16_t receivedCRC = ((uint16_t) BLE_payloadRXHex[BLE_lengthPayloadRX-2]<<8) + BLE_payloadRXHex[BLE_lengthPayloadRX-1];
									//if( CRCCalculator(BLE_lengthPayloadRX - 2 ,(uint8_t*)BLE_payloadRXHex) == receivedCRC)
									//	receivedCompleteData = COMPLETE;
									//	else
									//	{ sendERROR(); receivedCompleteData = UNCOMPLETED; }
								}
								break;
							}
								
							/* MCU is receiving Mode ID from NRF52 */
							case MODEID: {
								BLE_modeID = (ModeID_TypeDef)(*BLE_pArrayWaitingForStopReceiving - '0');
								BLE_processOfBLE = DONE;

								break;
							}
							
							/* MCU is receiving Node ID from NRF52 */
							case NODEID: {
								if (BLE_recentNotificationAT == NPMGR) {
									if (BLE_nodeID == 0)
										BLE_nodeID = *BLE_pArrayWaitingForStopReceiving - '0';
									else BLE_nodeID = BLE_nodeID * 10 + *BLE_pArrayWaitingForStopReceiving - '0';
								}
								else if (BLE_recentNotificationAT == NNCONN || BLE_recentNotificationAT == NNDISCON){
									if (BLE_nodeID == 0)
										BLE_nodeID = *BLE_pArrayWaitingForStopReceiving - '0';
									else if (*BLE_pArrayWaitingForStopReceiving == 0x0D)
										BLE_processOfBLE = DONE;
									else BLE_nodeID = BLE_nodeID * 10 + *BLE_pArrayWaitingForStopReceiving - '0';
								}
								/* End if */
								break;
							}
							
							/* Default */
							default:
								break;
						}
							
						break;
					}
				}
			}
		}
	}
}


/**
* @brief  Convert String was just received
* @param None
* @retval None
*/
void String2Hex(){
	uint8_t innerCounter;
	for (innerCounter = 0; innerCounter<36; innerCounter++)
		BLE_payloadRXString[innerCounter] = (BLE_payloadRXString[innerCounter]  > 0x2F && BLE_payloadRXString[innerCounter] < 0x3A) ? (BLE_payloadRXString[innerCounter] - '0') : (BLE_payloadRXString[innerCounter] > 0x40 && BLE_payloadRXString[innerCounter] < 0x47) ? (BLE_payloadRXString[innerCounter] - 'A' + 0xA) : 0;
	
	for (innerCounter = 0; innerCounter < 18; innerCounter++) 
		BLE_payloadRXHex[innerCounter] = BLE_payloadRXString[2 * innerCounter] * 0x10 + BLE_payloadRXString[2 * innerCounter + 1];
}


/**
  * @brief  Reset Strings, Arrays, Variables for Bluetooth control
  * @param  None
  * @retval None
  */
void resetParsingProcess(void){
	uint8_t innerCounter;
	for (innerCounter = 0; innerCounter<37; innerCounter++)
		*(BLE_payloadRXString + innerCounter) = '\0';
	for (innerCounter = 0; innerCounter<9; innerCounter++)
		*(BLE_notificationAT + innerCounter) = '\0';
	for (innerCounter = 0; innerCounter<18; innerCounter++)
		*(BLE_payloadRXHex + innerCounter) = '\0';
	for (innerCounter = 0; innerCounter<55; innerCounter++)
		*(BLE_arrayWaitingForStopReceiving + innerCounter) = '\0';
	
	BLE_pPayloadRX = BLE_payloadRXHex;
	BLE_pNotificationAT = BLE_notificationAT;
	BLE_pPayloadRXString = BLE_payloadRXString;
	BLE_pArrayWaitingForStopReceiving = BLE_arrayWaitingForStopReceiving;
	
	BLE_processOfBLE = NOTIFICATION;
	
	BLE_nodeID = 0;
	BLE_modeID = MODE_CONFIG;
	
	BLE_payloadRXCounter = 0;
	BLE_lengthPayloadRX = 0;
}


/**
  * @brief  Get recent notification AT
  * @param  None
  * @retval enum RecentNotification_TypeDef
  */
RecentNotification_TypeDef BLE_ReturnRecentNotificationAT(void){
	if((BLE_recentNotificationAT !=NONE) && (BLE_CRLFCounter == 2)){
		RecentNotification_TypeDef inner = BLE_recentNotificationAT;
		BLE_recentNotificationAT = NONE;
		return inner;
	}
	else return NONE;
}

/**
  * @brief  Get Node ID of node connect, disconnect or send Data to Gateway
  * @param  None
  * @retval Node ID
  */
uint8_t BLE_GetNodeID(void){
	BLE_recentNotificationAT = NONE;
	return BLE_nodeID;
}
	
	
/**
  * @brief  Get current Mode ID of module Bluetooth
  * @param  None
  * @retval Mode ID
  */
ModeID_TypeDef BLE_GetModeID(void){
	BLE_recentNotificationAT = NONE;
	return BLE_modeID;
}


/**
  * @brief  Transmit hexa decimal Data wwas receiced from Central to main function
  * @param  Length of Data, arrayData
  * @retval None
  */
void BLE_ReceivedFromCentral(uint8_t* BLE_lengthData, uint8_t* arrData) {
	uint8_t innerCounter;
	BLE_recentNotificationAT = NONE;
	*BLE_lengthData = BLE_lengthPayloadRX;
	for (innerCounter = 0; innerCounter < BLE_lengthPayloadRX; innerCounter++) 
		arrData[innerCounter] = BLE_payloadRXHex[innerCounter];
}


/**
  * @brief  Transmit hexa decimal Data wwas receiced from Node to main function
  * @param  Node ID, Length of Data, arrayData
  * @retval None
  */
void BLE_ReceivedFromNode(uint8_t* BLEnodeID, uint8_t* BLE_lengthData, uint8_t* arrData){
	uint8_t innerCounter;
	BLE_recentNotificationAT = NONE;
	*BLE_lengthData = BLE_lengthPayloadRX;
	*BLEnodeID = BLE_nodeID;
	for (innerCounter = 0; innerCounter < BLE_lengthPayloadRX; innerCounter++) 
		arrData[innerCounter] = BLE_payloadRXHex[innerCounter];
}


/**
  * @brief  Calculate CRC from received array and compare with received CRC
  * @param  pointer of received array format HexaDecimal
  * @retval None
  */
uint16_t CRCCalculator(uint8_t len, uint8_t* arrCRC){
	uint16_t valueCRC = 0xffff;
	uint8_t counter = 0, BLE_DataHexArrayCounter;

	
	for (BLE_DataHexArrayCounter = 0; BLE_DataHexArrayCounter < len; BLE_DataHexArrayCounter++)
	{
		valueCRC ^= arrCRC[BLE_DataHexArrayCounter];
		do
		{
			if (valueCRC & 0x8000){
				valueCRC <<= 1;
				valueCRC ^= CRC_16_CCITT_POLYNOMIAL;
			}
			else
				valueCRC <<= 1;
			counter++;
		} while (counter < 16);
		counter = 0;
	}
	return valueCRC;
//	uint16_t receivedCRC = ((uint16_t) tR[BLE_DataHexArrayCounter]<<8) + tR[BLE_DataHexArrayCounter+1];
//	if(valueCRC == receivedCRC)
//		receivedCompleteData = COMPLETE;
//	else
//	{ sendERROR(); receivedCompleteData = UNCOMPLETED; }
	
}


/**
  * @brief  USART1 Transmitter Interrupt Request
  * @param  None
  * @retval None
  */
void BLE_USART_TX_IRQ(void){
	BLE_pArrayBufferForSendingCompletely++;
	
	if(*BLE_pArrayBufferForSendingCompletely != '\0')
		USART_SendData(USART1, *BLE_pArrayBufferForSendingCompletely);
	else 
	{
		USART_ITConfig(BLE_USARTx, USART_IT_TXE, DISABLE);
		Flag_TxComplete = SET;
	}
};	



/**
  * @brief  Send Data to Central (Gateway or Smartphone)
  * @param  Leng of payload, Hexa-Decimal array payload
  * @retval None
  */
void BLE_SendNewAT(void){

	if( Flag_TransATComplete == RESET )
		return;
	if( Flag_NewTransAT == TRANS_AT_NONE )
		return;
		
	//else if( (Flag_TransATComplete == SET) && (Flag_NewTransAT == SET) )
	sprintf(BLE_arrayBufferForSendingCompletely, "%s" ,BLE_stringBufferSendNewAT);
	
	BLE_pArrayBufferForSendingCompletely = BLE_arrayBufferForSendingCompletely;
	
	USART_SendData(USART1, *BLE_pArrayBufferForSendingCompletely);
	USART_ITConfig(BLE_USARTx, USART_IT_TXE, ENABLE);
	
	if( (Flag_NewTransAT == TRANS_AT_FOR_NODE) && (GetFlagRestartForward() == SET) )
		SetFlagRestartForward(RESET);
	flag_recentSendAT = Flag_NewTransAT;
	Flag_NewTransAT = TRANS_AT_NONE;
	Flag_TransATComplete = RESET;
}
/**
  * @brief  Send AT+NRST to reset module by software
  * @param  None
  * @retval None
  */
void BLE_SendReset(void){
	sprintf(BLE_stringBufferSendNewAT,"AT+NRST\r");	
	Flag_NewTransAT = TRANS_AT_FOR_BLE;
	
	BLE_SendNewAT();
}

/**
  * @brief  Send AT+NGMODE to query Mode ID
  * @param  None
  * @retval None
  */
void BLE_QueryCurrentMode(void){
	sprintf(BLE_stringBufferSendNewAT,"AT+NGMODE\r");	
	Flag_NewTransAT = TRANS_AT_FOR_BLE;
	
	BLE_SendNewAT();
}

/**
  * @brief  Send Data to specified Node
  * @param  Node ID, Leng of payload, Hexa-Decimal array payload
  * @retval None
  */
void BLE_Send2Node(uint8_t nodeID, uint8_t lengthPayload, uint8_t* arrayPayload){
	
	Hex2String(lengthPayload, arrayPayload);
	sprintf(BLE_stringBufferSendNewAT, "AT+NPMGS=%u,%u,%s\r", nodeID, lengthPayload, BLE_payloadTXString);
	
	Flag_NewTransAT = TRANS_AT_FOR_NODE;
	
	BLE_SendNewAT();
	
}

void BLE_Send2Central(uint8_t lengthPayload, uint8_t* arrayPayload)
{
	Hex2String(lengthPayload, arrayPayload);
	sprintf(BLE_stringBufferSendNewAT, "AT+NCMGS=%u,%s\r" , lengthPayload, BLE_payloadTXString);
	
	Flag_NewTransAT = TRANS_AT_FOR_CENTRAL;
	
	BLE_SendNewAT();
}

void BLE_ReSendAT(void)
{		
	if( Flag_TransATComplete == RESET )
		return;
	
	BLE_pArrayBufferForSendingCompletely = BLE_arrayBufferForSendingCompletely;
	
	USART_SendData(USART1, *BLE_pArrayBufferForSendingCompletely);
	USART_ITConfig(BLE_USARTx, USART_IT_TXE, ENABLE);
	
	Flag_TransATComplete = RESET;
}
void BLE_SetFlagTransATComplete(void)
{
	Flag_TransATComplete = SET;
}
/**
  * @brief  Tranform Hexa-Decimal array payload to String payload to send
  * @param  Leng of payload, Hexa-Decimal array payload
  * @retval None
  */
void Hex2String(uint8_t lengthPayload, uint8_t* arrayPayload){
	uint8_t i;
	BLE_payloadTXString[0] = '\0';

	for( i = 0; i < lengthPayload; i++ )
		sprintf(BLE_payloadTXString,"%s%02X", BLE_payloadTXString, arrayPayload[i]);
}

/**
  * @brief  Response from BLE, this function is used by main function
  * @param  ErrorStatus flag;
  * @retval None
  */
void BLE_Response(ErrorStatus flag){
	
	if(flag == SUCCESS)
		sprintf((char*)BLE_stringBufferSendNewAT, "AT+NCMGS=4,%s\r",arrResponseOK);
	else
		sprintf((char*)BLE_stringBufferSendNewAT, "AT+NCMGS=4,%s\r",arrResponseERROR);
	
	Flag_NewTransAT = TRANS_AT_FOR_CENTRAL;
	
	BLE_SendNewAT();
}


TransAT_TypeDef GetStatusRecentAT(void)
{
	return flag_recentSendAT;
}



