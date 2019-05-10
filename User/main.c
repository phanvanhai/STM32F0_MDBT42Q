
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user.h"
#include "Blue_Uart.h"

/* Uncomment the corresponding line if this is the firt time run */
//#define FIRT_TIME_RUN

/* Private variables ---------------------------------------------------------*/


void GPIO_WriteReverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->ODR ^= (uint8_t)GPIO_Pin;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	#if FIRT_TIME_RUN
		Flash_Default(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR, FLASH_PAGE_SIZE);
		while(1);
	#else
		PACKET_CmdTypeDef cmd;
	
	/* Configuration */
		_RTC_Config();
		TIM_Config();
		BLE_USARTConfig();

	/* Read data stored in Flash */
		Flash2PinMAC();
		Flash2Arr();

	/* Declare variable */
		uint8_t						arrData[MAX_LEN_PAYLOAD_FORMART_HEX];
		uint8_t 					lenData 			= 0;
		ErrorStatus					flag_signIn 		= ERROR;
		ErrorStatus					flag;
	
		FlagStatus					flag_ControlForIt	= SET;
		MAC_TypeDef 				myMAC;
		MAC_TypeDef 				MACNode;
		
		TransAT_TypeDef 			flag_recentSendAT 	= TRANS_AT_NONE;
		ModeID_TypeDef 				mode 				= MODE_CONFIG;
		RecentNotification_TypeDef	notifi 				= NONE;
		uint8_t 					idNode;
		uint8_t 					count_error 		= 0;

		flag_signIn = SUCCESS;
		
		TimeOut_Reset(1250*2);
		while(1)
		{
			notifi = BLE_ReturnRecentNotificationAT();
			if( notifi != NONE )
			{
				switch( mode )
				{
					case MODE_CONFIG:
						switch( notifi )
						{
							case NCMGR:
								BLE_ReceivedFromCentral( &lenData, arrData );
								cmd	= GetCMD( arrData );
								
								switch(cmd)
								{
									case CMD_SIGNIN:			// only use one time when app adds new device
									{
										flag = SUCCESS;
										flag_signIn = CheckSignIn(arrData);
										BLE_Response(flag_signIn);
									}
									break;
																	
									case CMD_SETPIN_MAC:		// only use when user config the device
									{
										flag = SUCCESS;
										
										flag = PinMAC2Flash(arrData);
										BLE_Response(flag);
										if( flag == ERROR )
										{
											while(1);
										}
									}
									break;
									
									case CMD_SETTIME:
									{
										SetTimeCurrent(arrData);									
										UpdateSetAlarm();
									}
									break;								
									
									case CMD_DISPLAY:
										Illuminate_Normal(arrData);
									break;
									
									default:
									break;
								}
							break;

							case OK:
								BLE_SetFlagTransATComplete();
								BLE_SendNewAT();
							break;
							
							case ERROr:
								BLE_SetFlagTransATComplete();
								BLE_ReSendAT();
							break;
							
							case NMODE:
								mode = BLE_GetModeID();
							break;
							
							default:
								break;
						}
					break;
				
					
					case MODE_NODE:
						switch( notifi )
						{
							case NCMGR:
							{
								BLE_ReceivedFromCentral( &lenData, arrData );
								cmd	= GetCMD( arrData );
								switch(cmd)
								{
									case CMD_SETTIME:
									{
										SetTimeCurrent(arrData);									
										UpdateSetAlarm();
										break;								
									}
									case CMD_DISPLAY:
									{
										Illuminate_Normal(arrData);
									}
									break;
									case CMD_STARTALARM:
									{
										SetSizeArrAlarm(arrData);
									}
									break;
									case CMD_SETALARM:
									{
										if( CheckReceiveAllAlarm() == RESET )
										{
											UpdateArrAlarm(arrData);									
											if( CheckReceiveAllAlarm() == SET )
											{
												SortAlarm();
												flag = Arr2Flash();
												// reset MCU
												if( flag == ERROR )
												{
													while(1);
												}
												else
													UpdateSetAlarm();	
											}
										}
									}
									break;
										
									default:
										break;
								}
							}
							break;
							
							case OK:
								BLE_SetFlagTransATComplete();
								BLE_SendNewAT();
							break;
							
							case ERROr:
								BLE_SetFlagTransATComplete();
								BLE_ReSendAT();
							break;
							
							case NMODE:
								mode = BLE_GetModeID();
							break;
							
							case NNCONN:
								myMAC = GetMAC();
								NotifiConnect2Central( myMAC, SET);	//-----------Gui ma MAC cho Gate khi da ket noi dc voi Gate-------------------
							break;
							
							case NNDISCON:
								BLE_SendReset();	// gui lenh reset module ve che do Config
							break;
							
							default:
							break;
						}
					break;

					case MODE_GATE:
					switch( notifi )
					{
						case NCMGR:
						{
							BLE_ReceivedFromCentral( &lenData, arrData );
							cmd	= GetCMD( arrData );
							
							if( (cmd & 0x0F) != (CMD_CREATE_GROUP & 0x0F) )
							{
								GATE_StartForward2Node(lenData, arrData);		// forward to All Node in Group
								flag_ControlForIt = CheckControlForIt();
								if( CheckForwardComplete() == SET )	
									BLE_Response(SUCCESS);
							}
						
							if( (flag_ControlForIt == RESET) && ((cmd & 0x0F) != (CMD_CREATE_GROUP & 0x0F)) )									
								break;
							// else flag_ControlForIt = SET || CMD_CREATE_GROUP || CMD_ADD_TO_GROUP
							switch( cmd )
							{	
								case CMD_DISPLAY:
								{
									Illuminate_Normal(arrData);
								}
								break;
								
								case CMD_SETTIME:
								{
									SetTimeCurrent(arrData);									
									UpdateSetAlarm();
								}
								break;
								case CMD_STARTALARM:
								{
									SetSizeArrAlarm(arrData);
								}
								break;
																	
								case CMD_SETALARM:
								{
									if( CheckReceiveAllAlarm() == RESET )
									{
										UpdateArrAlarm(arrData);									
										if( CheckReceiveAllAlarm() == SET )
										{
											SortAlarm();
											flag = Arr2Flash();
											if( flag == ERROR )
											{
												while(1);
											}
											else
												UpdateSetAlarm();	
										}
									}
								}
								break;
								case CMD_CREATE_GROUP:
								case CMD_ADD_TO_GROUP:
								{
									flag = SUCCESS;
									flag = GATE_CreateGroup((uint8_t)cmd, lenData, arrData);
									flag_ControlForIt = CheckControlForIt();
									BLE_Response(flag);			// ---------Phan hoi viec tao nhom co thanh cong hay ko?-----------
								}
								break;
								
								default:
								break;
							}
						}								
						break;
						
						case NPMGR:
						{
							BLE_ReceivedFromNode(&idNode, &lenData, arrData);
							cmd	= GetCMD( arrData );
							
							if( cmd == CMD_NOTIFI_CONNECT_FROMM_NODE )
							{
								GATE_AddMAC(idNode, arrData); 
								BLE_Send2Central(lenData, arrData);		//--------forward thong bao co ket noi toi app ---------------
							}
						}
						break;
						
						case OK:
							count_error = 0;
							BLE_SetFlagTransATComplete();
							flag_recentSendAT = GetStatusRecentAT();
						
							if( flag_recentSendAT == TRANS_AT_FOR_NODE )
							{
								if( GetFlagRestartForward() == SET )
								{
									GATE_Forward2Node(ERROR);		//------forward lai tu dau --------------------------
									SetFlagRestartForward(RESET);
								}
								else
									GATE_Forward2Node(SUCCESS);		// tiep tuc forward cho Node
								if( CheckForwardComplete() == SET )
									BLE_Response(SUCCESS);			//------phan hoi cho app khi da forward den tat ca node yeu cau
							}
							else
								BLE_SendNewAT();					//-------gui lenh tiep theo -----------------------------------									
						break;
						
						case ERROr:
							BLE_SetFlagTransATComplete();
							flag_recentSendAT = GetStatusRecentAT();

							count_error++;
							if( count_error > 3 )
							{
								count_error = 0;
								if( flag_recentSendAT == TRANS_AT_FOR_NODE )
								{
									GATE_Forward2Node(SUCCESS);			//-----sau 3 lan loi ->bo qua va toi thiet bi tiep theo
									if( CheckForwardComplete() == SET )
										BLE_Response(ERROR);			//----co thiet bi trong nhom khong dc ket noi -------
								}
							}
							else
								BLE_ReSendAT();
						break;
							
						case NMODE:
							mode = BLE_GetModeID();
						break;
						
						case NNCONN:
							idNode = BLE_GetNodeID();
							GATE_AddIdNode(idNode);
						break;

						case NNDISCON:
							idNode = BLE_GetNodeID();
							MACNode = FindMACFromId(idNode);
							NotifiConnect2Central(MACNode, RESET);	//--------Thong bao cho App biet node co MAC da mat ket noi----------------------
							GATE_RemoveIdNode(idNode);
						break;
													
						default:
						break;
					}
					break;

					default:
						break;
				}
			}
			IWDG_ReloadCounter();	
		}
		
#endif
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
