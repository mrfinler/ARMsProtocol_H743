/*
 * ARMs_Protocol.c
 *
 *  Created on: Jan 21, 2022
 *      Author: Natmatee
 */
/*
 ******************************************************************************
 * @file    ARMs_Protocol.c
 * @brief   This file provides code for Interface UART and Communication
 * 			with Manipulator.
 * 			License by Narwhal & Zhu Corp
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <ARMsProtocol.h>
#include "usart.h"
#include "crc.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
ARMsProtocol_DATA ARMsProtocol_Data;
ARMsProtocol_HandleTypedef ARMsProtocol_h1;


/*=============================================================================*/
/**
 * @brief	Initial Function : Initial the parameters that use in this library
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Init(void){
	// setting ARMsProtocol_h1
	ARMsProtocol_h1.handle = &huart3;
	ARMsProtocol_h1.Instance = USART3;
	ARMsProtocol_h1.slave_id = 0;

	//setting ARMsProtocol_Data
	ARMsProtocol_Data.Code = 0;
	ARMsProtocol_Data.State = 0;
	ARMsProtocol_Data.Count = 0;

	// Enable UART IT
	HAL_UART_Receive_IT(ARMsProtocol_h1.handle, (uint8_t*) &ARMsProtocol_Data.Rx_reg, 1);
}


/*=============================================================================*/
/**
 * @brief	Interface Function : Data frame checking and state machine
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Interface(void){
	ARMsProtocol_Data.Flag = 1;
	if(ARMsProtocol_Data.Flag  == 1){
		//check header
		if(ARMsProtocol_Data.State == 0){
			ARMsProtocol_Data.Header = ARMsProtocol_Data.Rx_buf[0];
			if(ARMsProtocol_Data.Header == ARMsProtocol_HEADER){
				ARMsProtocol_Data.State = 1;
			}
		}
		//check slave id
		if(ARMsProtocol_Data.State == 1){
			ARMsProtocol_Data.Id = ARMsProtocol_Data.Rx_buf[1];
			if(ARMsProtocol_Data.Id == ARMsProtocol_h1.slave_id){
				ARMsProtocol_Data.Instruction = ARMsProtocol_Data.Rx_buf[2];
				ARMsProtocol_Data.Length = ARMsProtocol_Data.Rx_buf[3];
				ARMsProtocol_Data._CRC = ARMsProtocol_Data.Rx_buf[3 + ARMsProtocol_Data.Length];
				ARMsProtocol_Data.State = 2;
			}
		}
		// check crc
		if(ARMsProtocol_Data.State == 2){
			ARMsProtocol_CALC_CRC((uint32_t *) &ARMsProtocol_Data.Rx_buf[2], ARMsProtocol_Data.Length + 1);
			if(ARMsProtocol_Data._CRC == ARMsProtocol_Data.CRC_CAL){
				for(int i = 0;i <= ARMsProtocol_Data.Length - 2;i++){
					ARMsProtocol_Data.Data_buf[i] = ARMsProtocol_Data.Rx_buf[i+4];
				}
				ARMsProtocol_Data.State = 3;
			}
			else{
				ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ILLEGALCRC);
			}
			ARMsProtocol_FUNC_Rx_Clrbuf(ARMsProtocol_Data.Count);
			ARMsProtocol_Data.Count = 0;
		}
		if(ARMsProtocol_Data.State == 3){
			switch(ARMsProtocol_Data.Instruction){
			/* USER CODE BEGIN 0 */
			case ARMsProtocol_ADDR_SETHOME:
				ARMsProtocol_FUNC_Sethome();
				break;
			case ARMsProtocol_ADDR_JOINTJOG:
				ARMsProtocol_FUNC_Jointjog();
				break;
			case ARMsProtocol_ADDR_CATESIANJOG:
				ARMsProtocol_FUNC_Catesianjog();
				break;
			case ARMsProtocol_ADDR_RECIEVETRAJECTORY:
				ARMsProtocol_FUNC_Recievetrajectory();
				break;
			case ARMsProtocol_ADDR_CONTROLGRIPPER:
				ARMsProtocol_FUNC_Controlgripper();
				break;
			case ARMsProtocol_ADDR_SETZEROENCODER:
				ARMsProtocol_FUNC_Setzeroencoder();
				break;
			/* USER CODE END 0 */
			default:
				ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ILLEGALFUNC);
			}
		}
		ARMsProtocol_Data.Flag  = 0;
		ARMsProtocol_Data.State = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Recieve Callback Function
 * @param  	huart : UART_TypeDef of USART
 * @retval 	None
 */
void ARMsProtocol_FUNC_Rx_Callback(UART_HandleTypeDef *huart){
	if (huart->Instance == ARMsProtocol_h1.Instance) {
		ARMsProtocol_Data.Rx_buf[ARMsProtocol_Data.Count++] = ARMsProtocol_Data.Rx_reg;
			if (ARMsProtocol_Data.Count >= 100) {
				ARMsProtocol_Data.Count = 0;
			}
		HAL_UART_Receive_IT(huart, &ARMsProtocol_Data.Rx_reg, 1);
	}
}


/*=============================================================================*/
/**
 * @brief	Transmit Callback Function
 * @param  	huart : UART_TypeDef of USART
 * @retval 	None
 */
void ARMsProtocol_FUNC_Tx_Callback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit_IT(huart, ARMsProtocol_Data.Tx_buf, sizeof(ARMsProtocol_Data.Tx_buf));
}


/*=============================================================================*/
/**
 * @brief	Exception Response function
 * @param  	huart : UART_TypeDef of USART
 * 			code  : Report Code -> 	0x01 = ARMsProtocol_TRANSMIT_ILLEGALFUNC
 * 									0x02 = ARMsProtocol_TRANSMIT_ILLEGALCRC
 * 									0x03 = ARMsProtocol_TRANSMIT_ACKNOWLEDGE
 * 									0x04 = ARMsProtocol_TRANSMIT_DONE
 * @retval 	None
 */
void ARMsProtocol_EXCEPTION_Response(UART_HandleTypeDef *huart, uint8_t code){
	ARMsProtocol_Data.Tx_buf[0] = 0xFF;
	ARMsProtocol_Data.Tx_buf[1] = ARMsProtocol_h1.slave_id;
	ARMsProtocol_Data.Tx_buf[2] = code;
	ARMsProtocol_CALC_CRC((uint32_t*) &ARMsProtocol_Data.Tx_buf, sizeof(ARMsProtocol_Data.Tx_buf)-1);
	ARMsProtocol_Data.Tx_buf[3] = ARMsProtocol_Data.CRC_CAL;
	ARMsProtocol_FUNC_Tx_Callback(huart);
}


/*=============================================================================*/
/**
 * @brief	CRC Calculation Function
 * @param  	*nData  : Data
 * 			wLength : Length of Data
 * @retval 	None
 */
void ARMsProtocol_CALC_CRC (uint32_t *pBuffer, uint32_t BufferLength)
{
	ARMsProtocol_Data.CRC_CAL = HAL_CRC_Calculate(&hcrc, pBuffer, BufferLength) ^ 0xFF;
}


/*=============================================================================*/
/**
 * @brief	Clear Recieve Buffer
 * @param  	count : amount of data in Rx_buf
 * @retval 	None
 */
void ARMsProtocol_FUNC_Rx_Clrbuf(uint8_t count){
	for(int i =0; i <= count - 1; i++){
		ARMsProtocol_Data.Rx_buf[i] = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Clear Data Buffer
 * @param  	count : amount of data in Data_buf
 * @retval 	None
 */
void ARMsProtocol_FUNC_Data_Clrbuf(){
	for(int i =0; i <= ARMsProtocol_Data.Length; i++){
		ARMsProtocol_Data.Data_buf[i] = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Sethome Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Sethome(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Jointjog Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Jointjog(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Catesianjog Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Catesianjog(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 4 */

	/* USER CODE END 4 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Recievetrajectory Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Recievetrajectory(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 5 */

	/* USER CODE END 5 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Controlgripper Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Controlgripper(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 6 */

	/* USER CODE END 6 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Setzeroencoder Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Setzeroencoder(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 7 */

	/* USER CODE END 7 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}
