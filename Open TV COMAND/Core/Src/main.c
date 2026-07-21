/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usb_hid_keys.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = { 0, };
uint8_t RxData[8] = { 0, };
uint32_t TxMailbox = 0;
_Bool RxInput = 0;
_Bool KeyMode = 0;
unsigned char NumTx = 0;
unsigned char NumPress = 0;
unsigned char TxDataOut = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} keyboardHID;

keyboardHID keyboardhid = { 0, 0, 0, 0, 0, 0, 0, 0 };

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		RxInput = 1;
		if (RxHeader.DLC == 1) {
			NumTx = RxData[0] - 0x01;
		}
	}

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */

	TxHeader.StdId = 0x0264;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	TxHeader.DLC = 3; // Power UP TX
	TxData[0] = 0xE0;
	TxData[1] = 0x01;
	TxData[2] = 0x00;
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
		;
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//==============================================================================================================================================
		/* TV tuner emulator BEGIN */
		if (RxInput == 1) {
			//==============================================================================================================================================
			/* TV tuner init BEGIN */

			if (RxHeader.DLC == 3 && RxData[0] == 0xE0 && RxData[1] == 0x01 && RxData[2] == 0x00) { // Handshake

				NumTx = 0;
				TxHeader.DLC = 3;
				TxData[0] = 0xE0;
				TxData[1] = 0x01;
				TxData[2] = 0x00;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}

			else if (RxHeader.DLC == 2 && RxData[0] == 0xE1 && RxData[1] == 0x01) { //Purpose unknown (?)

				TxHeader.DLC = 7;
				TxData[0] = 0x10;
				TxData[1] = 0x15;
				TxData[2] = 0x01;
				TxData[3] = 0x01; // or 0x00 unknown difference
				TxData[4] = 0x02;
				TxData[5] = 0x00;
				TxData[6] = 0x00;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

				TxHeader.DLC = 4; 			 		//Video encoding system
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 0x27;
				TxData[2] = 0x02; 					// 0x01 = PAL 50 Herz, 0x02 = NTSC 60 Herz
				TxData[3] = 0x00;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}

			else if (RxHeader.DLC == 2 && RxData[0] == 0xA3 && RxData[1] == 0x00) { //Ping - Pong

				TxHeader.DLC = 2;
				TxData[0] = 0xE1;
				TxData[1] = 0x01;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}

			else if (RxHeader.DLC > 1) {  //ACK for all requests
				TxHeader.DLC = 1;
				TxData[0] = ((RxData[0] + 0x01) & 0x0F) + 0xB0;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}
			}

			if (RxHeader.DLC == 3 && RxData[1] == 0x00 && RxData[2] == 0x01) { //Purpose unknown (?)

				TxHeader.DLC = 3;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 0x01;
				TxData[2] = 0x01;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}

			/* TV tuner init END */
			//==============================================================================================================================================
			/* SW HW data BEGIN */

			if (RxHeader.DLC == 2 && RxData[1] == 0x08) { //Request tuner SW HW data  (open the service menu - version - TV)

				TxHeader.DLC = 8;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = 0x09;						//Purpose unknown (?)
				TxData[2] = 0x01;						//Purpose unknown (?)
				TxData[3] = 0x01;						//Purpose unknown (?)
				TxData[4] = 0x20;						//Purpose unknown (?)
				TxData[5] = 0x44;						//SW TV tuner Version
				TxData[6] = 0x33; 						//SW TV tuner Version
				TxData[7] = '1';						//ASCII TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}
				HAL_Delay(5);
				TxHeader.DLC = 8;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = '1';						//ASCII TV tuner Version
				TxData[2] = '1';						//ASCII TV tuner Version
				TxData[3] = '1';						//ASCII TV tuner Version
				TxData[4] = '1';						//ASCII TV tuner Version
				TxData[5] = '1'; 						//ASCII TV tuner Version
				TxData[6] = '1';						//ASCII TV tuner Version
				TxData[7] = '1';						//ASCII TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}


				HAL_Delay(5);
				TxHeader.DLC = 8;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = 0x00;						//END ASCII TV tuner Version
				TxData[2] = '2';						//ASCII TV tuner Version
				TxData[3] = '2';						//ASCII TV tuner Version
				TxData[4] = '2';						//ASCII TV tuner Version
				TxData[5] = '2';						//ASCII TV tuner Version
				TxData[6] = '2'; 						//ASCII TV tuner Version
				TxData[7] = '2';						//ASCII TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}


				HAL_Delay(5);
				TxHeader.DLC = 8;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = '2';						//ASCII TV tuner Version
				TxData[2] = '2';						//ASCII TV tuner Version
				TxData[3] = '2';						//ASCII TV tuner Version
				TxData[4] = '2';						//ASCII TV tuner Version
				TxData[5] = 0x00;						//END ASCII TV tuner Version
				TxData[6] = 0x22; 						//HW TV tuner Version
				TxData[7] = 0x11;//HW TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}
			/* SW HW data END */
			//==============================================================================================================================================
			/* COMAND to TV settings BEGIN */

			if (RxHeader.DLC == 8 && RxData[1] == 0x24) { //COMAND to TV settings

			//Variable settings = RxData[2]  //0x0* == Search on Freq + miles			0x*0 == Deutsche  	0xFF == no data
			//								 //0x4* == Search on Freq + kilometers		0x*1 == English
			//						    	 //0x8* == Search on Memory + miles 		0x*2 == French
			//								 //0xC* == Search on Memory + kilometers	0x*3 == Italian
			//																			0x*4 ==	Spanish
			//Light interior = RxData[3] //0x00 == Light off, 0x0C-0x64 == Light on and dimming 			0xFF == no data
			//Light sensor	 = RxData[4] //0x00 == day, 0x40 == night										0xFF == no data
			//Key lock state = RxData[5] //0x00 ==  key is out lock,  0x01 == key insert, 0x03 == ACC, 0x0F == ignition, 0x17 == RUN,  0x07 == release RUN 0xFF == no data
			//UNKNOW SETTING = RxData[6] //always == 0x00 (speed?)											0xFF == no data
			//UNKNOW SETTING = RxData[7] //always == 0x00 (speed?)											0xFF == no data

			}

			/* COMAND to TV settings  END */
			//==============================================================================================================================================
			/* TV to COMAND to Cluster BEGIN */

			if (RxHeader.DLC == 5 && RxData[1] == 0x30) { //example trigger

				TxHeader.DLC = 8;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = 0x47; 						//Sending ASCII TV tuner - COMAND - instrument cluster
				TxData[2] = 0xFF;						//Number TV
				TxData[3] = 0xFF;						//Purpose unknown (?)
				TxData[4] = TxDataOut;					//1 ASCII TV tuner - COMAND - instrument cluster //Send 0x00 for END roll
				TxData[5] = TxDataOut + 0x01;						//2 ASCII TV tuner - COMAND - instrument cluster
				TxData[6] = TxDataOut + 0x02; 						//3 ASCII TV tuner - COMAND - instrument cluster
				TxData[7] = TxDataOut + 0x03; 						//4 ASCII TV tuner - COMAND - instrument cluster
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

				TxHeader.DLC = 6;
				NumTx = NumTx + 0x01;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = TxDataOut + 0x04; 						//5 ASCII TV tuner - COMAND - instrument cluster
				TxData[2] = TxDataOut + 0x05; 						//6 ASCII TV tuner - COMAND - instrument cluster
				TxData[3] = TxDataOut + 0x06; 						//7 ASCII TV tuner - COMAND - instrument cluster
				TxData[4] = TxDataOut + 0x07; 						//8 ASCII TV tuner - COMAND - instrument cluster
				TxData[5] = 0x00;                       			//END ASCII TV tuner - COMAND - instrument cluster
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}
				TxDataOut++;

			}

			/* TV to COMAND to Cluster END */
			//==============================================================================================================================================
			/* COMAND to HID BEGIN */

			if (RxHeader.DLC == 5 && RxData[1] == 0x30 && RxData[2] == 0x01) { // Key press

				NumPress++;

				if (RxData[3] == 0x40) { // 0 Key
					keyboardhid.KEYCODE1 = KEY_KP0;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

				if (RxData[3] == 0x41) { // 1 Key
					keyboardhid.KEYCODE1 = KEY_KP1;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}
				if (RxData[3] == 0x42) { // 2 Key

					if (KeyMode == 0) {
						keyboardhid.KEYCODE1 = KEY_UP;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}

					else {

						keyboardhid.KEYCODE1 = KEY_KP2;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
				}
				if (RxData[3] == 0x43) { // 3 Key
					keyboardhid.KEYCODE1 = KEY_KP3;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}
				if (RxData[3] == 0x44) { // 4 Key

					if (KeyMode == 0) {
						keyboardhid.KEYCODE1 = KEY_LEFT;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}

					else {

						keyboardhid.KEYCODE1 = KEY_KP4;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
				}
				if (RxData[3] == 0x45) { // 5 Key

					if (NumPress == 40) {
						KeyMode = !KeyMode;
					}

					if (KeyMode == 0) {
						keyboardhid.KEYCODE1 = KEY_ENTER;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}

					else {
						keyboardhid.KEYCODE1 = KEY_KP5;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
				}
				if (RxData[3] == 0x46) { // 6 Key

					if (KeyMode == 0) {
						keyboardhid.KEYCODE1 = KEY_RIGHT;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}

					else {
						keyboardhid.KEYCODE1 = KEY_KP6;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
				}
				if (RxData[3] == 0x47) { // 7 Key
					keyboardhid.KEYCODE1 = KEY_KP7;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}
				if (RxData[3] == 0x48) { // 8 Key

					if (KeyMode == 0) {
						keyboardhid.KEYCODE1 = KEY_DOWN;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}

					else {

						keyboardhid.KEYCODE1 = KEY_KP8;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
				}
				if (RxData[3] == 0x49) { // 9 Key
					keyboardhid.KEYCODE1 = KEY_KP9;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}
				if (RxData[3] == 0x50) { // * Key

					if (NumPress == 1) {
						keyboardhid.KEYCODE1 = KEY_KPASTERISK;
						USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					}
					if (NumPress == 40) {
						TxHeader.DLC = 4;   //Open sound settings
						NumTx = NumTx + 0x01;
						TxData[0] = (NumTx & 0x0F) + 0x10;
						TxData[1] = 0x2D;
						TxData[2] = 0x01;
						TxData[3] = 0x01;
						while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
							;
						if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
							HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
						}
					}

				}
				if (RxData[3] == 0x51) { // # Key
					keyboardhid.KEYCODE1 = KEY_KPDOT;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}
				if (RxData[3] == 0x62) { // Back Key
					keyboardhid.KEYCODE1 = KEY_B;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

				if (RxData[3] == 0x63) { // Forward Key
					keyboardhid.KEYCODE1 = KEY_N;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

				if (RxData[3] == 0x68) { // Forward Steering Wheel Key
					keyboardhid.KEYCODE1 = KEY_N;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

				if (RxData[3] == 0x69) { // Back Steering Wheel Key
					keyboardhid.KEYCODE1 = KEY_B;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

				if (RxData[3] == 0x71) { // Press Encoder
					keyboardhid.KEYCODE1 = KEY_ENTER;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

				}

				if (RxData[3] == 0x80) { // RET Key
					keyboardhid.KEYCODE1 = KEY_BACKSPACE;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

			} else if (RxHeader.DLC == 5 && RxData[1] == 0x30 && RxData[2] == 0x02) { // Key release
				NumPress = 0;
				keyboardhid.KEYCODE1 = KEY_NONE;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
			}

			if (RxHeader.DLC == 5 && RxData[1] == 0x34) { // Encoder rotary

				if (RxData[2] == 0x01) { // + rotary

					keyboardhid.KEYCODE1 = KEY_RIGHT;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

				}
				if (RxData[2] == 0x02) { // - rotary
					keyboardhid.KEYCODE1 = KEY_LEFT;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

			}

			if (RxHeader.DLC == 3 && RxData[1] == 0x2A) { // State TV mode

				if (RxData[2] == 0x01) { // Open TV
					keyboardhid.KEYCODE1 = KEY_F1;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

				}
				if (RxData[2] == 0x02) { // Close TV
					keyboardhid.KEYCODE1 = KEY_F2;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

			}

			if (RxHeader.DLC == 3 && RxData[1] == 0x2C) { // State sound background TV mode

				if (RxData[2] == 0x01) { // TV sound enable
					keyboardhid.KEYCODE1 = KEY_F1;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

				}

			}

			if (RxHeader.DLC == 3 && RxData[1] == 0x3A) { // State Mute button

				if (RxData[2] == 0x00) { // UNMute
					keyboardhid.KEYCODE1 = KEY_F1;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

				}
				if (RxData[2] == 0x01) { // Mute
					keyboardhid.KEYCODE1 = KEY_F2;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
					HAL_Delay(50);
					keyboardhid.KEYCODE1 = 0x00;
					USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				}

			}

			if (RxHeader.DLC == 3 && RxData[1] == 0x36 && RxData[2] == 0x02) { // COMAND Power off

				keyboardhid.KEYCODE1 = KEY_F5;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
				HAL_Delay(50);
				keyboardhid.KEYCODE1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));

			}

			/* COMAND to HID END */
			//==============================================================================================================================================
			RxInput = 0;
		}

		/* TV tuner emulator END */
		//==============================================================================================================================================
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 36;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0464 << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0464 << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
//sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {

	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
