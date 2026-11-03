/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "usb.h"
#include "gpio.h"
#include "usb_device.h"
#include "usb_hid_keys.h"
#include "usbd_cdc_acm.h"
#include "usbd_cdc_acm_if.h"
#include "usbd_hid_keyboard.h"
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

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = { 0, };
uint8_t RxData[8] = { 0, };
uint32_t TxMailbox = 0;
uint32_t hid_release_tick = 0;    // The time when you need to release the key
uint8_t hid_need_release = 0;    // A flag indicating that the key is currently "push"

_Bool RxInput = 0;
_Bool KeyMode = 0;
unsigned char NumTx = 0;
unsigned char NumPress = 0;

extern uint8_t RX_Buffer[NUMBER_OF_CDC][APP_RX_DATA_SIZE];
volatile uint8_t recvDone = 0;
volatile uint16_t recvLen = 0;
uint8_t TxCDCData[9] = { 0 };
static const char hex_lut[] = "0123456789ABCDEF";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDevice;

/* DFU BOOT support for STM32F072 */

#define STM32_DFU_ADDRESS 0x1FFFC800UL

/* Check incoming CDC command */
static uint8_t DFU_IsBootCommand(const uint8_t *data, uint32_t length) {
	const uint8_t command[] = "DFU_BOOT";
	uint32_t i;

	if ((data == NULL) || (length < sizeof(command) - 1U) || (length > sizeof(command) + 1U)) {
		return 0U;
	}

	if (memcmp(data, command, sizeof(command) - 1U) != 0) {
		return 0U;
	}

	for (i = sizeof(command) - 1U; i < length; ++i) {
		if ((data[i] != '\r') && (data[i] != '\n')) {
			return 0U;
		}
	}

	return 1U;
}

/* Jump to STM32 system memory bootloader */
__attribute__((naked, noreturn))
static void DFU_BranchToROM(void) {
	__asm volatile (
			"ldr r0, =0x1FFFC800\n"
			"ldr r1, [r0]\n"
			"ldr r0, [r0, #4]\n"
			"msr msp, r1\n"
			"cpsie i\n"
			"bx r0\n"
	);
}

/* Jumping to the ROM from the running application leaves USB dead until the board
 * is re-plugged. Set a flag that survives a reset, reset the chip, and jump from the
 * top of main() while every peripheral is still in its reset state. */
#define DFU_MAGIC 0xDF00B007UL
static volatile uint32_t dfuMagic __attribute__((section(".noinit")));

static void DFU_EnterBootloader(void) __attribute__((noreturn));

static void DFU_EnterBootloader(void) {
	/* Disconnect USB from host */
	(void) HAL_PCD_DevDisconnect(&hpcd_USB_FS);
	HAL_Delay(100U);

	dfuMagic = DFU_MAGIC;
	NVIC_SystemReset();

	for (;;) {
	}
}

/* END DFU BOOT support */

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

// Fast inline function for assembling and sending a package to CDC
__STATIC_INLINE void CAN_To_CDC_FormatAndTransmit(uint16_t stdId, uint8_t dlc, const uint8_t *data) {
	char tx_buf[45]; // A local buffer on the stack prevents race conditions between interrupts

	// Filling in the constant prefix "RX ID " and " HEX "
	tx_buf[0] = 'R';
	tx_buf[1] = 'X';
	tx_buf[2] = ' ';
	tx_buf[3] = 'I';
	tx_buf[4] = 'D';
	tx_buf[5] = ' ';
	tx_buf[10] = ' ';
	tx_buf[11] = 'H';
	tx_buf[12] = 'E';
	tx_buf[13] = 'X';
	tx_buf[14] = ' ';

	// Convert StdId to 4 HEX characters
	tx_buf[6] = hex_lut[(stdId >> 12) & 0x0F];
	tx_buf[7] = hex_lut[(stdId >> 8) & 0x0F];
	tx_buf[8] = hex_lut[(stdId >> 4) & 0x0F];
	tx_buf[9] = hex_lut[stdId & 0x0F];

	// Data Byte conversion
	int idx = 15;
	for (uint32_t i = 0; i < dlc; i++) {
		tx_buf[idx++] = hex_lut[(data[i] >> 4) & 0x0F];
		tx_buf[idx++] = hex_lut[data[i] & 0x0F];
		tx_buf[idx++] = ' ';
	}

	// Windows-style Line completion (CRLF)
	tx_buf[idx++] = '\r';
	tx_buf[idx++] = '\n';

	// Direct transfer to USB CDC
	CDC_Transmit(0, (uint8_t*) tx_buf, idx);
}

void SendKey(uint8_t key) {
	keyboardhid.KEYCODE1 = key;
	USBD_HID_Keyboard_SendReport(&hUsbDevice, (uint8_t*) &keyboardhid, sizeof(keyboardhid));
}

void SendKeyClick(uint8_t key) {
	SendKey(key);
	hid_release_tick = HAL_GetTick() + 10; // Memorize the release time
	hid_need_release = 1;
	SendKey(KEY_NONE);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		RxInput = 1;

		if (RxHeader.DLC == 1) {
			NumTx = RxData[0];
		}

		// Calling formatting and end-to-end sending CDC
		CAN_To_CDC_FormatAndTransmit(RxHeader.StdId, RxHeader.DLC, RxData);
	}

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef fifo1RxHeader;
	uint8_t fifo1RxData[8];

	// Taking the package from isolated FIFO 1
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &fifo1RxHeader, fifo1RxData) == HAL_OK) {
		// Calling formatting and end-to-end sending CDC
		CAN_To_CDC_FormatAndTransmit(fifo1RxHeader.StdId, fifo1RxHeader.DLC, fifo1RxData);
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	if (dfuMagic == DFU_MAGIC) { /* DFU_BOOT asked for the ROM bootloader before the reset */
		dfuMagic = 0;
		__HAL_RCC_SYSCFG_CLK_ENABLE();
		__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
		DFU_BranchToROM();
	}

	/* dfu-util's "leave" jumps here from the ROM with system memory still mapped at 0,
	 * so the interrupt vectors (no VTOR on the M0) would be the ROM's and USB never
	 * comes up until a re-plug. Map our flash back first. */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_SYSCFG_REMAPMEMORY_FLASH();
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
	MX_USB_PCD_Init();
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
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

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

		// Non-blocking reset of the HID key
		if (hid_need_release && (HAL_GetTick() >= hid_release_tick)) {
			hid_need_release = 0;
			SendKey(KEY_NONE); //
		}

		/* Read the flag once per pass: a line arriving between the two checks below
		 * skipped the DFU_BOOT check and went out to the cluster as text. */
		const uint8_t line = recvDone;

		/* DFU_BOOT command via USB CDC-ACM */
		if ((line == 1U) && DFU_IsBootCommand(RX_Buffer[0], recvLen)) {
			static uint8_t response[] = "DFU_BOOT_OK\r\n";

			/* Send acknowledgement */
			(void) CDC_Transmit(0, response, sizeof(response) - 1U);

			recvDone = 0U;

			/* Allow USB transmission to complete */
			HAL_Delay(100U);

			/* Jump to system DFU bootloader */
			DFU_EnterBootloader();
		}
		/* END DFU_BOOT command */

		//==============================================================================================================================================
		/* TV tuner emulator BEGIN */

		if (line == 1) {
			// Data capture (Snapshot) under the protection of disabling interrupts
			__disable_irq();
			uint16_t localLen = recvLen;
			uint8_t localBuffer[9] = { 0 };
			memcpy(localBuffer, TxCDCData, localLen);
			recvDone = 0; // Now USB can accept a new line in the background while we are busy with CAN
			__enable_irq();

			// Calculation of the total length of the payload (text + terminator 0x00)
			uint8_t dataTotalLen = localLen + 1;

			// --- FORMATION AND SENDING OF THE FIRST PACKET ---
			// In the first packet, a maximum of 4 bytes are available for text (after 4 service bytes)
			uint8_t pack1TextLen = (dataTotalLen > 4) ? 4 : dataTotalLen;

			uint8_t TxData[8] = { 0 }; // A clean array for CAN
			TxHeader.DLC = 4 + pack1TextLen; // Dynamic DLC
			TxData[0] = (NumTx & 0x0F) + 0x00; // Serial number of package #1
			TxData[1] = 0x47;                  // ASCII text to IC prefix
			TxData[2] = 0xFF;                  // TV Number
			TxData[3] = 0xFF;                  // Purpose unknown (?)
			for (uint8_t i = 0; i < pack1TextLen; i++) {
				if (i == localLen) {
					TxData[4 + i] = 0x00; // We set the end sign if the text has ended
				} else {
					TxData[4 + i] = localBuffer[i];
				}
			}

			// Freeze protection: sending with a hard timeout of 10 ms
			uint32_t timeout = HAL_GetTick() + 10;
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
				if (HAL_GetTick() > timeout)
					break;
			}
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
				NumTx++;
			}

			// --- FORMING AND SENDING A SECOND PACKET (only if the text is > 3 characters) ---
			if (dataTotalLen > 4) {
				uint8_t pack2TextLen = dataTotalLen - 4; // Remaining bytes (maximum 5)

				memset(TxData, 0, 8); // Clearing the array for the second frame
				TxHeader.DLC = 1 + pack2TextLen; // 1 service byte + remainder of payload
				TxData[0] = (NumTx & 0x0F) + 0x10; // Serial number of packet #2
				for (uint8_t i = 0; i < pack2TextLen; i++) {
					uint8_t textIndex = 4 + i;
					if (textIndex == localLen) {
						TxData[1 + i] = 0x00; // End of line
					} else {
						TxData[1 + i] = localBuffer[textIndex];
					}
				}

				timeout = HAL_GetTick() + 10;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
					if (HAL_GetTick() > timeout)
						break;
				}
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					NumTx++;
				}
			}

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // End of transmission
		}

		if (RxInput == 1) {
			//==============================================================================================================================================
			/* TV tuner init BEGIN */

			if (NumTx == 0) {

				TxHeader.DLC = 6;                    //Purpose unknown (?)
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 0x15;
				TxData[2] = 0x01;
				TxData[3] = 0x01; // or 0x00 unknown difference
				TxData[4] = 0x02;
				TxData[5] = 0x02; // or 0x00 unknown difference
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}
			}

			if (RxData[0] == 0xE0 && RxData[1] == 0x01 && RxData[2] == 0x00) { // Handshake

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

			if (RxData[0] == 0xA3 && RxData[1] == 0x00) { //Ping - Pong

				TxHeader.DLC = 2;
				TxData[0] = 0xE1;
				TxData[1] = 0x01;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				}

			}

			if (RxHeader.DLC > 1 && RxData[0] < 50) {  //ACK for all requests
				TxHeader.DLC = 1;
				TxData[0] = ((RxData[0] + 0x01) & 0x0F) + 0xB0;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {

				}
			}

			if (RxData[1] == 0x00 && RxData[2] == 0x01) { //Purpose unknown (?)

				TxHeader.DLC = 3;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 0x01;
				TxData[2] = 0x01;
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}

			}

			if (RxData[1] == 0x26) { //Video status (?)

				TxHeader.DLC = 4;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 0x27;
				TxData[2] = 0x02; 					// 0x01 = PAL 50 Hertz, 0x02 = NTSC 60 Hertz
				TxData[3] = 0x01; 			// 0x00 = video off, 0x01 = video on (?)
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}
			}

			/* TV tuner init END */
			//==============================================================================================================================================
			/* SW HW data BEGIN */

			if (RxData[1] == 0x08) { //Request tuner SW HW data  (open the service menu - version - TV)

				TxHeader.DLC = 8;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = 0x09;						//Purpose unknown (?)
				TxData[2] = 0x01;						//Purpose unknown (?)
				TxData[3] = 0x01;						//Purpose unknown (?)
				TxData[4] = 0x20;						//Purpose unknown (?)
				TxData[5] = 0x00;						//SW TV tuner Version
				TxData[6] = 0x03; 						//SW TV tuner Version
				TxData[7] = '1';						//ASCII TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}
				HAL_Delay(5);
				TxHeader.DLC = 8;
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
					NumTx++;
				}

				HAL_Delay(5);
				TxHeader.DLC = 8;
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
					NumTx++;
				}

				HAL_Delay(5);
				TxHeader.DLC = 8;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = '2';						//ASCII TV tuner Version
				TxData[2] = '2';						//ASCII TV tuner Version
				TxData[3] = '2';						//ASCII TV tuner Version
				TxData[4] = '2';						//ASCII TV tuner Version
				TxData[5] = 0x00;						//END ASCII TV tuner Version
				TxData[6] = 0x22; 						//HW TV tuner Version
				TxData[7] = 0x11; 						//HW TV tuner Version
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}

			}
			/* SW HW data END */
			//==============================================================================================================================================
			/* COMAND to TV settings BEGIN */

			if (RxData[1] == 0x24) { //COMAND to TV settings

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
			/* COMAND to TV warnings BEGIN */

			if (RxData[1] == 0x42) { //COMAND to TV warnings

				//Warnings = RxData[2]
				//0x00 == Clear Warning
				//0x01 == Low Voltage, disable after 3 minutes
				//0x02 == Low Voltage, disable after 10 seconds
				//0x03 == Low Voltage, disable right now
				//0x04 == Turn the key, disable after 3 minutes
				//0x05 == Turn the key, disable after 10 seconds
				//0x06 == Turn the key, disable right now
				//0x07 == NAVI, No navigation disk
				//0x08 == NAVI, Voice output is muted
				//0x09 == Commn, Process Active

			}

			/* COMAND to TV warnings  END */
			//==============================================================================================================================================
			/* TV to COMAND to Cluster BEGIN */

			if (RxData[1] == 0x46) {

				TxHeader.DLC = 8;
				TxData[0] = (NumTx & 0x0F) + 0x00;
				TxData[1] = 0x47; 						//Sending ASCII TV tuner - COMAND - instrument cluster
				TxData[2] = 0xFF;						//Number TV
				TxData[3] = 0xFF;						//Purpose unknown (?)
				TxData[4] = 'L';						//1 ASCII TV tuner - COMAND - instrument cluster //Send 0x00 for END roll
				TxData[5] = 'o';						//2 ASCII TV tuner - COMAND - instrument cluster
				TxData[6] = 'a'; 						//3 ASCII TV tuner - COMAND - instrument cluster
				TxData[7] = 'd'; 						//4 ASCII TV tuner - COMAND - instrument cluster
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}

				TxHeader.DLC = 6;
				TxData[0] = (NumTx & 0x0F) + 0x10;
				TxData[1] = 'i'; 						//5 ASCII TV tuner - COMAND - instrument cluster
				TxData[2] = 'n'; 						//6 ASCII TV tuner - COMAND - instrument cluster
				TxData[3] = 'g'; 						//7 ASCII TV tuner - COMAND - instrument cluster
				TxData[4] = 0x00; 						//8 ASCII TV tuner - COMAND - instrument cluster
				TxData[5] = 0x00;             			//END ASCII TV tuner - COMAND - instrument cluster
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
					;
				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
					NumTx++;
				}

			}

			/* TV to COMAND to Cluster END */
			//==============================================================================================================================================
			/* COMAND to HID BEGIN */

			if (RxData[1] == 0x30) { // Key process

				if (RxData[2] == 0x01) { // Key press

					NumPress++; // Counter for long press

					switch (RxData[3]) {
					case 0x40: // 0 Key
						SendKey(KEY_KP0);
						break;
					case 0x41: // 1 Key
						SendKey(KEY_KP1);
						break;
					case 0x42: // 2 Key
						SendKey((KeyMode == 0) ? KEY_UP : KEY_KP2);
						break;
					case 0x43: // 3 Key
						SendKey(KEY_KP3);
						break;
					case 0x44: // 4 Key
						SendKey((KeyMode == 0) ? KEY_LEFT : KEY_KP4);
						break;

					case 0x45: // 5 Key
						if (NumPress == 40) {
							KeyMode = !KeyMode;
						}
						SendKey((KeyMode == 0) ? KEY_ENTER : KEY_KP5);
						break;

					case 0x46: // 6 Key
						SendKey((KeyMode == 0) ? KEY_RIGHT : KEY_KP6);
						break;
					case 0x47: // 7 Key
						SendKey(KEY_KP7);
						break;
					case 0x48: // 8 Key
						SendKey((KeyMode == 0) ? KEY_DOWN : KEY_KP8);
						break;
					case 0x49: // 9 Key
						SendKey(KEY_KP9);
						break;

					case 0x50: // * Key
						if (NumPress == 1) {
							SendKey(KEY_KPASTERISK);
						} else if (NumPress == 40) {
							TxHeader.DLC = 4;   // Open sound settings
							TxData[0] = (NumTx & 0x0F) + 0x10;
							TxData[1] = 0x2D;
							TxData[2] = 0x01;
							TxData[3] = 0x01;
							while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
								;
							if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
								HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
								NumTx++;
							}
						}
						break;

					case 0x51: // # Key
						SendKey(KEY_KPDOT);
						break;
					case 0x62: // Back Key
						SendKey(KEY_B);
						break;
					case 0x69: // Back Steering Wheel Key
						SendKey(KEY_B);
						break;
					case 0x63: // Forward Key
						SendKey(KEY_N);
						break;
					case 0x68: // Forward Steering Wheel Key
						SendKey(KEY_N);
						break;
					case 0x71: // Press Encoder
						SendKey(KEY_ENTER);
						break;
					case 0x80: // RET Key
						SendKey(KEY_BACKSPACE);
						break;
					default:
						break;
					}

				} else if (RxData[2] == 0x02) { // Key release
					NumPress = 0;
					SendKey(KEY_NONE);
				}
			}

			if (RxData[1] == 0x34) { // Encoder rotary
				if (RxData[2] == 0x01) // + rotary
					SendKeyClick(KEY_KPPLUS);
				else if (RxData[2] == 0x02) // - rotary
					SendKeyClick(KEY_KPMINUS);
			}

			if (RxData[1] == 0x2A) { // State TV mode
				if (RxData[2] == 0x01) // Open TV
					SendKeyClick(KEY_F1);
				else if (RxData[2] == 0x02) // Close TV
					SendKeyClick(KEY_F2);
			}

			if (RxData[1] == 0x2C) { // State sound background TV mode
				if (RxData[2] == 0x01)  // TV sound enable
					SendKeyClick(KEY_F1);
			}

			if (RxData[1] == 0x3A) { // State Mute button
				if (RxData[2] == 0x00) // UNMute
					SendKeyClick(KEY_F1);
				else if (RxData[2] == 0x01) // Mute
					SendKeyClick(KEY_F2);

			}

			if (RxData[1] == 0x36 && RxData[2] == 0x02) { // COMAND Power off
				if (RxData[2] == 0x02)
					SendKeyClick(KEY_F5);
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
#ifdef USE_FULL_ASSERT
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
