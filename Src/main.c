/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include <stdio.h>
#include "ir_remote.h"
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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
decode_results results;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t REPORT_ID;
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE_1;
	uint8_t KEYCODE_2;
	uint8_t KEYCODE_3;
	uint8_t KEYCODE_4;
	uint8_t KEYCODE_5;
	uint8_t KEYCODE_6;
} keyboardHID;

keyboardHID keyboard = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };

#pragma pack(push, 1)
typedef struct {
	uint8_t REPORT_ID;
	uint16_t MEDIAKEY;
} mediakeyHID;
#pragma pack(pop)

mediakeyHID mediakey = { 2, 0 };

uint8_t keyF = 0;

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
	MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	ir_enableIRIn();

	char trans_str[64] = { 0, };
	snprintf(trans_str, 64, "IR-receiver\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) trans_str, strlen(trans_str), 100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE A*/
	while (1) {

		if (keyF)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

		if (ir_decode(&results)) {
			snprintf(trans_str, 64, "Code: HEX %p DEC %lu\r\n",
					(void*) results.value, results.value);
			HAL_UART_Transmit(&huart1, (uint8_t*) trans_str, strlen(trans_str),
					100);
			HAL_Delay(50);
			ir_resume();

			if (results.value == 0xd538681b) {
				mediakey.MEDIAKEY = 0b0000000000000001; // VOL-
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0x8c22657b) {
				mediakey.MEDIAKEY = 0b0000000000000010; // VOL+
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0x51e43d1b) {
				mediakey.MEDIAKEY = 0b0000000000000100; // MUTE
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0x9716be3f) {
				mediakey.MEDIAKEY = 0b0000000000001000; // PREV
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0xe5cfbd7f) {
				mediakey.MEDIAKEY = 0b0000000000010000; // NEXT
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0xdc0197db) {
				mediakey.MEDIAKEY = 0b0000000000100000; // PLAY/PAUSE
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0x86b0e697) {
				mediakey.MEDIAKEY = 0b0000000010000000; // STOP
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0x488f3cbb) {
				mediakey.MEDIAKEY = 0b0000000001000000; // PAUSE
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0xa23c94bf) { // PLAY
				mediakey.MEDIAKEY = 0b0000100000000000;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0xab91951f) { // REWIND
				mediakey.MEDIAKEY = 0b0000000100000000;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}

			else if (results.value == 0xff9186b7) { // FORWARD
				mediakey.MEDIAKEY = 0b0000001000000000;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
				HAL_Delay(50);
				mediakey.MEDIAKEY = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			}
			/*
			 else if (results.value == 0x00ff00) { // RESERVED MEDIA BUTTON
			 mediakey.MEDIAKEY = 0b0000010000000000;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 HAL_Delay(50);
			 mediakey.MEDIAKEY = 0x00;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 }

			 else if (results.value == 0x00ff00) { // RESERVED MEDIA BUTTON
			 mediakey.MEDIAKEY = 0b0001000000000000;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 HAL_Delay(50);
			 mediakey.MEDIAKEY = 0x00;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 }


			 else if (results.value == 0x00ff00) { // RESERVED MEDIA BUTTON
			 mediakey.MEDIAKEY = 0b0010000000000000;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 HAL_Delay(50);
			 mediakey.MEDIAKEY = 0x00;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 }


			 else if (results.value == 0x00ff00) { // RESERVED MEDIA BUTTON
			 mediakey.MEDIAKEY = 0b0100000000000000;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 HAL_Delay(50);
			 mediakey.MEDIAKEY = 0x00;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 }

			 else if (results.value == 0x00ff00) { // RESERVED MEDIA BUTTON
			 mediakey.MEDIAKEY = 0b1000000000000000;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 HAL_Delay(50);
			 mediakey.MEDIAKEY = 0x00;
			 USBD_HID_SendReport(&hUsbDeviceFS, &mediakey, sizeof(mediakey));
			 }
			 */
			else if (results.value == 0x35a9425f) {
				keyboard.KEYCODE_1 = 0x52; // Command: "Up"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x9ef4941f) {
				keyboard.KEYCODE_1 = 0x50; // Command: "Left"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x8e5d3ebb) {
				keyboard.KEYCODE_1 = 0x4F; // Command: "Right"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xa3c8eddb) {
				keyboard.KEYCODE_1 = 0x51; // Command: "Down"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xf377c5b7) {
				keyboard.KEYCODE_1 = 0x28; // Command: "Enter"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xc101e57b) {
				keyboard.KEYCODE_1 = 0x29; // Command: "Esc"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x5be75e7f) {
				keyboard.KEYCODE_1 = 0x09; // [OFF] Command: "F"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x97483bfb) {
				keyboard.KEYCODE_1 = 0x2C; // Command: "Space"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x3d9ae3f7) {
				keyboard.KEYCODE_1 = 0x2B; // Command: "Tab"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xf63c8657) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x1E; // Command: "1"
				else
					keyboard.KEYCODE_1 = 0x3A; // Command: "F1"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xe721c0db) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x1F; // Command: "2"
				else
					keyboard.KEYCODE_1 = 0x3B; // Command: "F2"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xaa99e7b) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x20; // Command: "3"
				else
					keyboard.KEYCODE_1 = 0x3C; // Command: "F3"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x6456f67f) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x21; // Command: "4"
				else
					keyboard.KEYCODE_1 = 0x3D; // Command: "F4"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x378e71f) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x22; // Command: "5"
				else
					keyboard.KEYCODE_1 = 0x3E; // Command: "F5"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x53bfa11b) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x23; // Command: "6"
				else
					keyboard.KEYCODE_1 = 0x3F; // Command: "F6"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x2355bd47) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x24; // Command: "7"
				else
					keyboard.KEYCODE_1 = 0x40; // Command: "F7"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xb3ca0f43) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x25; // Command: "8"
				else
					keyboard.KEYCODE_1 = 0x41; // Command: "F8"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xd10306e3) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x26; // Command: "9"
				else
					keyboard.KEYCODE_1 = 0x42; // Command: "F9"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0xb40c5d5f) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x27; // Command: "0"
				else
					keyboard.KEYCODE_1 = 0x43; // Command: "F10"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x2149c0df) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x2D; // Command: "-"
				else
					keyboard.KEYCODE_1 = 0x44; // Command: "F11"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x4480af5b) {
				if (!keyF)
					keyboard.KEYCODE_1 = 0x2E; // Command: "="
				else
					keyboard.KEYCODE_1 = 0x45; // Command: "F12"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x13549bdf) {
				keyboard.KEYCODE_1 = 0x44; // Command: "F11"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x45473c1b) {
				keyboard.KEYCODE_1 = 0x2A; // Command: "Backspace"
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
				HAL_Delay(50);
				keyboard.MODIFIER = 0x00;
				keyboard.KEYCODE_1 = 0x00;
				USBD_HID_SendReport(&hUsbDeviceFS, &keyboard, sizeof(keyboard));
			}

			else if (results.value == 0x44c407db) {
				if (!keyF)
					keyF = 1;
				else
					keyF = 0;
			}

		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 49;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : IR_Receiver_Pin */
	GPIO_InitStruct.Pin = IR_Receiver_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_Receiver_GPIO_Port, &GPIO_InitStruct);

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

