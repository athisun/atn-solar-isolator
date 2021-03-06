
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #include "can.h"
#include "iwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

// #define CAN_ID_PRECHARGE 0x6F7

#define PRECHARGE_TIME 1000

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

volatile uint8_t do_precharge = 1;
volatile uint8_t turn_off = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/

// fifo0 interrupt handler
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   CAN_RxHeaderTypeDef header;
//   uint8_t data[8] = {};
//   HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data);

//   if (header.StdId != CAN_ID_PRECHARGE)
//   {
//     // something went wrong the filter?
//     // ignore packet
//     return;
//   }

//   // https://drive.google.com/file/d/13xNdZU3iHUccOYC51neVvtTOsEyVUbjg/view

//   if (data[1] == 4)
//   {
//     do_precharge = 1;
//   }
//   else
//   {
//     turn_off = 1;
//   }
// }

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_CAN1_Init();
  MX_IWDG_Init();

  // reset the precharge and switch pins
  HAL_GPIO_WritePin(PRECHARGE_POS_GPIO_Port, PRECHARGE_POS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PRECHARGE_NEG_GPIO_Port, PRECHARGE_NEG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SW_POS_GPIO_Port, SW_POS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SW_NEG_GPIO_Port, SW_NEG_Pin, GPIO_PIN_RESET);

  /* Infinite loop */
  while (1)
  {
    // busy loop
    HAL_Delay(1000);

    // turn the led on
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    // pull the precharge pins the right way
    HAL_GPIO_WritePin(PRECHARGE_POS_GPIO_Port, PRECHARGE_POS_Pin, GPIO_PIN_SET);

    // wait for the precharge time
    HAL_Delay(PRECHARGE_TIME);

    // turn on the tactor
    HAL_GPIO_WritePin(SW_POS_GPIO_Port, SW_POS_Pin, GPIO_PIN_SET);

    // small overlap delay
    HAL_Delay(10);

    // disable precharge
    HAL_GPIO_WritePin(PRECHARGE_POS_GPIO_Port, PRECHARGE_POS_Pin, GPIO_PIN_RESET);

    // turn the led off
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void blinky(uint8_t n, uint32_t delay1, uint32_t delay2, uint32_t delay3)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < n; i++)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay1);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay2);
  }
  HAL_Delay(delay3);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */

  // blink sos a few times
  for (uint8_t i = 0; i < 5; i++)
  {
    // blink sos
    blinky(3, 100, 100, 200);
    blinky(3, 300, 100, 200);
    blinky(3, 100, 100, 1000);
  }

  // then start a reset
  HAL_NVIC_SystemReset();
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
