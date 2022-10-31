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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4 // to indicate that our processor is Cortex-M4
#include "arm_math.h" //complex header file that works differently for different Cortex processor
#include "lab1math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ITM_Port32(n) is a location in memory
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */
  // define our variables and array

  // part 0 - Max value
//  float maxC=0;
//  uint32_t maxIndexC;
//  float maxAsm=0;
//  uint32_t maxIndexAsm;
//  float maxCMSIS=0;
//  uint32_t maxIndexCMSIS;
//  // the max is 88.49 at index 5
//  float array[10] = {48.21, 79.48, 24.27, 28.82, 78.24, 88.49, 31.19, 5.52,
//		  82.70, 77.73};

  //part 1 - Element-wise multiplication
  int N = 1000;
  float array1[1000] = {3.83,44.16,43.78,20.26,26,12.37,34.84,49.82,38.97,28.07,13.7,4.72,5.15,15.15,34.5,22.44,26.27,44.91,49.37,33.52,7.21,41.77,18.49,1.41,35.97,18.59,36.31,13.98,23.12,48.48,18.11,24.04,24.63,20.94,14.32,21.03,40.94,45.53,46.01,27.7,38.91,18.19,35.75,40.33,43.3,46.39,18.29,32.68,45.77,12.49,22.33,22.48,23.12,4.73,9.99,48.5,21.86,10.82,15.61,10.92,29.54,39.16,32.54,46.7,9.18,38.22,2.86,11.13,14.46,29.25,17.17,12.69,34.84,21.24,37.97,33.15,48.37,17.14,41.52,2.54,38.33,6.51,40.02,43.94,5.84,18.31,47.79,16.38,17.92,14.42,23.37,15.07,10.55,13.76,32.94,17.04,10.81,20.36,43.68,22.13,38.49,19.52,37.96,46.59,21.77,29.58,7.14,49.07,24.53,13.21,21.43,19.05,20.54,9.48,44.92,38.6,23.65,32.46,31.19,9.5,18.1,3.57,31.64,30.35,10.46,27.99,9.78,43.72,4.48,48.96,9.47,6.1,42.38,24.55,25.72,3.67,1.27,38.12,23.04,11.81,31.17,13.94,27.32,2.21,1.1,37.42,4,18.36,44.17,25.95,28.24,46.87,19.76,5.56,18.43,25.73,10.72,37.59,7.63,38.2,6.27,41.09,23.35,18.76,40.02,39.1,13.95,11.16,37.52,3.85,38.97,47.94,44.67,39.95,40.46,14.84,16.81,37.15,21.64,27.47,35.18,5.44,32.35,39.35,10.63,1.47,9.85,20.18,24.71,24.99,5.47,27.72,34.12,46.67,34.88,49.35,2.03,5.13,23.24,31.23, 3.83,44.16,43.78,20.26,26,12.37,34.84,49.82,38.97,28.07,13.7,4.72,5.15,15.15,34.5,22.44,26.27,44.91,49.37,33.52,7.21,41.77,18.49,1.41,35.97,18.59,36.31,13.98,23.12,48.48,18.11,24.04,24.63,20.94,14.32,21.03,40.94,45.53,46.01,27.7,38.91,18.19,35.75,40.33,43.3,46.39,18.29,32.68,45.77,12.49,22.33,22.48,23.12,4.73,9.99,48.5,21.86,10.82,15.61,10.92,29.54,39.16,32.54,46.7,9.18,38.22,2.86,11.13,14.46,29.25,17.17,12.69,34.84,21.24,37.97,33.15,48.37,17.14,41.52,2.54,38.33,6.51,40.02,43.94,5.84,18.31,47.79,16.38,17.92,14.42,23.37,15.07,10.55,13.76,32.94,17.04,10.81,20.36,43.68,22.13,38.49,19.52,37.96,46.59,21.77,29.58,7.14,49.07,24.53,13.21,21.43,19.05,20.54,9.48,44.92,38.6,23.65,32.46,31.19,9.5,18.1,3.57,31.64,30.35,10.46,27.99,9.78,43.72,4.48,48.96,9.47,6.1,42.38,24.55,25.72,3.67,1.27,38.12,23.04,11.81,31.17,13.94,27.32,2.21,1.1,37.42,4,18.36,44.17,25.95,28.24,46.87,19.76,5.56,18.43,25.73,10.72,37.59,7.63,38.2,6.27,41.09,23.35,18.76,40.02,39.1,13.95,11.16,37.52,3.85,38.97,47.94,44.67,39.95,40.46,14.84,16.81,37.15,21.64,27.47,35.18,5.44,32.35,39.35,10.63,1.47,9.85,20.18,24.71,24.99,5.47,27.72,34.12,46.67,34.88,49.35,2.03,5.13,23.24,31.23, 3.83,44.16,43.78,20.26,26,12.37,34.84,49.82,38.97,28.07,13.7,4.72,5.15,15.15,34.5,22.44,26.27,44.91,49.37,33.52,7.21,41.77,18.49,1.41,35.97,18.59,36.31,13.98,23.12,48.48,18.11,24.04,24.63,20.94,14.32,21.03,40.94,45.53,46.01,27.7,38.91,18.19,35.75,40.33,43.3,46.39,18.29,32.68,45.77,12.49,22.33,22.48,23.12,4.73,9.99,48.5,21.86,10.82,15.61,10.92,29.54,39.16,32.54,46.7,9.18,38.22,2.86,11.13,14.46,29.25,17.17,12.69,34.84,21.24,37.97,33.15,48.37,17.14,41.52,2.54,38.33,6.51,40.02,43.94,5.84,18.31,47.79,16.38,17.92,14.42,23.37,15.07,10.55,13.76,32.94,17.04,10.81,20.36,43.68,22.13,38.49,19.52,37.96,46.59,21.77,29.58,7.14,49.07,24.53,13.21,21.43,19.05,20.54,9.48,44.92,38.6,23.65,32.46,31.19,9.5,18.1,3.57,31.64,30.35,10.46,27.99,9.78,43.72,4.48,
		  48.96,9.47,6.1,42.38,24.55,25.72,3.67,1.27,38.12,23.04,11.81,31.17,13.94,27.32,2.21,1.1,37.42,4,18.36,44.17,25.95,28.24,46.87,19.76,5.56,18.43,25.73,10.72,37.59,7.63,38.2,6.27,41.09,23.35,18.76,40.02,39.1,13.95,11.16,37.52,3.85,38.97,47.94,44.67,39.95,40.46,14.84,16.81,37.15,21.64,27.47,35.18,5.44,32.35,39.35,10.63,1.47,9.85,20.18,24.71,24.99,5.47,27.72,34.12,46.67,34.88,49.35,2.03,5.13,23.24,31.23, 3.83,44.16,43.78,20.26,26,12.37,34.84,49.82,38.97,28.07,13.7,4.72,5.15,15.15,34.5,22.44,26.27,44.91,49.37,33.52,7.21,41.77,18.49,1.41,35.97,18.59,36.31,13.98,23.12,48.48,18.11,24.04,24.63,20.94,14.32,21.03,40.94,45.53,46.01,27.7,38.91,18.19,35.75,40.33,43.3,46.39,18.29,32.68,45.77,12.49,22.33,22.48,23.12,4.73,9.99,48.5,21.86,10.82,15.61,10.92,29.54,39.16,32.54,46.7,9.18,38.22,2.86,11.13,14.46,29.25,17.17,12.69,34.84,21.24,37.97,33.15,48.37,17.14,41.52,2.54,38.33,6.51,40.02,43.94,5.84,18.31,47.79,16.38,17.92,14.42,23.37,15.07,10.55,13.76,32.94,17.04,10.81,20.36,43.68,22.13,38.49,19.52,37.96,46.59,21.77,29.58,7.14,49.07,24.53,13.21,21.43,19.05,20.54,9.48,44.92,38.6,23.65,32.46,31.19,9.5,18.1,3.57,31.64,30.35,10.46,27.99,9.78,43.72,4.48,48.96,9.47,6.1,42.38,24.55,25.72,3.67,1.27,38.12,23.04,11.81,31.17,13.94,27.32,2.21,1.1,37.42,4,18.36,44.17,25.95,28.24,46.87,19.76,5.56,18.43,25.73,10.72,37.59,7.63,38.2,6.27,41.09,23.35,18.76,40.02,39.1,13.95,11.16,37.52,3.85,38.97,47.94,44.67,39.95,40.46,14.84,16.81,37.15,21.64,27.47,35.18,5.44,32.35,39.35,10.63,1.47,9.85,20.18,24.71,24.99,5.47,27.72,34.12,46.67,34.88,49.35,2.03,5.13,23.24,31.23, 3.83,44.16,43.78,20.26,26,12.37,34.84,49.82,38.97,28.07,13.7,4.72,5.15,15.15,34.5,22.44,26.27,44.91,49.37,33.52,7.21,41.77,18.49,1.41,35.97,18.59,36.31,13.98,23.12,48.48,18.11,24.04,24.63,20.94,14.32,21.03,40.94,45.53,46.01,27.7,38.91,18.19,35.75,40.33,43.3,46.39,18.29,32.68,45.77,12.49,22.33,22.48,23.12,4.73,9.99,48.5,21.86,10.82,15.61,10.92,29.54,39.16,32.54,46.7,9.18,38.22,2.86,11.13,14.46,29.25,17.17,12.69,34.84,21.24,37.97,33.15,48.37,17.14,41.52,2.54,38.33,6.51,40.02,43.94,5.84,18.31,47.79,16.38,17.92,14.42,23.37,15.07,10.55,13.76,32.94,17.04,10.81,20.36,43.68,22.13,38.49,19.52,37.96,46.59,21.77,29.58,7.14,49.07,24.53,13.21,21.43,19.05,20.54,9.48,44.92,38.6,23.65,32.46,31.19,9.5,18.1,3.57,31.64,30.35,10.46,27.99,9.78,43.72,4.48,48.96,9.47,6.1,42.38,24.55,25.72,3.67,1.27,38.12,23.04,11.81,31.17,13.94,27.32,2.21,1.1,37.42,4,18.36,44.17,25.95,28.24,46.87,19.76,5.56,18.43,25.73,10.72,37.59,7.63,38.2,6.27,41.09,23.35,18.76,40.02,39.1,13.95,11.16,37.52,3.85,38.97,47.94,44.67,39.95,40.46,14.84,16.81,37.15,21.64,27.47,35.18,5.44,32.35,39.35,10.63,1.47,9.85,20.18,24.71,24.99,5.47,27.72,34.12,46.67,34.88,49.35,2.03,5.13,23.24,31.23};
  float productsC[N];
  float productsAsm[N];
  float productsCMSIS[N];

  //part 2 - Standard Deviation
  float stdC = 0;
  float stdASM = 0;
  float stdCMSIS = 0;


  // PERSONNAL CODE
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  ITM_Port32(31)  = 1; // sends data value 1 to ITM port 31
//	  for (uint32_t i=0; i<1000; i++)
//		  cMax(&array, 10, &maxC, &maxIndexC);
//	  ITM_Port32(31) = 2; // sends data value 2 to ITM port 31
//	  for (uint32_t i=0; i<1000; i++)
//		  asmMax(&array, 10, &maxAsm, &maxIndexAsm);
//	  ITM_Port32(31) = 3; // sends data value 2 to ITM port 31
//	  for (uint32_t i=0; i<1000; i++)
//		  arm_max_f32(&array, 10, &maxCMSIS, &maxIndexCMSIS);

	  ITM_Port32(31) = 1;
	  for (uint32_t i=0; i<1000; i++)
		  cMultiply(&array1, &array1, N, &productsC);
	  ITM_Port32(31) = 2;
	  for (uint32_t i=0; i<1000; i++)
		  arm_mult_f32(&array1, &array1, &productsCMSIS, 10);
	  ITM_Port32(31) = 3;
	  for (uint32_t i=0; i<1000; i++)
		  asmMultiply(&array1, &array1, N, &productsAsm);
	  ITM_Port32(31) = 4;
	  for (uint32_t i=0; i<1000; i++)
		  cstd(&array1, N, &stdC);
	  ITM_Port32(31) = 5;
	  for (uint32_t i=0; i<1000; i++)
		  arm_std_f32(&array1, N, &stdCMSIS);
	  ITM_Port32(31) = 6;
	  for (uint32_t i=0; i<1000; i++)
	 		asmstd(&array1, N, &stdASM);
	  ITM_Port32(31) = 7;

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
