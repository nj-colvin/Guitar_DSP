/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

#include "peripherals.h"
#include "SPI.h"
#include "UART.h"
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
signed int dataIn;
signed int dataOut;
unsigned char dataTx[8], dataRx[8];

signed int sampleBuffer[20000];
unsigned int sampleBufferIndex;

unsigned int control[2];

long a, b, c, d;
int a0 = 190; // >> 8
int b0 = 175; // >> 8
int c0 = 160;
int d0 = 147;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Configure_MCP3561R(unsigned char *buffer, char readback);

void Clean(void);
void Delay(void);
void Flange(void);
void Phaser_Stage(int stageNumber, int a);
void Phaser(void);
void Reverb(void);
void Sin_Delay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int lookUp[256] = {
		0x8000,0x8324,0x8647,0x896a,0x8c8b,0x8fab,0x92c7,0x95e1,
		0x98f8,0x9c0b,0x9f19,0xa223,0xa527,0xa826,0xab1f,0xae10,
		0xb0fb,0xb3de,0xb6b9,0xb98c,0xbc56,0xbf17,0xc1cd,0xc47a,
		0xc71c,0xc9b3,0xcc3f,0xcebf,0xd133,0xd39a,0xd5f5,0xd842,
		0xda82,0xdcb3,0xded7,0xe0eb,0xe2f1,0xe4e8,0xe6cf,0xe8a6,
		0xea6d,0xec23,0xedc9,0xef5e,0xf0e2,0xf254,0xf3b5,0xf504,
		0xf641,0xf76b,0xf884,0xf989,0xfa7c,0xfb5c,0xfc29,0xfce3,
		0xfd89,0xfe1d,0xfe9c,0xff09,0xff61,0xffa6,0xffd8,0xfff5,
		0xffff,0xfff5,0xffd8,0xffa6,0xff61,0xff09,0xfe9c,0xfe1d,
		0xfd89,0xfce3,0xfc29,0xfb5c,0xfa7c,0xf989,0xf884,0xf76b,
		0xf641,0xf504,0xf3b5,0xf254,0xf0e2,0xef5e,0xedc9,0xec23,
		0xea6d,0xe8a6,0xe6cf,0xe4e8,0xe2f1,0xe0eb,0xded7,0xdcb3,
		0xda82,0xd842,0xd5f5,0xd39a,0xd133,0xcebf,0xcc3f,0xc9b3,
		0xc71c,0xc47a,0xc1cd,0xbf17,0xbc56,0xb98c,0xb6b9,0xb3de,
		0xb0fb,0xae10,0xab1f,0xa826,0xa527,0xa223,0x9f19,0x9c0b,
		0x98f8,0x95e1,0x92c7,0x8fab,0x8c8b,0x896a,0x8647,0x8324,
		0x8000,0x7cdb,0x79b8,0x7695,0x7374,0x7054,0x6d38,0x6a1e,
		0x6707,0x63f4,0x60e6,0x5ddc,0x5ad8,0x57d9,0x54e0,0x51ef,
		0x4f04,0x4c21,0x4946,0x4673,0x43a9,0x40e8,0x3e32,0x3b85,
		0x38e3,0x364c,0x33c0,0x3140,0x2ecc,0x2c65,0x2a0a,0x27bd,
		0x257d,0x234c,0x2128,0x1f14,0x1d0e,0x1b17,0x1930,0x1759,
		0x1592,0x13dc,0x1236,0x10a1,0xf1d,0xdab,0xc4a,0xafb,
		0x9be,0x894,0x77b,0x676,0x583,0x4a3,0x3d6,0x31c,
		0x276,0x1e2,0x163,0xf6,0x9e,0x59,0x27,0xa,
		0x0,0xa,0x27,0x59,0x9e,0xf6,0x163,0x1e2,
		0x276,0x31c,0x3d6,0x4a3,0x583,0x676,0x77b,0x894,
		0x9be,0xafb,0xc4a,0xdab,0xf1d,0x10a1,0x1236,0x13dc,
		0x1592,0x1759,0x1930,0x1b17,0x1d0e,0x1f14,0x2128,0x234c,
		0x257d,0x27bd,0x2a0a,0x2c65,0x2ecc,0x3140,0x33c0,0x364c,
		0x38e3,0x3b85,0x3e32,0x40e8,0x43a9,0x4673,0x4946,0x4c21,
		0x4f04,0x51ef,0x54e0,0x57d9,0x5ad8,0x5ddc,0x60e6,0x63f4,
		0x6707,0x6a1e,0x6d38,0x7054,0x7374,0x7695,0x79b8,0x7cdb};

unsigned int lookupIndex = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned char buffer[24];
	unsigned int effectNumber = 0;
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
  /* USER CODE BEGIN 2 */
  GPIO_Setup();
  TIMER4_Setup();
  TIMER5_Setup();
  TIMER12_Setup();
  SPI_Setup();
  UART_Setup();
  ADC_Setup();
  TIMER6_Setup();


  sprintf((char*)buffer, "Start\r\n");
  UART_Transmit(buffer, 7);

  Configure_MCP3561R(buffer, 1);

  // read from adc

  dataTx[0] = 0x41; //01 0000 01 - command byte: static read at adcdata

  SPI_Communicate(dataTx, dataRx, 4);

  HAL_Delay(100);

  while(!UART_Buffer_Is_Free());

  sprintf((char*)buffer, "%2x %2x %2x %2x\r\n", dataRx[0], dataRx[1], dataRx[2], dataRx[3]);
  UART_Transmit(buffer, 13);


//  while(1){
//
//	  SPI_Communicate(dataTx, dataRx, 4);
//
//	  HAL_Delay(100);
//
//	  GPIOA->BSRR |= (GPIO_BSRR_BS4);
//
//	  while(!UART_Buffer_Is_Free());
//
//	  sprintf((char*)buffer, "%2x %2x %2x %2x\r\n", dataRx[0], dataRx[1], dataRx[2], dataRx[3]);
//	  UART_Transmit(buffer, 13);
//	  dataRx[0] = 0;
//
//	  HAL_Delay(1000);
//  }


  // resync sync pulses (may get out of sync with ADC communication)
  TIM3->CNT = 0;
  dataTx[0] = 0;
  //enable sync pulse output pulse
  TIM2->CCMR1 |= (7 << TIM_CCMR1_OC1M_Pos);

  unsigned int Ucounter;

  effectNumber = 5;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  if(ADC1->SR & ADC_SR_EOC){

		  ADC1->SR &= ~ADC_SR_EOC;

		  // read/write
		  SPI_Communicate(dataTx, dataRx, 3);
		  //wait for SPI
		  while(SPI_In_Progress());

//		  dataIn = (dataRx[1] << 8) + dataRx[2];
		  dataIn = ADC1->DR - 0x7fff;

		  // Effects Here

		  switch(effectNumber){
		  	  case 0:
		  		  Clean();
		  		  break;
		  	  case 1:
		  		  Delay();
		  		  break;
		  	  case 2:
		  		  Flange();
				  break;
		  	  case 3:
				  Phaser();
				  break;
			  case 4:
				  Reverb();
				  break;
			  case 5:
				  Sin_Delay();
				  break;

		  }
		  // End Effects

		  dataTx[1] = dataOut >> 8;
		  dataTx[2] = dataOut;

		  if (Ucounter >= 32000 && UART_Buffer_Is_Free()){
			  sprintf((char*)buffer, "%2x %2x %2x %2x\r\n", dataRx[0], dataRx[1], dataRx[2], dataRx[3]);
			  UART_Transmit(buffer, 13);
			  Ucounter = 0;
		  }
		  else{
			  Ucounter++;
		  }
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Clean(void){
	dataOut = dataIn + 0x7FFF;
}

void Delay(void){
	dataOut = sampleBuffer[sampleBufferIndex] + 0x7FFF;

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 5000){
	  sampleBufferIndex = 0;
	}
}

void Flange(void){
	dataOut = sampleBuffer[(sampleBufferIndex + (lookUp[TIM12->CNT]>>9)) % 1000] + 0x7FFF;

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 1000){
	  sampleBufferIndex = 0;
	}
}

void Phaser_Stage(int stageNumber, int a){
	int inputOffset = (stageNumber - 1) * 2;
	int outputOffset = stageNumber * 2;

	sampleBuffer[sampleBufferIndex + outputOffset] = (a * (sampleBuffer[!sampleBufferIndex + outputOffset] - sampleBuffer[sampleBufferIndex + inputOffset]) + \
		  (sampleBuffer[!sampleBufferIndex + inputOffset] << 8)) >> 8;

	if (sampleBuffer[sampleBufferIndex + outputOffset] > 32767){
	  sampleBuffer[sampleBufferIndex + outputOffset] = 32767;
	}
	else if (sampleBuffer[sampleBufferIndex + outputOffset] < -32768){
	  sampleBuffer[sampleBufferIndex + outputOffset] = -32768;
	}
}

void Phaser(void){
	  sampleBuffer[sampleBufferIndex] = dataIn;

	  a = a0 + (lookUp[TIM12->CNT] >> 10);
	  b = b0 + (lookUp[TIM12->CNT] >> 10);
	  c = c0 + (lookUp[TIM12->CNT] >> 10);
	  d = d0 + (lookUp[TIM12->CNT] >> 10);

	  Phaser_Stage(1, a);
	  Phaser_Stage(2, b);
	  Phaser_Stage(3, c);
	  Phaser_Stage(4, d);

	  dataOut = sampleBuffer[sampleBufferIndex+8] + 0x7FFF;

	  sampleBufferIndex++;
	  if (sampleBufferIndex >= 2){
		  sampleBufferIndex = 0;
	  }
}

void Reverb(void){
	dataOut = ((sampleBuffer[sampleBufferIndex] + \
		  sampleBuffer[(sampleBufferIndex + 2000) % 10000] + \
		  sampleBuffer[(sampleBufferIndex + 4000) % 10000] + \
		  sampleBuffer[(sampleBufferIndex + 7500) % 10000]) >> 2) + 0x7FFF;

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 10000){
	  sampleBufferIndex = 0;
	}
}

void Sin_Delay(void){
	dataOut = sampleBuffer[(sampleBufferIndex + (lookUp[TIM12->CNT]>>6)) % 10000] + 0x7FFF;

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 10000){
	  sampleBufferIndex = 0;
	}
}


void TIM3_IRQHandler(void){
	dataTx[1] = (lookUp[lookupIndex] >> 8);
	dataTx[2] = lookUp[lookupIndex];
	lookupIndex++;
	if (lookupIndex >= 200){
		lookupIndex = 0;
	}

	TIM3->SR &= ~TIM_SR_UIF;
}



void Configure_MCP3561R(unsigned char *buffer, char readback){
	  // configure ADC registers

	  dataTx[0] = 0x46; //01 0001 10 - command byte: write starting at config0
	  dataTx[1] = 0xE3; // 1110 0011 - config 0: internal reference, start adc conversions
	  dataTx[2] = 0x00; // 0000 0000 - config 1: 16bit adc
	  dataTx[3] = 0x8B; // 1000 1011 - config 2: gain = 1
	  dataTx[4] = 0xC0; // 1100 0000 - config 3: continuous conversion, 24bit adc register, no calibration
	  dataTx[5] = 0x73; // 0111 0011 - IRQ
	  dataTx[6] = 0x08; // 0000 1000 - Multiplexer: V+ = CH0, V- = GND

	  SPI_Communicate(dataTx, dataRx, 7);

	  HAL_Delay(100);

	  GPIOA->BSRR |= (GPIO_BSRR_BS4);

	  // read back adc oncfig

	  if (readback){
		  dataTx[0] = 0x47; //01 0001 11 - command byte: read starting at config0

		  SPI_Communicate(dataTx, dataRx, 7);

		  HAL_Delay(100);

		  GPIOA->BSRR |= (GPIO_BSRR_BS4);

		  for (int i = 0; i < 7; i++){
			  while(!UART_Buffer_Is_Free());

			  sprintf((char*)buffer, "%3x\r\n", dataRx[i]);
			  UART_Transmit(buffer, 5);
		  }
	  }
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
