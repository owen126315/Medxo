
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define mpu9250Address	0xD0			//(0x68<<1|0)
#define ACC_SENSITIVITY 16384.0
#define GYR_SENSITIVITY 65.5
#define dt 0.01       						//sampling rate
#define SAMPLES 128      					//Must be a power of 2
#define SAMPLING_FREQUENCY 64		 	//Hz
#define M_PI 3.141592654
#define SAMPLES 128       //Must be a power of 2
#define SAMPLING_FREQUENCY 64 //Hz
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch,FILE *f);
void MPU_Config(void);
void I2C_readData(void);
void ComplementaryFilter(void);
void Windowing(double *vData, int samples);
uint8_t Exponent(uint16_t value);
void Swap(double *x, double *y);
void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power);
void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples);
double MajorPeak(double *vD, uint16_t samples, int samplingFreq);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t i2cBuf[8];
int16_t aX, aY, aZ, gX, gY, gZ;
//float accelX_9250, accelY_9250, accelZ_9250, gyroX_9250, gyroY_9250, gyroZ_9250;
float pitch, roll;
double rData[SAMPLES], iData[SAMPLES];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		static int n  = 0;
		I2C_readData();
		ComplementaryFilter();
		rData[n] = pitch;
		
		if(++n == SAMPLES){
			Windowing(rData, SAMPLES);
			Compute(rData, iData, SAMPLES, Exponent(SAMPLES));
			ComplexToMagnitude(rData, iData, SAMPLING_FREQUENCY);
			double peak = MajorPeak(rData, SAMPLES, SAMPLING_FREQUENCY);
			int freq = round(peak);
			float amplitude = ((float)rData[freq] - (int)rData[freq])*100;
			printf(" \t%d \t%d.%d", freq, (int)rData[freq], (int)amplitude);
			for(int i = 0; i<SAMPLES; i++){
				iData[i] = 0;
			}
			
			
			n = 0;
		}
		printf("\r\n");
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	for(int i = 0; i<SAMPLES; i++){
    rData[i] = 0;
    iData[i] = 0;
  }
	
	for(uint8_t i = 0; i<255; i++){
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK){
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			break;
		}
	}
	//2. i2c write example
	//a) MPU_Config
	MPU_Config();
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1925;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE *f){
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart1,temp,1,2);
		return ch;
}

void MPU_Config(){
	i2cBuf[0] = 0x6B;			//PWR_MGMT_1	
	i2cBuf[1] = 0x00;			//prevent sleep mode
	HAL_I2C_Master_Transmit(&hi2c1, mpu9250Address, i2cBuf, 2, 10);
	
	i2cBuf[0] = 0x1B;			//ACCEL_CONFIG
	i2cBuf[1] = 0x08;			//range: +-500 deg/sec
	HAL_I2C_Master_Transmit(&hi2c1, mpu9250Address, i2cBuf, 2, 10);
	
	i2cBuf[0] = 0x1C;			//ACCEL_CONFIG
	i2cBuf[1] = 0x00;			//range: +-2g
	HAL_I2C_Master_Transmit(&hi2c1, mpu9250Address, i2cBuf, 2, 10);
}

void I2C_readData(){
	i2cBuf[0] = 0x3B; 		//read accelX request
		HAL_I2C_Master_Transmit(&hi2c1, mpu9250Address, i2cBuf, 1, 10);
		i2cBuf[1] = 0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu9250Address, &i2cBuf[1], 6, 10);
		aX = (i2cBuf[1]<<8 | i2cBuf[2]);
		aY = (i2cBuf[3]<<8 | i2cBuf[4]);
		aZ = (i2cBuf[5]<<8 | i2cBuf[6]);
		
		i2cBuf[0] = 0x43; 		//read gyroX request
		HAL_I2C_Master_Transmit(&hi2c1, mpu9250Address, i2cBuf, 1, 10);
		i2cBuf[1] = 0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu9250Address, &i2cBuf[1], 6, 10);
		gX = (i2cBuf[1]<<8 | i2cBuf[2]);
		gY = (i2cBuf[3]<<8 | i2cBuf[4]);
		gZ = (i2cBuf[5]<<8 | i2cBuf[6]);
		printf("%d \t%d \t%d \t%d \t%d \t%d", aX, aY, aZ, gX, gY, gZ);
}

void ComplementaryFilter(){
  float pitchAcc, rollAcc;
  // Integrate the gyroscope data -> int(angularSpeed) = angle
  pitch += ((float)gX / GYR_SENSITIVITY) * dt; // Angle around the X-axis
  roll -= ((float)gY / GYR_SENSITIVITY) * dt;    // Angle around the Y-axis
  
  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(aX) + abs(aY) + abs(aZ);

  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)aY, (float)aZ) * 180.0 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;
    
    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)aX, (float)aZ) * 180.0 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;
    //printf("pitch:%d roll:%d\r\n", (int)pitch, (int)roll);
  }
}

void Windowing(double *vData, int samples){
  for(int i = 0; i<samples; i++){
    double ratio = (double)i/((double)samples-1);
    double weighingFactor = 1.0;
    weighingFactor = 0.54 - (0.46 * cos(2 * M_PI * ratio));
    vData[i] *= weighingFactor;
    vData[samples - (i+1)] *= weighingFactor;
  }
}

uint8_t Exponent(uint16_t value){
  uint8_t result = 0;
  while(((value >> result) & 1) != 1) result++;
  return result;
}

void Swap(double *x, double *y){
  double temp = *x;
  *x = *y;
  *y = temp;
}

void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power){
  // Computes in-place complex-to-complex FFT
  // Reverse bits
  uint16_t j = 0;
  for (uint16_t i = 0; i < (samples - 1); i++) {
    if (i < j) {
      Swap(&vReal[i], &vReal[j]);
    }
    uint16_t k = (samples >> 1);
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  // Compute the FFT
  double c1 = -1.0;
  double c2 = 0.0;
  uint16_t l2 = 1;
  for (uint8_t l = 0; (l < power); l++) {
    uint16_t l1 = l2;
    l2 <<= 1;
    double u1 = 1.0;
    double u2 = 0.0;
    for (j = 0; j < l1; j++) {
      for (uint16_t i = j; i < samples; i += l2) {
        uint16_t i1 = i + l1;
        double t1 = u1 * vReal[i1] - u2 * vImag[i1];
        double t2 = u1 * vImag[i1] + u2 * vReal[i1];
        vReal[i1] = vReal[i] - t1;
        vImag[i1] = vImag[i] - t2;
        vReal[i] += t1;
        vImag[i] += t2;
      }
      double z = ((u1 * c1) - (u2 * c2));
      u2 = ((u1 * c2) + (u2 * c1));
      u1 = z;
    }
    c2 = sqrt((1.0 - c1) / 2.0);
    c2 = -c2;
    c1 = sqrt((1.0 + c1) / 2.0);
  }
}

void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples){
  for(uint16_t i = 0; i<samples; i++){
    vReal[i] = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
  }
}

double MajorPeak(double *vD, uint16_t samples, int samplingFreq){
  int maxY = 0;
  uint16_t IndexOfMaxY = 0;
  for(uint16_t i = 1; i < ((samples >> 1)+1); i++){
    if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) {
      if (vD[i] > maxY) {
        maxY = vD[i];
        IndexOfMaxY = i;
      }
    }
  }
  
  double delta = 0.5 * ((vD[IndexOfMaxY-1] - vD[IndexOfMaxY+1]) / (vD[IndexOfMaxY-1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY+1]));
  double interpolatedX = ((IndexOfMaxY + delta)  * (double)samplingFreq) / (samples-1);
  
  if(IndexOfMaxY==(samples >> 1)) //To improve calculation on edge values
  interpolatedX = ((IndexOfMaxY + delta)  * (double)samplingFreq) / (samples);
  
  // returned value: interpolated frequency peak apex
  return(interpolatedX);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
