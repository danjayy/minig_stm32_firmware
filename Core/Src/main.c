/* USER CODE BEGIN Header */
/*
 * Author: Albert Carrasco
 * Last review: 19/3/2024
 *
 * DESCRIPTION:
 * Code for STM32G431KB
 *
 */
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "uart_utils.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Global variables for UART reception
uint8_t rx_buffer[256]; // Buffer to store incoming data (adjust size as needed)
uint16_t rx_index = 0;  // Current index in the buffer
uint8_t rx_data;        // Single byte for interrupt reception
volatile uint8_t rx_complete = 0; // Flag to indicate complete message

uint8_t state=0;				//State of the state machine

uint8_t i;
uint16_t currentPoint[6] = {31874,31874,31874,31874,31874,31874};		//current value of CCR for the thrusters
uint16_t currentLightPoint = 31874; //current value of CCR for the lights
uint8_t message[34];			//message received by USART. Longitud max==28
uint16_t value[7];
uint16_t setPoint[6];			//CCR value we want to achieve for the motors
uint16_t setLightPoint;			//CCR value we want to achieve for the lights

int16_t slope[6];				//step between each value for the motors
int16_t lightslope;

// Camera PWM calculation parameters
uint16_t camera_ccr;

int16_t maxSlope = 1050;		//max step between each value of CCR
int16_t minSlope = -1050;		//min step between each value of CCR

int16_t error;					//Sum of slopes of motors
int16_t errorlights;			//Sum of slopes of lights

//Lookup table of CCR values:
uint16_t CCRtable[] =
		{23374,23545,23603,23655,23702,23753,23803,23841,23878,23927,23977,24017,24057,24104,24154,24212,
		24269,24323,24366,24407,24444,24482,24525,24570,24635,24676,24711,24776,24841,24874,24907,24959,
		25009,25056,25096,25130,25170,25235,25299,25357,25409,25450,25490,25593,25624,25654,25691,25734,
		25801,25862,25902,25940,25977,26021,26103,26161,26200,26231,26261,26357,26390,26422,26462,26505,
		26554,26606,26671,26723,26771,26814,26857,26968,27020,27077,27137,27202,27260,27327,27391,27443,
		27500,27556,27608,27660,27713,27798,27856,27918,27982,28047,28112,28183,28258,28328,28394,28468,
		28534,28594,28652,28710,28786,28858,28938,29025,29104,29182,29247,29312,29384,29466,29553,29624,
		29698,29784,29878,29964,30038,30116,30209,30304,30399,30503,30624,30728,30847,30987,31120,31874,
		32621,32758,32917,33050,33148,33273,33367,33465,33554,33637,33728,33822,33906,33989,34082,34149,
		34215,34308,34390,34464,34542,34609,34660,34762,34836,34927,34996,35080,35145,35204,35277,35337,
		35391,35442,35533,35600,35666,35736,35809,35879,35934,35993,36056,36128,36183,36244,36307,36358,
		36414,36473,36528,36601,36664,36715,36770,36832,36898,36934,36971,37060,37134,37174,37209,37284,
		37346,37388,37424,37459,37495,37534,37574,37625,37672,37713,37788,37846,37876,37906,38002,38032,
		38062,38103,38158,38210,38262,38317,38359,38396,38438,38489,38551,38603,38636,38669,38744,38801,
		38850,38892,38933,38974,39016,39067,39117,39164,39205,39242,39281,39325,39366,39401,39435,39485,
		39539,39605,39644,39680,39736,39797,39847,39886,39919,39953,39992,40031,40102,40146,40181,40374};

int32_t CamCCRtable[] =
		{28333, 17000, 8500, 5667, 4250, 3400, 2833, 2429, 2125, 1889, 1700, 1545, 1417, 1308, 1214, 1133,
		1062, 1000, 944, 895, 850, 810, 773, 739, 708, 680, 654, 630, 607, 586, 567};

int32_t CamARRtable[] =
		{56666, 33999, 16999, 11333, 8499, 6799, 5666, 4857, 4249, 3777, 3399, 3090, 2833, 2615, 2428, 2266,
		2124, 1999, 1888, 1789, 1699, 1619, 1545, 1478, 1416, 1359, 1307, 1259, 1214, 1172, 1133};

uint16_t check;		//Calculated value of checksum

uint8_t length;				//length of the message read

//UART_HandleTypeDef huart1; // UART1 handler

// Initialize UART interrupt reception
void startUartReceive(UART_HandleTypeDef *huart) {
    // Start UART reception in interrupt mode for 1 byte
    HAL_UART_Receive_IT(huart, &rx_data, 1);
}
// Non-blocking function to read UART data
uint16_t readLine(UART_HandleTypeDef *huart, uint8_t *data, uint16_t max_len) {
    uint16_t count = 0;

    // Check if a complete message is ready
    if (rx_complete) {
        // Copy data from rx_buffer to user-provided data buffer
        for (count = 0; count < rx_index && count < max_len; count++) {
            data[count] = rx_buffer[count];
        }

        // Reset buffer and flags
        rx_index = 0;
        rx_complete = 0;

        // Restart UART reception
        HAL_UART_Receive_IT(huart, &rx_data, 1);
    }

    return count;
}


//uint16_t readLine(UART_HandleTypeDef *huart, uint8_t *data, uint16_t max_len, uint16_t timeout) {
//	uint16_t count = 0;
//	while (HAL_UART_Receive(huart, &data[count], 1, timeout) == HAL_OK && count < max_len) {
//		count++;
//		if (data[count+1] == 'e' && data[count+2]== 'e')
//			break;
//	}
//	return count;
//}

uint8_t light;
uint8_t camera;
uint16_t checksum;	//Value if checksum received in cameras' message

const uint8_t timeout = 2; //Timeout if PC crashes
//Timing for lights & cameras:
const float t1 = 0.5;
const float t2 = 1;
const float t3 = 1.5;

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  startUartReceive(&huart1);	// Initialising the uart receive interrupt callback
  //START PWM OUTPUTS:
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		//Motor 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);		//Motor 2
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);		//Motor 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);		//Motor 4
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  		//Motor 5
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);		//Motor 6

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		//Lights
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);		//Camera

  //Send a PWM with Duty Cycle 1500 microseconds to start the T200 for at least 7 seconds:
  TIM2->CCR1 = 31874;	//Motor 1
  TIM2->CCR2 = 31874;	//Motor 2
  TIM16->CCR1 = 31874;	//Motor 3
  TIM3->CCR2 = 31874;	//Motor 4
  TIM3->CCR1 = 31874;  	//Motor 5
  TIM17->CCR1 = 31874;	//Motor 6
  HAL_Delay(7000);

  //Turn off lights:
  TIM3->CCR3 = 23374; 	//Lights initial pwm
  // Disable PWM output initially for the Camera
  TIM4->CCER &= ~TIM_CCER_CC1E;


  HAL_TIM_Base_Start(&htim6);	//Timer timeout start

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(state) {
		case 0:
			memset(message,0,34);		//Clean the array before writing on it
			//Receive message: check Id and length
			length=readLine(&huart1, message, sizeof(message));

			//Cameras & lights:
			if(length==16 && message[0]=='c' && message[1]=='c') {
				HAL_TIM_Base_Stop(&htim6);
				TIM6->CNT = 0;	//Reset timer

				camera = (message[3]-'0')*10 + (message[4]-'0');		//value for cameras
				light = (message[6]-'0')*100 + (message[7]-'0')*10 + (message[8]-'0');	//Value for lights
				checksum = (message[10]-'0')*100 + (message[11]-'0')*10 + (message[12]-'0');	//Checksum received
				state=1;
			}


			else if(length==34 && message[0]=='a' && message[1]=='a') {
				HAL_TIM_Base_Stop(&htim6);
				TIM6->CNT = 0;	//Reset timer

				value[0] = (message[3]-'0')*100 + (message[4]-'0')*10 + (message[5]-'0');			//Value for motor 1
				value[1] = (message[7]-'0')*100 + (message[8]-'0')*10 + (message[9]-'0');			//Value for motor 2
				value[2] = (message[11]-'0')*100 + (message[12]-'0')*10 + (message[13]-'0');		//Value for motor 3
				value[3] = (message[15]-'0')*100 + (message[16]-'0')*10 + (message[17]-'0');		//Value for motor 4
				value[4] = (message[19]-'0')*100 + (message[20]-'0')*10 + (message[21]-'0');		//Value for motor 5
				value[5] = (message[23]-'0')*100 + (message[24]-'0')*10 + (message[25]-'0');		//Value for motor 6
				value[6] = (message[27]-'0')*1000 + (message[28]-'0')*100 + (message[29]-'0')*10 + (message[30]-'0');		//Checksum received
				state=1;
	  		  }

	  		  else {
	  			  //Security protocol, if PC crashes stops thrusters (2 seconds of no data received):
	  			  if(__HAL_TIM_GET_COUNTER(&htim6) >= timeout*32768) {
	  				  TIM2->CCR1 = 31874;	//Motor 1
	  				  TIM2->CCR2 = 31874;	//Motor 2
	  				  TIM16->CCR1 = 31874;	//Motor 3
	  				  TIM3->CCR2 = 31874;	//Motor 4
	  				  TIM3->CCR1 = 31874;  	//Motor 5
	  				  TIM17->CCR1 = 31874;	//Motor 6

	  				  //Update value in CurrentPoint variable:
	  				  for(i=0;i<=5;i++) {
	  					  currentPoint[i]=31874;
	  				  }

	  				  HAL_TIM_Base_Stop(&htim6);
	  				  TIM6->CNT = 0;	//Reset timer
	  			  }
	  			  state=0;
	  		  }
		  break;

		  case 1:
			  //Check if values received are inside range:

			  if(message[0]=='a' && message[1]=='a') {
				  for(i=0;i<=5;i++) {
					  if(value[i]>255 || value[i]<0) {
	  				  	  state=0;
	  				  	  break;
	  				  }
	  				  else {
	  					  state=2;
	  				  }
				  }
			  }

			  else if(message[0]=='c' && message[1]=='c') {
				  for(i=0;i<=1;i++) {
					  if(light>255 || light<0) {
						  state=0;
						  break;
					  }
	  				  if (camera>30 || camera<0) {
	  					  state=0;
	  					  break;
	  				  }
	  				  else {
	  					  state=2;
	  				  }
				  }
			  }

		  break;

		  case 2:
			  //Check if checksum is inside range and calculate checksum to compare:

			  if(message[0]=='a' && message[1]=='a') {
				  if(value[6]>=0 && value[6]<=1530) {
					  check = value[0] + value[1] + value[2] + value[3] + value[4] + value[5];
					  if (check==value[6]) {
						  state = 3;
					  }
					  else {
						  state=0;
					  }
				  }
				  else {
					  state=0;
				  }
			  }

			  else if(message[0]=='c' && message[1]=='c') {
				  if(checksum>=0 && checksum<=285) {
					  check = light + camera;
					  if (check==checksum) {
						  state = 3;
						  HAL_TIM_Base_Start(&htim7);	//Start timer for lights & cameras timing [VERIFY IF THIS IS NECESSARY]
					  }
					  else {
						  state=0;
					  }
				  }
				  else {
					  state=0;
				  }
			  }
		  break;

		  case 3:
			  //Process to send PWM values

			  if(message[0]=='a' && message[1]=='a') {
				  //setPoint equals to the value in the LUT from position 0 to 255:
				  setPoint[0] = CCRtable[value[0]];
				  setPoint[1] = CCRtable[value[1]];
				  setPoint[2] = CCRtable[value[2]];
				  setPoint[3] = CCRtable[value[3]];
				  setPoint[4] = CCRtable[value[4]];
				  setPoint[5] = CCRtable[value[5]];

				  for(i=0;i<=5;i++) {
					  slope[i]=setPoint[i]-currentPoint[i];

					  //Limit the values of slope:
					  if(slope[i]>maxSlope) {
						  slope[i]=maxSlope;
					  }

					  else if (slope[i]<minSlope) {
						  slope[i]=minSlope;
					  }

					  //Values to send:
					  currentPoint[i]+=slope[i];
				  }


				  //Send values:
				  TIM2->CCR1 = currentPoint[0];		//Motor 1
				  TIM2->CCR2 = currentPoint[1];		//Motor 2
				  TIM16->CCR1 = currentPoint[2];	//Motor 3
				  TIM3->CCR2 = currentPoint[3];		//Motor 4
				  TIM3->CCR1 = currentPoint[4];  	//Motor 5
				  TIM17->CCR1 = currentPoint[5];	//Motor 6

//				  HAL_Delay(200);
				  error=0;	//Reset error

				  for(i=0;i<=5;i++) {
					  error=error+slope[i];
				  }
				  //When all setPoints are achieved return to initial state:
				  if(error==0) {
					  HAL_TIM_Base_Start(&htim6);
					  state=0;
				  }
				  else {
					  state=3;
				  }
			  }

			  if(message[0]=='c' && message[1]=='c') {
//				  if(message[0]=='c' && message[1]=='c') {
					  //setPoint equals to the value in the LUT from position 0 to 255:
					  setLightPoint = CCRtable[light];
					  lightslope = setLightPoint - currentLightPoint;
					  //Limit the values of the slope
					  if(lightslope>maxSlope) {
						  lightslope=maxSlope;
					  }
					  else if (slope[i]<minSlope) {
						  lightslope=minSlope;
					  }
					  //Values to send:
					  currentLightPoint = currentLightPoint + lightslope;

					  // Send light values
					  TIM3->CCR3 = currentLightPoint;
					  if(camera != 0) {
						  //Enable camera PWM channel and send the pwm value
						  TIM4->CCER |= TIM_CCER_CC1E;
						  TIM4->CCR1 = CamCCRtable[camera]; // Camera CCR value to send
						  TIM4->ARR = CamARRtable[camera]; // Camera CCR value to send
					  }
					  else {
						  // Disable the pwm so the camera doesn't take any pictures
						  TIM4->CCER &= ~TIM_CCER_CC1E;
					  }

//					  HAL_Delay(200);
					  error=0;	//Reset error

//					  error = error + lightslope;
					  //When all setPoints are achieved return to initial state:
					  if(error==0) {
						  HAL_TIM_Base_Start(&htim6);
						  state=0;
					  		}
					  else {
						  state=3;
					  }
			  }

		  break;
	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 53124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 56667;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7782;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 200;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 53124;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 53124;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
