/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _CW 0
#define _CCW 1
#define _STOP 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t TxMailbox;

uint8_t flag = 2;
uint8_t f;
uint8_t a;
uint8_t state = 0;
uint8_t control_data[2];
uint8_t side;
float t_speed = 0.0;
float pred_t_speed = 1.0;
uint8_t data[2];

uint32_t EncVal;
uint32_t i=1;
uint32_t k;
uint32_t count;
uint32_t count1;
uint32_t input_capture;
uint32_t pwm;

enum States
{
	State_Start,
	State_Left,
	State_Right,
	State_Zero
};

_Bool side_change;

//float t_speed = 1.000;
uint8_t rotate_speed_can;
uint8_t rotate_speed_side;
uint8_t driver_tx_data[2];
float rotate_time;
float rotate_speed;
float target_speed_r = 0.0;
float target_speed_l = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {
		//__HAL_TIM_SetCounter(&htim1, 0);
		TIM3->CNT = 0;
//		TIM1->CCR1 = 0;
	}
}
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);

  pHeader.DLC = 2;
  pHeader.IDE = CAN_ID_STD;
  pHeader.RTR = CAN_RTR_DATA;
  pHeader.StdId = 0x3F;

  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0xF<<5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_Delay(50);
  TIM2->CCR4 = 220;

  void move_right(float r_speed)
  {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
  target_speed_r = r_speed;
  rotate_speed_side = 1;
  if (target_speed_r >= 0.1) {
	  input_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
	  rotate_time = (float)(input_capture*520)/1000000;
	  rotate_speed = (1/rotate_time);
	  rotate_speed_can = (uint8_t)(rotate_speed*60);
	  if (rotate_time == 0) {
		  rotate_speed = 0;
	  }

	  pwm = TIM2->CCR4;

	  if ((rotate_speed<(target_speed_r*0.999))&&(rotate_speed!=0)) {
		  if (TIM2->CCR4 > 750) {
			  TIM2->CCR4 = 750;
	  	  }
	  	  TIM2->CCR4 += 1;
	  	  driver_tx_data[0] = rotate_speed_can;
	  	  driver_tx_data[1] = rotate_speed_side;
	  	  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  	  HAL_Delay(3);
	  }
	  else if ((rotate_speed>(target_speed_r*1.001))) {
	  	  TIM2->CCR4 -= 1;
	  	  driver_tx_data[0] = rotate_speed_can;
	  	  driver_tx_data[1] = rotate_speed_side;
	  	  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  	  HAL_Delay(3);
	  }
  }
//  else {
//	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
//	  rotate_speed_can = 0;
//	  //TIM2->CCR4 = 0;
//  }
  }

  void move_left(float l_speed)
  {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
  target_speed_l = l_speed;
  rotate_speed_side = 2;
  if (target_speed_l >= 0.1) {
	  input_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
	  rotate_time = (float)(input_capture*520)/1000000;
	  rotate_speed = (1/rotate_time);
	  rotate_speed_can = (uint8_t)(rotate_speed*60);
	  if (rotate_time == 0) {
		  rotate_speed = 0;
	  }

	  pwm = TIM2->CCR4;

	  if ((rotate_speed<(target_speed_l*0.999f))&&(rotate_speed!=0)) {
		  if (TIM2->CCR4 > 750) {
			  TIM2->CCR4 = 750;
	  	  }
	  	  TIM2->CCR4 += 1;
	  	  driver_tx_data[0] = rotate_speed_can;
	  	  driver_tx_data[1] = rotate_speed_side;
	  	  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  	  HAL_Delay(3);
	  }
	  else if ((rotate_speed>(target_speed_l*1.001f))) {
		  TIM2->CCR4 -= 1;
		  driver_tx_data[0] = rotate_speed_can;
		  driver_tx_data[1] = rotate_speed_side;
		  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  	  HAL_Delay(3);
	  }
  }
//  else {
//	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
//	  rotate_speed_can = 0;
//	  //TIM2->CCR4 = 0;
//  }
  }

  void reduce_speed_right() {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	  target_speed_r = 0.1;
	  rotate_speed_side = 1;
	  if (target_speed_r >= 0) {
	  	input_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
	  	rotate_time = (float)(input_capture*520)/1000000;
	  	rotate_speed = (1/rotate_time);
	  	rotate_speed_can = (uint8_t)(rotate_speed*60);
	  	if (rotate_time == 0) {
	  		rotate_speed = 0;
	  	}
	  	pwm = TIM2->CCR4;
	  	if ((rotate_speed<(target_speed_r*0.999f))&&(rotate_speed!=0)) {
	  		  if (TIM2->CCR4 > 750) {
	  			  TIM2->CCR4 = 750;
	  	  	  }
	  		  if (TIM2->CCR4 < 1) {
	  			  TIM2->CCR4 = 1;
	  		  }
	  	  	  TIM2->CCR4 += 1;
	  	  	  driver_tx_data[0] = rotate_speed_can;
	  	  	  driver_tx_data[1] = rotate_speed_side;
	  	  	  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  	  	  HAL_Delay(3);
	  	  }
	  	  else if ((rotate_speed>(target_speed_r*1.001f))) {
	  		  if (TIM2->CCR4 > 750) {
	  			  TIM2->CCR4 = 750;
	  		  }
	  		  if (TIM2->CCR4 < 1) {
	  			  TIM2->CCR4 = 1;
	  		  }
	  		  TIM2->CCR4 -= 1;
	  		  driver_tx_data[0] = rotate_speed_can;
	  		  driver_tx_data[1] = rotate_speed_side;
	  		  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
	  		  HAL_Delay(3);
	  	  	  //HAL_Delay(4);
	  	  }
	    }
//	    else {
//	  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//	  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
//	  	  rotate_speed_can = 0;
//	  	  //TIM2->CCR4 = 0;
//	    }
  }

  void reduce_speed_left() {
  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
  	  target_speed_l = 0.1;
  	  rotate_speed_side = 2;
  	  if (target_speed_l >= 0) {
  	  	input_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
  	  	rotate_time = (float)(input_capture*520)/1000000;
  	  	rotate_speed = (1/rotate_time);
  	  	rotate_speed_can = (uint8_t)(rotate_speed*60);
  	  	if (rotate_time == 0) {
  	  		rotate_speed = 0;
  	  	}
  	  	pwm = TIM2->CCR4;
  	  	if ((rotate_speed<(target_speed_l*0.999f))&&(rotate_speed!=0)) {
  	  	 if (TIM2->CCR4 > 750) {
  	  			  TIM2->CCR4 = 750;
  	  	  	  }
  	  	  	  TIM2->CCR4 += 1;
  	  	  	  driver_tx_data[0] = rotate_speed_can;
  	  	  	  driver_tx_data[1] = rotate_speed_side;
  	  	  	  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
  	  	  	  HAL_Delay(3);
  	  	  }
  	  	  else if ((rotate_speed>(target_speed_l*1.001f))) {
  	  		  TIM2->CCR4 -= 1;
  	  		  driver_tx_data[0] = rotate_speed_can;
  	  		  driver_tx_data[1] = rotate_speed_side;
  	  		  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);
  	  		  HAL_Delay(3);
  	  	  	  //HAL_Delay(4);
  	  	  }
  	    }
//  	    else {
//  	  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//  	  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
//  	  	  rotate_speed_can = 0;
//  	  	  //TIM2->CCR4 = 0;
//  	    }
    }


  void stop_movement(void)
  {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	  TIM2->CCR4 = 50;
	  rotate_speed_can = 0;
	  rotate_speed_side = 0;
  }

//  void stop_movement();
//  {
//
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  switch(state)
//	  {
//	  	  case State_Start:
//	  	  {
//	  		  if (t_speed > 0) {
////	  			while (rotate_speed >= 0.06) {
////	  				reduce_speed_right();
////	  			}
////	  			stop_movement();
//	  			state = State_Right;
//	  			break;
//	  		  }
//	  		  else if (t_speed < 0) {
////	  			while (rotate_speed >= 0.06) {
////	  				reduce_speed_left();
////	  			}
////	  			stop_movement();
//	  			state = State_Left;
//	  			break;
//	  		  }
//	  		  else if (t_speed == 0) {
//
//	  			state = State_Zero;
//	  			break;
//	  		  }
//	  	  }
//	  	  case State_Right:
//	  	  {
//	  		  if (t_speed > 0) {
//	  			  move_right(t_speed);
//	  		  }
//	  		  else if (t_speed < 0) {
//	  			  while (rotate_speed >= 0.06) {
//	  				  reduce_speed_right();
//	  			  }
//	  			  stop_movement();
//	  			  state = State_Start;
//	  			  break;
//	  		  }
//	  		  else if (t_speed == 0) {
//	  			  state = State_Zero;
//	  			  break;
//	  		  }
//	  	  }
//	  	  case State_Left:
//	  	  {
//	  		  if (t_speed < 0) {
//	  		  	 move_left(-t_speed);
//	  		  }
//	  		  else if (t_speed > 0) {
//	  			 while (rotate_speed >= 0.06) {
//	  				 reduce_speed_left();
//	  			 }
//	  			 stop_movement();
//	  		  	 state = State_Start;
//	  		  	 break;
//	  		  }
//	  		  else if (t_speed == 0) {
//	  			 state = State_Zero;
//	  			 break;
//	  		  }
//	  	  }
//	  	  case State_Zero:
//	  	  {
//	  		  stop_movement();
//	  		  state = State_Start;
//	  		  break;
//	  	  }
//	  }
	  if ((t_speed > 0)&&(flag == State_Right)) {
		  move_right(t_speed);
		  flag = State_Right;
	  }
	  else if ((t_speed < 0)&&(flag == State_Right)) {
		  //move_left(-t_speed);
		  while (rotate_speed >= 0.1) {
			  reduce_speed_right();
		  }
		  stop_movement();
		  flag = State_Left;
		  //move_left(-t_speed);
	  }
	  else if ((t_speed < 0)&&(flag == State_Left)) {
		  move_left(-t_speed);
		  flag = State_Left;
	  }
	  else if ((t_speed > 0)&&(flag == State_Left)) {
		  while (rotate_speed >= 0.1) {
			  reduce_speed_left();
		  }
		  stop_movement();
		  flag = State_Right;
	  }
	  else if (t_speed == 0) {
		  stop_movement();
	  }
	  else {
	  	 stop_movement();
	  }
	  if ((HAL_GetTick() % 20) == 0) {

		  driver_tx_data[0] = rotate_speed_can;
		  driver_tx_data[1] = rotate_speed_side;
		  HAL_CAN_AddTxMessage(&hcan1, &pHeader, &driver_tx_data, &TxMailbox);

	  }
	  pred_t_speed = t_speed;

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 
                           PB14 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
