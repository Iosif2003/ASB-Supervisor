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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "can_mcu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Tank_Pressure_min 3.5	//bar
#define Tank_Pressure_max 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

//	Debug Variables	//

volatile int Debug_State;
volatile int Initial_Check_Step;
volatile int APU_Transition;

//	Monitoring Variables	//

volatile bool Monitoring, Monitor_All;
int Manual_monitor_counter;
bool Manual_monitor_check, Watchdog_Check, Brake_Pressure_Check, Tank_Pressure_Check, Interlock_Valve1_Check, Interlock_Valve2_Check, APU_Communication_Check;
int Watchdog_Check_error, Brake_Pressure_Check_errors, Tank_Pressure_Check_errors, Interlock_Valve1_Check_errors, Interlock_Valve2_Check_errors;
int Brake_Pressure_Check_ms, Tank_Pressure_Check_ms, APU_Communication_Check_ms;
float Brake_Pressure;

// Servo / service brake variables used by CAN payloads.
int Servo_Command;
ServoEnum Servo_Status;
bool Service_Brake_Check, Servo_Interlock_Check;
//	CAN Variables	//

CAN_RxHeaderTypeDef msgHeaderRx;
uint32_t txMailbox;
uint8_t rxData[8] = {0};																								// Buffer for received data
uint8_t txData[8] = {0};
float Brake_Pressure;
bool  Can_Error;
volatile uint32_t Can_LastError;
volatile uint32_t Can_LastEsr;
volatile uint32_t Can_LastTsr;
volatile uint32_t Can_LastLec;
volatile uint32_t Can_LastTec;
volatile uint32_t Can_LastRec;
volatile uint32_t Can_LastBusOff;
volatile uint32_t Can_LastErrorPassive;
volatile uint32_t Can_LastErrorWarning;
volatile uint32_t Can_DebugState;
volatile uint32_t Can_DebugAttempts;
volatile uint32_t Can_DebugSuccess;
volatile uint32_t Can_DebugTimeouts;
volatile uint32_t Can_DebugAddTxErrors;
volatile uint32_t Can_DebugLastStdId;
volatile uint32_t Can_DebugLastTsr;
volatile uint32_t Can_DebugLastEsr;
volatile uint32_t Can_DebugLastFreeLevel;
volatile uint32_t Can_DebugTxCompleteCount;
volatile uint32_t Can_DebugTxAbortCount;
volatile uint32_t Can_DebugErrorCallbackCount;
volatile uint32_t Can_DebugManualCheckState;
struct can_mcu_apu_state_mission_t can_mcu_apu_state_mission;
struct can_mcu_vcu_servo_control_t can_mcu_vcu_servo_control;
struct can_mcu_vcu_bools_t can_mcu_vcu_bools;
struct can_mcu_dash_brake_t can_mcu_dash_brake;
struct can_mcu_asb_t canTxStruct_asb;
struct can_mcu_asb_datalogger_t canTxStruct_asb_datalogger;
float brake_pressure_front;
float brake_pressure_rear;

bool Initial_Checked;
bool Manual_Initial_Checked;

// GPIO Variables	//
			
bool ASMS_Out, TSMS_Out_NOT, ASRelay_In, ASRelay_Out, Interlock_Valve1, Interlock_Valve2, Interlock_PV, Interlock_Steering;		// Digital INPUTS from ASB
bool ASRelay_State, Valve1_GND, Valve2_GND;

// EBS-Tank Variables	//

float Tank_Pressure;
EBSEnum EBS_Status;
float ADC_Voltage_Value;
float Voltage_Change;
uint16_t adc_buffer[ADC_BUFFER_SIZE];	// 0- 4095 adc value
uint16_t ADC_Value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

static void CAN_UpdateDiagnostics(CAN_HandleTypeDef *hcan);
static HAL_StatusTypeDef CAN_SendStdMessage(uint32_t std_id, uint32_t dlc, const uint8_t *data, uint32_t *mailbox);
static void CAN_SendAsbStatusTask(void);
static void CAN_SendAsbDataloggerTask(void);
int Selected_Mission();
void Manual_Initial_Check();
void Manual_Monitoring();
bool Continuous_Monitoring();
void As_Initial_Check();
void Init_All();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef CAN_SendStdMessage(uint32_t std_id, uint32_t dlc, const uint8_t *data, uint32_t *mailbox)
{
	CAN_TxHeaderTypeDef txHeader = {0};
	uint32_t startTick = HAL_GetTick();

	Can_DebugAttempts++;
	Can_DebugLastStdId = std_id;
	Can_DebugLastTsr = hcan1.Instance->TSR;
	Can_DebugLastEsr = hcan1.Instance->ESR;
	Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	Can_DebugState = 3100;
	Debug_State = 2100;
	txHeader.StdId = std_id;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = dlc;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
	{
		Can_DebugLastTsr = hcan1.Instance->TSR;
		Can_DebugLastEsr = hcan1.Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		Can_DebugState = 3101;
		Debug_State = 2101;
		if ((HAL_GetTick() - startTick) > 5U)
		{
			Can_DebugTimeouts++;
			Can_DebugState = 3102;
			Debug_State = 2102;
			Can_Error = true;
			CAN_UpdateDiagnostics(&hcan1);
			return HAL_TIMEOUT;
		}
	}

	Can_DebugLastTsr = hcan1.Instance->TSR;
	Can_DebugLastEsr = hcan1.Instance->ESR;
	Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	Can_DebugState = 3103;
	Debug_State = 2103;
	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, (uint8_t *)data, mailbox) != HAL_OK)
	{
		Can_DebugAddTxErrors++;
		Can_DebugLastTsr = hcan1.Instance->TSR;
		Can_DebugLastEsr = hcan1.Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		Can_DebugState = 3104;
		Debug_State = 2104;
		Can_Error = true;
		CAN_UpdateDiagnostics(&hcan1);
		return HAL_ERROR;
	}

	Can_DebugSuccess++;
	Can_DebugLastTsr = hcan1.Instance->TSR;
	Can_DebugLastEsr = hcan1.Instance->ESR;
	Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	Can_DebugState = 3105;
	Debug_State = 2105;
	return HAL_OK;
}

static void CAN_SendAsbStatusTask(void)
{
	Can_DebugState = 3110;
	Debug_State = 2110;
	canTxStruct_asb.asms_state = ASMS_Out;
	canTxStruct_asb.tsms_out = !(TSMS_Out_NOT);
	canTxStruct_asb.initial_checked = Initial_Checked;
	canTxStruct_asb.service_brake_status = Servo_Status;
	canTxStruct_asb.ebs_status = EBS_Status;
	canTxStruct_asb.initial_check_step = Initial_Check_Step;
	canTxStruct_asb.monitor_tank_pressure = Tank_Pressure_Check;
	canTxStruct_asb.monitor_brake_pressure = Brake_Pressure_Check;
	canTxStruct_asb.monitor_servo_check = Service_Brake_Check;
	canTxStruct_asb.monitor_apu = APU_Communication_Check;
	canTxStruct_asb.ebs_tank_pressure = Tank_Pressure * 100;

	can_mcu_asb_pack(txData, &canTxStruct_asb, sizeof(txData));
	Can_DebugState = 3111;
	Debug_State = 2111;
	if (CAN_SendStdMessage(CAN_MCU_ASB_FRAME_ID, CAN_MCU_ASB_LENGTH, txData, &txMailbox) != HAL_OK)
	{
		Can_DebugState = 3112;
		Debug_State = 2112;
		Error_Handler();
	}
	Can_DebugState = 3113;
	Debug_State = 2113;
}

static void CAN_SendAsbDataloggerTask(void)
{
	Can_DebugState = 3120;
	Debug_State = 2120;
	canTxStruct_asb_datalogger.asms_out = ASMS_Out;
	canTxStruct_asb_datalogger.ebs_pneumatic_pressure = Tank_Pressure;
	canTxStruct_asb_datalogger.brake_pressure_rear = brake_pressure_rear;
	canTxStruct_asb_datalogger.brake_pressure_front = brake_pressure_front;
	canTxStruct_asb_datalogger.eb_sstate_unavailable = (EBS_Status == EBS_Unavailable);
	canTxStruct_asb_datalogger.eb_sstate_armed = (EBS_Status == EBS_Armed);
	canTxStruct_asb_datalogger.eb_sstate_activated = (EBS_Status == EBS_Triggered);
	canTxStruct_asb_datalogger.servicebrakestate_disengaged = (Servo_Status == Servo_Disengaged);
	canTxStruct_asb_datalogger.servicebrakestate_engaged = (Servo_Status == Servo_Engaged);
	canTxStruct_asb_datalogger.servicebrakestate_available = (Servo_Status == Servo_Available);
	canTxStruct_asb_datalogger.watchdog_ok = Watchdog_Check;
	canTxStruct_asb_datalogger.valve_interlock_ok = (Interlock_Valve1_Check && Interlock_Valve2_Check);
	canTxStruct_asb_datalogger.servo_interlock_ok = Servo_Interlock_Check;
	canTxStruct_asb_datalogger.as_state = can_mcu_apu_state_mission.as_state;
	
	can_mcu_asb_datalogger_pack(txData, &canTxStruct_asb_datalogger, sizeof(txData));
	Can_DebugState = 3121;
	Debug_State = 2121;
	if (CAN_SendStdMessage(CAN_MCU_ASB_DATALOGGER_FRAME_ID, CAN_MCU_ASB_DATALOGGER_LENGTH, txData, &txMailbox) != HAL_OK)
	{
		Can_DebugState = 3122;
		Debug_State = 2122;
		Error_Handler();
	}
	Can_DebugState = 3123;
	Debug_State = 2123;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Init_All();
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  		Debug_State = 214;  // DMA FAILS

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(Selected_Mission() == Autonomous){
		  switch(can_mcu_apu_state_mission.as_state){
		  case(AS_Off): 				// AS_OFF = 1 (CAN)
		  Debug_State = 1;
		  if ((htim3.Instance->CR1 & TIM_CR1_OPM) != 0){ 		 	//If One Pulse Mode is Enabled
			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); 	//Stop the timer (in case it expired we need to stop and start it again to work properly
			  htim3.Instance->CR1 &= ~TIM_CR1_OPM; 		// Disable OPM
			  __HAL_TIM_SET_COUNTER(&htim3, 0);    		//Reset counter to 0 (a bit extra)
			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	//Restart the timer with the correct setup
		          }
		  Monitoring = false;
		  Initial_Checked = false;		//This makes the APU AS_off until Initial Checks are complete
		  ASRelay_State = 0;			// Open the SDC
		  HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
		  if(ASMS_Out == 1)				//Necessary for the checks
		  		As_Initial_Check();
		  break;


		  case(AS_Driving):	 			// AS_Driving = 3 (CAN)
		  Debug_State = 4;
		  if(can_mcu_apu_state_mission.as_set_finished == CAN_MCU_APU_STATE_MISSION_AS_SET_FINISHED_SET__FINISHED__TRUE_CHOICE)
		  {
			  Debug_State = 5;
			  Valve1_GND = 0;
			  HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_RESET);
			  Valve2_GND = 0;
			  HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_RESET);
			  ASRelay_State = 0;		//Open SDC at finished state
			  HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);

			  htim3.Instance->CR1 &= ~TIM_CR1_OPM;		//This disables one pulse mode and returns to regular timer functionality watchdog timer
			  Monitoring = false;
		  	// APU notifies that its mission has finished and waits for ASB to do the appropriate actions in order for the car to be in actual Finished State
		  	// For Finished State APU needs: 1)EBS_Ativated, 2)Mission_Finished(Finish line detected + Standstill) after EBS we must open the SDC
		  	// So if set finished is true i have to engage ebs and the open SDC (AS_relay_Signal = 0)
		  	// If RES is opened -> Emergency or if EBS_Activated an Mission_Not_Finished -> Emergency
		  	}
		  break;


		  case(AS_Emergency):	 // AS_Emergency = 5 (CAN)	//We enter emergency if ebs is activated but vehicle is moving or mission is not finished
		  Debug_State = 7;
		  Monitoring = false;
		  Valve1_GND = 0;
		  HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_RESET);
		  Valve2_GND = 0;
		  HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_RESET);
		  ASRelay_State = 0;	//Open SDC
		  HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
		  
		  }
	  }
	  else if(Selected_Mission() == Manual)			//Manual Mission received
	  {
		  Debug_State = 8;
		  Initial_Check_Step = 0;
		  Monitoring = false;
		  //This ensures watchdog is Normal mode while we are running manual mode
		  if ((htim3.Instance->CR1 & TIM_CR1_OPM) != 0) 		//If One Pulse Mode is Enabled
	      {
			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); 				//Stop the timer (in case it expired we need to stop and start it again to work properly
			  htim3.Instance->CR1 &= ~TIM_CR1_OPM; 		 			// Disable OPM
			  __HAL_TIM_SET_COUNTER(&htim3, 0);    					//Reset counter to 0 (a bit extra)
			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);				//Restart the timer with the correct setup
	      }

		  if((Manual_Initial_Checked == false) && (ASMS_Out == 0) && (EBS_Status == EBS_Unavailable))	//Ensure that nothing goes wrong until SDC closes//ASB must check that no autonomous brake actuation is possible ASMS = 0 -> Servo Disengaged before closing the SDC (Manual mode)
			  Manual_Initial_Check();				//We enter this once because of Change in Manual initial checkes change if succesfull



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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 49999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 49999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 15;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 39999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 39;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim6.Init.Prescaler = 64934;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6159;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLed_GPIO_Port, UserLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Valve2_GND_ST_Pin|Valve1_GND_ST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Interlock_Steering_Pin Interlock_Valve1_Pin */
  GPIO_InitStruct.Pin = Interlock_Steering_Pin|Interlock_Valve1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ASRelay_State_Pin */
  GPIO_InitStruct.Pin = ASRelay_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ASRelay_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ASRelay_In_Pin ASRelay_Out_Pin TSMS_Out_NOT_Pin ASMS_Out_Pin */
  GPIO_InitStruct.Pin = ASRelay_In_Pin|ASRelay_Out_Pin|TSMS_Out_NOT_Pin|ASMS_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLed_Pin */
  GPIO_InitStruct.Pin = UserLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Interlock_valve2_Pin Interlock_PV_Pin */
  GPIO_InitStruct.Pin = Interlock_valve2_Pin|Interlock_PV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Valve2_GND_ST_Pin Valve1_GND_ST_Pin */
  GPIO_InitStruct.Pin = Valve2_GND_ST_Pin|Valve1_GND_ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void CAN_UpdateDiagnostics(CAN_HandleTypeDef *hcan)
{
	uint32_t esr = hcan->Instance->ESR;

	Can_LastError = HAL_CAN_GetError(hcan);
	Can_LastEsr = esr;
	Can_LastTsr = hcan->Instance->TSR;
	Can_LastLec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
	Can_LastTec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
	Can_LastRec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
	Can_LastBusOff = ((esr & CAN_ESR_BOFF) != 0U);
	Can_LastErrorPassive = ((esr & CAN_ESR_EPVF) != 0U);
	Can_LastErrorWarning = ((esr & CAN_ESR_EWGF) != 0U);
}

int Selected_Mission(){
	//Scenario where AS_Mission == 0 -> AS_Mission == No_Mission we should not enter autonomous nor manual mode so it is left out (return 0)
	// 1 is Autonomous || 2 is Manual || 0 is No Mission
	if ((can_mcu_vcu_bools.mode == 0) && (can_mcu_apu_state_mission.as_mission >= 1) && (can_mcu_apu_state_mission.as_mission <= 6))	//AS_mode == driverless, as_mission == 1 ==  Aceleration, as_mission == 6 == Inspection
			return 1;
		else if ((can_mcu_vcu_bools.mode == 1) || (can_mcu_apu_state_mission.as_mission == 7)) // as_mode == 1 == manual, as_mission == 7 == Manual driving
			return 2;
		else
			return 0;
}

void As_Initial_Check(){

	Debug_State = 2;

	// activate EBS to ensure brakes engaged
	Valve1_GND = 0;
	HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_RESET);
	Valve2_GND = 0;
	HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_RESET);

	//	WAIT FOR RES TO CLOSE	//

	Initial_Check_Step = 1;

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous)		//While we wait for the SDC before ASB to close if something changes like ASMS is turned off we break from the initial check procedure
			return;
	}while(ASRelay_In == 0);											//AS_relay_in is RES1_Out so if this is low check RES button it is probably pressed

	//	EBS ENGAGED FUNCTIONALITY	//

	Initial_Check_Step = 2;

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous || ASRelay_In != 1)
			return;
	}while(Tank_Pressure < Tank_Pressure_min || Brake_Pressure < 11);	//Check that we have enough pneumatic pressure to activate ebs and that brakes are engaged

	//	CLOSE AS_RELAY	// (SDC CLOSE)

	Initial_Check_Step = 3;
	ASRelay_State = 1;
	HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_SET);

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous || ASRelay_In != 1)
		return;
	}while(ASRelay_Out != 1);					//Wait for AS Relay to close so AS_Relay out is high

	//	DISABLE WATCHDOG	//

	Initial_Check_Step = 4;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);	//Stop the timer that produces Watchdog PWM -> SDC opens

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous || ASRelay_In != 1){
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	//Restart it so as relay is able to close on command if we re-enter the initial checks
			return;
		}
	}while(ASRelay_Out != 0);					//Wait for AS RELAY to open

	//	ENABLE WATCHDOG	//

	Initial_Check_Step = 5;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	// Start the timer that produces Watchdog PWM

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous || ASRelay_In != 1)
			return;
	}while(ASRelay_Out != 1);					//Wait for AS RELAY to close

	//	WAIT FOR CLOSED SDC	//

	Initial_Check_Step = 6;

	do{
		if(ASMS_Out != 1	||	Selected_Mission() != Autonomous || 	ASRelay_Out != 1)	//From now on we check AS_Relay_Out only inste of IN=0 -> OUT=0 as well so it is included
			return;
	}while(TSMS_Out_NOT != 0);					//TSMS is the last part of the SDC so TSMS high means we have closed SDC

	//	DE-ACTIVATE EBS	//

		Initial_Check_Step = 7;

		Valve1_GND = 1;
		HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_SET);
		Valve2_GND = 1;
		HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_SET);
		HAL_Delay(700);

		do{
			if(ASMS_Out != 1	||	Selected_Mission() != Autonomous ||	TSMS_Out_NOT != 0)	//TSMS_Out_NOT is after AS_Relay_Out on our vehicle SDC so checking that includes AS_Relay_Out and therefore in as well
				return;
		}while(Brake_Pressure > 3);

	//	ACTIVATE EBS RIGHT	//

		Initial_Check_Step = 8;

		Valve1_GND = 0;
		HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_RESET);
		HAL_Delay(700);

		do{
			if(ASMS_Out != 1	||	Selected_Mission() != 1 ||	TSMS_Out_NOT != 0)
				return;
		}while(Brake_Pressure < 5);			// IT DEEDS SETTING

	//	DE-ACTIVATE EBS	//

		Initial_Check_Step = 9;

		Valve1_GND = 1;
		HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port, Valve1_GND_ST_Pin, GPIO_PIN_SET);
		HAL_Delay(700);

		do{
			if(ASMS_Out != 1	||	Selected_Mission() != Autonomous ||	TSMS_Out_NOT != 0)
				return;
		}while(Brake_Pressure > 3);

	//	ACTIVATE EBS LEFT	//

		Initial_Check_Step = 10;

		Valve2_GND = 0;
		HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_RESET);
		HAL_Delay(800);

		do{
			if(ASMS_Out != 1	||	Selected_Mission() != 1 ||	TSMS_Out_NOT != 0)
				return;
		}while(Brake_Pressure < 5);		// IT DEEDS SETTING


		Initial_Checked = true; 			//Initial Check has been completed (almost) notify APU and wait for AS_Ready transition


		Valve2_GND = 1;
		HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port, Valve2_GND_ST_Pin, GPIO_PIN_SET);


	//	WAIT FOR APU AS READY	//	(5 seconds Max time)

		Initial_Check_Step = 11;

		HAL_TIM_Base_Start_IT(&htim6);		//interrupt for 5 seconds
		do{
			if(APU_Transition == false){
				Initial_Checked = false;
				return;
			}
		}while(can_mcu_apu_state_mission.as_state != AS_Ready);

	//	AS READY RECEIVED	//

		Initial_Check_Step = 12;
		Monitoring = true;

	//	WATCHDOG OPM

		__HAL_TIM_SET_COUNTER(&htim3, 0);	//This ensures that the timer is at 0ms of period because if it was close to the end of its cycle it might end before we reset it via the monitoring function and it would be off
		htim3.Instance->CR1 |= TIM_CR1_OPM;	//Enables the One Pulse Mode bit on the watchdog PWM timer
		Initial_Check_Step = 13;			//AS_Initial_Check is completed succesfully if Initial_Check_Step = 13
	}


bool Continuous_Monitoring()	// runs every 10 ms
{

	__HAL_TIM_SET_COUNTER(&htim3, 0);	//This resets the timer to 0 ensuring that PWM generation is continued

	//	WATCHDOG CHECK	//

	if(ASRelay_State == 1 && ASRelay_In == 1 && ASRelay_Out == 0)	//If SDC reaches AS relay, and AS_Relay_Signal==1 but as relay is opened only watchdog	(or hardware malfunction) are the possible causes
	{
		Watchdog_Check_error++;
		if(Watchdog_Check_error > 50)
			Watchdog_Check = false;	//This is used in Datalogger can there is no point entering this in the monitor_all because if this fails sdc is opened already and ebs is triggered so we should go in emergency and there monitoring should go off so this no longer repeats
	}
	else
	{
		Watchdog_Check_error = 0;
		Watchdog_Check = true;	//Consider removing this (mistake in asf) it is extra, no need to watchdog check
	}
	//	BRAKE PRESSURE CHECKS	//

	if(Brake_Pressure < 150 && Brake_Pressure >= 0)	//Brake_Pressure at normal value range
	{
		Brake_Pressure_Check_ms += 10;				//Increases the time from last message (this goes to 0 when can message is received see the HAL_CAN_RxFifo0MsgPendingCallback()
		Brake_Pressure_Check_errors = 0;			//Makes the consecutive errors to 0
		Brake_Pressure_Check = true;					//Succesfull Brake Pressure Check
	}
	else
	{
		Brake_Pressure_Check_ms += 10;
		Brake_Pressure_Check_errors++;				//error counter + 1
	}

	if(Brake_Pressure_Check_errors >= 10 || Brake_Pressure_Check_ms > 400)	//10 consecutive errors needed in order to fail the check
			Brake_Pressure_Check = false;				//Failed Brake Pressure Check
	//	TANK PRESSURE CHECKS	//

	if(Tank_Pressure > Tank_Pressure_min && Tank_Pressure < Tank_Pressure_max)	//Tank_Pressure in acceptable limits
	{
		Tank_Pressure_Check_ms += 10;					//Increases time from last Value received, this becomes 0 when a new value is received look HAL_ADC_ConvCpltCallback
		Tank_Pressure_Check_errors = 0;				//Makes the consecutive errors to 0
		Tank_Pressure_Check = true;						//Succesfull Tank Pressure Check
	}
	else
	{
		Tank_Pressure_Check_ms += 10;
		Tank_Pressure_Check_errors++;					//error counter + 1
	}

	if(Tank_Pressure_Check_errors >= 10 || Tank_Pressure_Check_ms > 1000)	//10 consecutive errors needed in order to fail the check
			Tank_Pressure_Check = false;				//Failed Tank Pressure Check

	//	VALVE INTERLOCK 1 CHECK	//

	if(Interlock_Valve1 == 1)	//Valve_Interlock == 1 -> Valve connector connected
	{
		Interlock_Valve1_Check_errors = 0;				//Set consecutive errors to 0
		Interlock_Valve1_Check = true;						//Succesfull Valve Interlock Check
	}
	else
	{
		Interlock_Valve1_Check_errors++;					//error counter + 1
		if(Interlock_Valve1_Check_errors >= 10)	//10 consecutive errors needed in order to fail the check
			Interlock_Valve1_Check = false;				//Failed Valve Interlock Check
	}

	//	VALVE INTERLOCK 2 CHECK	//

	if(Interlock_Valve2 == 1)	//Servo connector is connected
	{
		Interlock_Valve2_Check_errors = 0;				//Set consecutive errors to 0
		Interlock_Valve2_Check = true;						//Succesfull Valve Interlock Check
	}
	else
	{
		Interlock_Valve2_Check_errors++;					//error counter + 1
			if(Interlock_Valve2_Check_errors >= 10)	//10 consecutive errors needed in order to fail the check
				Interlock_Valve2_Check = false;				//Failed Valve Interlock Check
	}
			//	APU COMMUNICATION CHECK	//

	if(APU_Communication_Check_ms <= 500)	//Time since last APU message <500ms ago
	{																			//This timer goes to 0 when a new message arrives
		APU_Communication_Check_ms += 10;		//Add  4321543gd`	10ms to time since last message
		APU_Communication_Check = true;
	}
	else
		APU_Communication_Check = false;//Time > 500ms -> Fail

	//Monitor_All = Brake_Pressure_Check && Tank_Pressure_Check && Interlock_Valve1_Check && Interlock_Valve2_Check && APU_Communication_Check;

	return (1);
}


void Manual_Initial_Check()
{
	Debug_State = 9;
	Can_DebugManualCheckState = 1;
	Initial_Check_Step = 100;
	ASRelay_State = 1;						//Close AS_Relay
	HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_SET);

	if((ASMS_Out == 1) || (Selected_Mission() != Manual) ||	(EBS_Status != EBS_Unavailable))//Ensure that nothing goes wrong until SDC closes
	{
		Can_DebugManualCheckState = 2;
		ASRelay_State = 0;				//If an error case appeears open the SDC
		HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
		return;
	}

	if(TSMS_Out_NOT == 1)				//Indicates SDC State
	{
		Can_DebugManualCheckState = 3;
		return;
	}

	Initial_Check_Step = 101;
	Can_DebugManualCheckState = 4;
	Manual_Initial_Checked = true; 			//This ensures that this function is entered only once
	Initial_Checked = true;
}

void Manual_Monitoring()
{
	if(Manual_Initial_Checked == true)
	{
		if(ASMS_Out == 0 && EBS_Status == EBS_Unavailable)
		{
			Manual_monitor_check = true;
			Manual_monitor_counter = 0;
				}
		else
		{
			Manual_monitor_counter++;
			if(Manual_monitor_counter > 20) //2 seconds of persisting continuous error
			{
				Manual_monitor_check = false;
				ASRelay_State = 0;
				HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
				Initial_Checked = false;
				Manual_Initial_Checked = false;
				Debug_State = 10;
					}
				}
			}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
		Tank_Pressure_Check_ms = 0;	//Monitoring functiion, each time a new value is received make it 0
        // ADC conversion done
        ADC_Value = adc_buffer[0];	//Storing the raw value of adc in ADC_Value
        // Use or log the value
		ADC_Voltage_Value = ((float)ADC_Value / 4095.0) * 3.3;	//Converts the adc raw value to voltage value
		Voltage_Change = ADC_Voltage_Value * 1.5;				//Converts the voltage value from 0-3.3V -> 0-5 V as the pressure sensor outputs
		Tank_Pressure = (2.5 * Voltage_Change) - 2.5;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(UserLed_GPIO_Port, UserLed_Pin);
	}

	if(htim->Instance == TIM4) //10ms
	{
		//	DIGITAL INPUTS	//
		ASMS_Out = HAL_GPIO_ReadPin(ASMS_Out_GPIO_Port, ASMS_Out_Pin);
		TSMS_Out_NOT = HAL_GPIO_ReadPin(TSMS_Out_NOT_GPIO_Port, TSMS_Out_NOT_Pin);
		ASRelay_In = HAL_GPIO_ReadPin(ASRelay_In_GPIO_Port,ASRelay_In_Pin);
		ASRelay_Out = HAL_GPIO_ReadPin(ASRelay_Out_GPIO_Port, ASRelay_Out_Pin);
		Interlock_PV = HAL_GPIO_ReadPin(Interlock_PV_GPIO_Port, Interlock_PV_Pin);
		Interlock_Steering = HAL_GPIO_ReadPin(Interlock_Steering_GPIO_Port, Interlock_Steering_Pin);
		Interlock_Valve1 = HAL_GPIO_ReadPin(Interlock_Valve1_GPIO_Port, Interlock_Valve1_Pin);
		Interlock_Valve2 = HAL_GPIO_ReadPin(Interlock_valve2_GPIO_Port, Interlock_valve2_Pin);

		//	EBS STATE	//
		if(Tank_Pressure < Tank_Pressure_min)
			EBS_Status = EBS_Unavailable;						//We do not have the sufficient tank pressure to consider ebs triggered when activated (it may be activated but brake pressure will me minimal)
		else if(TSMS_Out_NOT == 1 || Valve1_GND == 0 || Valve2_GND == 0)	//No SDC or EBS Activated -> EBS Activated + Enough Tank Presure -> EBS Triggered (What about mechanical Valve)
			EBS_Status = EBS_Triggered;
		else
			EBS_Status = EBS_Armed;								//Tank Pressure sufficient + SDC + EBS Not Activated -> Armed = Ready to be commanded and functional

		//	MONITORING	//
		if(Monitoring == true)
		{
			if(Selected_Mission() == Autonomous)
			{
				if(Continuous_Monitoring() == false || ASMS_Out == 0)
				{
					Monitoring = false; 			//This ensures monitoring function wont run again after failing
					Debug_State = 1170;
					Valve1_GND = 0;					//If an error occurs open SDC -> EBS Activated -> AS_Emergency
					HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_RESET);
					Valve2_GND = 0;
					HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_RESET);
					ASRelay_State = 0;				//SDC is open so EBS is triggered on matter what, so we match the physical state with the software side
					HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
				}
			}
		}
	}

	if(htim->Instance == TIM5)	//20 ms period
	{
		Can_DebugState = 3130;
		Debug_State = 2130;
		CAN_SendAsbStatusTask();
	}

	//	APU TRANSITION	//

	if (htim->Instance == TIM6)	//(5 sec) passed until Initial_Checked == true has been sent to can
	{
		if(can_mcu_apu_state_mission.as_state != AS_Ready)
			APU_Transition = false; //this means that 5 seconds passed and as did not transition to as ready
	}

	if(htim->Instance == TIM7)	//100ms
	{
		Can_DebugState = 3140;
		Debug_State = 2140;
		CAN_SendAsbDataloggerTask();

		if(Manual_Initial_Checked == true)
			Manual_Monitoring();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	//runs automaticly each time a can message is received
{
	if(hcan->Instance == CAN1)  	// CAN1 Interrupt
	{
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msgHeaderRx, rxData) != HAL_OK)
		{
			Can_Error = true;
			Error_Handler();
		}
		// Read received message

			if (msgHeaderRx.StdId == CAN_MCU_DASH_BRAKE_FRAME_ID ){
				Brake_Pressure_Check_ms = 0;
				can_mcu_dash_brake_unpack(&can_mcu_dash_brake, rxData, msgHeaderRx.DLC);

				brake_pressure_front = can_mcu_dash_brake_brake_pressure_front_decode(can_mcu_dash_brake.brake_pressure_front);
				brake_pressure_rear = can_mcu_dash_brake_brake_pressure_rear_decode(can_mcu_dash_brake.brake_pressure_rear);
				Brake_Pressure = brake_pressure_front;
			}
			else if (msgHeaderRx.StdId == CAN_MCU_APU_STATE_MISSION_FRAME_ID){
				APU_Communication_Check_ms = 0;
				can_mcu_apu_state_mission_unpack(&can_mcu_apu_state_mission, rxData, msgHeaderRx.DLC);
			}
			else if (msgHeaderRx.StdId == CAN_MCU_VCU_SERVO_CONTROL_FRAME_ID) {
				can_mcu_vcu_servo_control_unpack(&can_mcu_vcu_servo_control, rxData, msgHeaderRx.DLC);
				Servo_Command = can_mcu_vcu_servo_control.servo_control;
			}
			else if (msgHeaderRx.StdId == CAN_MCU_VCU_BOOLS_FRAME_ID)
				can_mcu_vcu_bools_unpack(&can_mcu_vcu_bools, rxData, msgHeaderRx.DLC);	//We read AS vcu_bools.mode 0 => DV, 1=> Manual
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugErrorCallbackCount++;
		Can_DebugState = 3300;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
		Can_Error = true;
		CAN_UpdateDiagnostics(hcan);
		Debug_State = 304;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxCompleteCount++;
		Can_DebugState = 3200;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxCompleteCount++;
		Can_DebugState = 3201;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxCompleteCount++;
		Can_DebugState = 3202;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxAbortCount++;
		Can_DebugState = 3210;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxAbortCount++;
		Can_DebugState = 3211;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		Can_DebugTxAbortCount++;
		Can_DebugState = 3212;
		Can_DebugLastTsr = hcan->Instance->TSR;
		Can_DebugLastEsr = hcan->Instance->ESR;
		Can_DebugLastFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}
}

void Init_All(){

	//	VARIABLES	//

	Debug_State = 0;					//No basic part of code has been entered
	Initial_Checked = false;
	ASMS_Out = 0;
	TSMS_Out_NOT = 1;
	ASRelay_In = 0;
	ASRelay_Out = 0;
	Interlock_Valve1 = 0;
	Interlock_Valve2 = 0;
	Tank_Pressure = 0;
	Brake_Pressure = 0;
	Manual_Initial_Checked = false;
	Manual_monitor_check = true;
	Manual_monitor_counter = 0;
	Watchdog_Check_error = 0;
	ADC_Voltage_Value = 0;
	ADC_Value = 0;
	brake_pressure_front = 0;
	brake_pressure_rear = 0;
	Initial_Check_Step = 0;
	Servo_Status = Servo_Disengaged;
	Servo_Command = 1;
	Can_Error = false;
	Can_LastError = HAL_CAN_ERROR_NONE;
	Can_LastEsr = 0;
	Can_LastTsr = 0;
	Can_LastLec = 0;
	Can_LastTec = 0;
	Can_LastRec = 0;
	Can_LastBusOff = 0;
	Can_LastErrorPassive = 0;
	Can_LastErrorWarning = 0;
	Can_DebugState = 0;
	Can_DebugAttempts = 0;
	Can_DebugSuccess = 0;
	Can_DebugTimeouts = 0;
	Can_DebugAddTxErrors = 0;
	Can_DebugLastStdId = 0;
	Can_DebugLastTsr = 0;
	Can_DebugLastEsr = 0;
	Can_DebugLastFreeLevel = 0;
	Can_DebugTxCompleteCount = 0;
	Can_DebugTxAbortCount = 0;
	Can_DebugErrorCallbackCount = 0;
	Can_DebugManualCheckState = 0;
	Brake_Pressure = 0;
	APU_Transition = true;

	HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);
	Valve1_GND=0;
	HAL_GPIO_WritePin(Valve1_GND_ST_GPIO_Port,Valve1_GND_ST_Pin, GPIO_PIN_RESET);
	Valve2_GND = 0;
	HAL_GPIO_WritePin(Valve2_GND_ST_GPIO_Port,Valve2_GND_ST_Pin, GPIO_PIN_RESET);
	ASRelay_State = 0;
	HAL_GPIO_WritePin(ASRelay_State_GPIO_Port, ASRelay_State_Pin, GPIO_PIN_RESET);

	//	MONITORING	//

	Monitor_All = true;					//Initialize all checks to true (less fail points)
	Monitoring = false;					//Start without monitoring
	Watchdog_Check = true;
	Brake_Pressure_Check_ms = 0;
	Brake_Pressure_Check = true;
	Brake_Pressure_Check_errors = 0;
	Tank_Pressure_Check_errors = 0;
	Tank_Pressure_Check_ms = 0;
	Tank_Pressure_Check = true;
	Interlock_Valve1_Check = true;
	Interlock_Valve1_Check_errors = 0;
	Interlock_Valve2_Check = true;
	Interlock_Valve2_Check_errors = 0;
	Service_Brake_Check = true;
	Servo_Interlock_Check = true;

	// ----------- INIT CAN PERIPHERAL --------------- //
		// CAN filter settings
		CAN_FilterTypeDef filterConfig;

		// Common filter configuration
		filterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // List mode
		filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		filterConfig.FilterActivation = ENABLE;
		filterConfig.SlaveStartFilterBank = 27;  // CAN1 uses 0�26, CAN2 starts at 27

		// List of allowed standard IDs
		uint16_t acceptedIDs[] = {
				0x00A,  // APU_STATE_MISSION
				0x065,  // DASH_BRAKE
				0x320,  // VCU_BOOLS
				0x324,  // VCU_SERVO_CONTROL
				//0x2ff,
		};

		int idCount = sizeof(acceptedIDs) / sizeof(acceptedIDs[0]);
		int bankIndex = 0;

		// Load 2 IDs per filter bank
		for (int i = 0; i < idCount; i += 2)
		{
				filterConfig.FilterBank = bankIndex++;

				// First ID (Standard ID -> shift left 5)
				filterConfig.FilterIdHigh     = acceptedIDs[i] << 5;
				filterConfig.FilterIdLow      = 0;

				// Second ID
				if (i + 1 < idCount) {
						filterConfig.FilterMaskIdHigh = acceptedIDs[i + 1] << 5;
						filterConfig.FilterMaskIdLow  = 0;
				} else {
						filterConfig.FilterMaskIdHigh = 0;
						filterConfig.FilterMaskIdLow  = 0;
				}

				if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
				{
					Can_Error = true;
					Error_Handler();
				}
		}

		if (HAL_CAN_Start(&hcan1) != HAL_OK)
		{
			Can_Error = true;
			Error_Handler();
		}
		if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
		{
			Can_Error = true;
			Error_Handler();
		}

		can_mcu_apu_state_mission.as_mission = No_Mission;
		can_mcu_apu_state_mission.as_state = AS_Off;
		// Clear buffers to avoid leftover values affecting CAN messages
		for(int i=0; i<8; i++)
			txData[i] = 0;
		for(int i=0; i<8; i++)
			rxData[i] = 0;

	//	TIMERS	//

	HAL_TIM_Base_Start_IT(&htim2);	//ADC trigger / debug heartbeat
	HAL_TIM_Base_Start_IT(&htim4);	//Monitor exct		(10ms) pr=49999 c=15
	HAL_TIM_Base_Start_IT(&htim5);	//CAN	ASB			(20ms) pr=39999 c=30
	HAL_TIM_Base_Start_IT(&htim7);	//CAN	ASB Datalogger	(100ms)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

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
