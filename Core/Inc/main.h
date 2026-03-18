/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  Servo_Disengaged = 1,
  Servo_Engaged,
  Servo_Available
} ServoEnum;

typedef enum {
  EBS_Unavailable = 1,
  EBS_Armed,
  EBS_Triggered
} EBSEnum;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Interlock_Steering_Pin GPIO_PIN_1
#define Interlock_Steering_GPIO_Port GPIOA
#define ASRelay_State_Pin GPIO_PIN_2
#define ASRelay_State_GPIO_Port GPIOA
#define PrAnagSignal_Pin GPIO_PIN_4
#define PrAnagSignal_GPIO_Port GPIOA
#define TankPrServBrake_Pin GPIO_PIN_4
#define TankPrServBrake_GPIO_Port GPIOC
#define PrSenServBrake_Pin GPIO_PIN_5
#define PrSenServBrake_GPIO_Port GPIOC
#define ASRelay_In_Pin GPIO_PIN_12
#define ASRelay_In_GPIO_Port GPIOB
#define ASRelay_Out_Pin GPIO_PIN_13
#define ASRelay_Out_GPIO_Port GPIOB
#define TSMS_Out_NOT_Pin GPIO_PIN_14
#define TSMS_Out_NOT_GPIO_Port GPIOB
#define ASMS_Out_Pin GPIO_PIN_15
#define ASMS_Out_GPIO_Port GPIOB
#define UserLed_Pin GPIO_PIN_6
#define UserLed_GPIO_Port GPIOC
#define WatchdogPWM_Pin GPIO_PIN_9
#define WatchdogPWM_GPIO_Port GPIOC
#define Interlock_Valve1_Pin GPIO_PIN_15
#define Interlock_Valve1_GPIO_Port GPIOA
#define Interlock_valve2_Pin GPIO_PIN_10
#define Interlock_valve2_GPIO_Port GPIOC
#define Interlock_PV_Pin GPIO_PIN_11
#define Interlock_PV_GPIO_Port GPIOC
#define Valve2_GND_ST_Pin GPIO_PIN_5
#define Valve2_GND_ST_GPIO_Port GPIOB
#define Valve1_GND_ST_Pin GPIO_PIN_6
#define Valve1_GND_ST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//	ADC	//
#define ADC_BUFFER_SIZE 1

// AS MODES	//

#define No_Mission 0
#define Autonomous 1
#define Manual 2

//	AS STATES	//
#define AS_ManualDriving   0 //(0u)
#define AS_Off             1 //(1u)
#define AS_Ready           2 //(2u)
#define AS_Driving         3 //(3u)
#define AS_Finished        4 //(4u)
#define AS_Emergency       5 //(5u)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
