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
#define ASMS_Pin GPIO_PIN_13
#define ASMS_GPIO_Port GPIOC
#define AS_Relay_Out_Pin GPIO_PIN_1
#define AS_Relay_Out_GPIO_Port GPIOC
#define AS_Relay_In_Pin GPIO_PIN_2
#define AS_Relay_In_GPIO_Port GPIOC
#define Interlock_Steering_Pin GPIO_PIN_3
#define Interlock_Steering_GPIO_Port GPIOC
#define Interlock_Proportional_Pin GPIO_PIN_0
#define Interlock_Proportional_GPIO_Port GPIOA
#define Interlock_Valve2_Pin GPIO_PIN_1
#define Interlock_Valve2_GPIO_Port GPIOA
#define Interlock_Valve1_Pin GPIO_PIN_2
#define Interlock_Valve1_GPIO_Port GPIOA
#define Proportional_Signal_Pin GPIO_PIN_4
#define Proportional_Signal_GPIO_Port GPIOA
#define EBS_Valve1_GND_Pin GPIO_PIN_5
#define EBS_Valve1_GND_GPIO_Port GPIOA
#define EBS_Valve2_GND_Pin GPIO_PIN_6
#define EBS_Valve2_GND_GPIO_Port GPIOA
#define TankPressure1_Pin GPIO_PIN_5
#define TankPressure1_GPIO_Port GPIOC
#define TankPressure2_Pin GPIO_PIN_0
#define TankPressure2_GPIO_Port GPIOB
#define UserLed_Pin GPIO_PIN_12
#define UserLed_GPIO_Port GPIOB
#define AS_Relay_Signal_Pin GPIO_PIN_13
#define AS_Relay_Signal_GPIO_Port GPIOB
#define WatchdogPWM_Pin GPIO_PIN_9
#define WatchdogPWM_GPIO_Port GPIOC
#define TSMS_Pin GPIO_PIN_9
#define TSMS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//	ADC	//
#define ADC_BUFFER_SIZE 2

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
