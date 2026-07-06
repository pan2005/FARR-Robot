/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "bsp_usart.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "chassis_control_task.h"
#include "com_with_gimbal.h"
#include "shoot_task.h"
#include "rdk_comm_task.h"
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
/* USER CODE BEGIN Variables */
extern const RC_ctrl_t * local_rc_ctrl;
/* USER CODE END Variables */
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t Com_Handle;
const osThreadAttr_t uart_com_attributes = {
  .name = "test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh - 1,
};

osThreadId_t Chassis_Handle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 256 * 12,
  .priority = (osPriority_t) osPriorityHigh ,
};

osThreadId_t Shoot_Handle;
const osThreadAttr_t shoot_task_attributes = {
  .name = "chassis_task",
  .stack_size = 256 * 2,
  .priority = (osPriority_t) osPriorityHigh ,
};

osThreadId_t RdkCom_Handle;
const osThreadAttr_t rdk_com_task_attributes = {
  .name = "rdk_com_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh - 1,
};


/* USER CODE END FunctionPrototypes */

void test_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of test */
  testHandle = osThreadNew(test_task, NULL, &test_attributes);
  Chassis_Handle = osThreadNew(chassis_control_task,NULL,&chassis_task_attributes);
  RdkCom_Handle = osThreadNew(rdk_comm_task,NULL,&rdk_com_task_attributes);
  //Shoot_Handle = osThreadNew(shoot_task,NULL,&shoot_task_attributes);
 // Com_Handle = osThreadNew(communication_with_gimbal_task,NULL,&uart_com_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void chassis_test_task(void * argument) {
  int16_t velocity_forword = 0;
  int16_t velocity_turn = 0;
  int16_t velocity_14 = 0;
  int16_t velocity_23 = 0;

  while (1) {
    if (switch_is_down(local_rc_ctrl->rc.s[1])) {
      velocity_forword = local_rc_ctrl->rc.ch[3] * 40;
      velocity_turn = local_rc_ctrl->rc.ch[2] * 5;
      CAN_cmd_chassis(velocity_forword + velocity_turn, -(velocity_forword - velocity_turn), velocity_forword + velocity_turn, -(velocity_forword - velocity_turn));

    }
    else if (switch_is_up(local_rc_ctrl->rc.s[1])) {

      velocity_14 = -local_rc_ctrl->rc.ch[2] * 30 + local_rc_ctrl->rc.ch[3] * 30;
      velocity_23 = local_rc_ctrl->rc.ch[2] * 30 + local_rc_ctrl->rc.ch[3] * 30;
      CAN_cmd_chassis(velocity_14,-velocity_23,velocity_23,-velocity_14);




    }
    osDelay(2);

  }


}
/* USER CODE END Application */

