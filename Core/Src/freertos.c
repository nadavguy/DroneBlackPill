/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMUAgent.h"
#include "AHRS.h"

#include "bme68x.h"

#include "bmm150.h"

#include "bmi088.h"
#include "bmi088a.h"
#include "bmi088g.h"
#include "bmi08x.h"
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
osStatus_t imuSemStatus;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMUSem */
osSemaphoreId_t IMUSemHandle;
const osSemaphoreAttr_t IMUSem_attributes = {
  .name = "IMUSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartIMUTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of IMUSem */
  IMUSemHandle = osSemaphoreNew(1, 1, &IMUSem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
	/* USER CODE BEGIN StartIMUTask */
	uint8_t status = 0;

	initBME680();
	initBMM150();
	initBMI088();
	/* Infinite loop */
	for(;;)
	{
		imuSemStatus = osSemaphoreAcquire(IMUSemHandle, 10);
		if (imuSemStatus == osOK)
		{
			bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
			rawARHSData.pressure = (data[0].pressure + data[1].pressure + data[2].pressure) / 3.0;
			rawARHSData.temperature = (data[0].temperature + data[1].temperature + data[2].temperature) / 3.0;

			get_data(&bmm);

			enable_bmi08x_interrupt();
			bmi08a_get_data_int_status(&status, &bmi);
			if (status & BMI08X_ACCEL_DATA_READY_INT)
			{
				rslt = bmi08a_get_data(&bmi08x_accel, &bmi);

				/* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
				rawARHSData.AccData[0] = lsb_to_mps2(bmi08x_accel.x, 24, 16);
				rawARHSData.AccData[1] = lsb_to_mps2(bmi08x_accel.y, 24, 16);
				rawARHSData.AccData[2] = lsb_to_mps2(bmi08x_accel.z, 24, 16);
			}
			bmi08g_get_data_int_status(&status, &bmi);
			if (status & BMI08X_GYRO_DATA_READY_INT)
			{
				bmi08g_get_data(&bmi08x_gyro, &bmi);

				/* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
				rawARHSData.GyroData[0] = lsb_to_dps(bmi08x_gyro.x, 250, 16);
				rawARHSData.GyroData[1] = lsb_to_dps(bmi08x_gyro.y, 250, 16);
				rawARHSData.GyroData[2] = lsb_to_dps(bmi08x_gyro.z, 250, 16);
			}
			disable_bmi08x_interrupt();

			rawARHSData.lastSampleTime = HAL_GetTick();
			rawARHSData.newMeasurementExists = true;
			osSemaphoreRelease(IMUSemHandle);
		}
		osDelay(1);
	}
	/* USER CODE END StartIMUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

