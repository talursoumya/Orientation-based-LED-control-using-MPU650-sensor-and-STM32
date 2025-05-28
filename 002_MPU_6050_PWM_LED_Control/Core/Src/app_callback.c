/*
 * app_callback.c
 *
 *  Created on: Mar 26, 2025
 *      Author: talur
 */


#include "main.h"
#include "MPU6050.h"
#include "cmsis_os.h"
#include "task.h"
//called during update event of the counter



extern volatile uint32_t g_channel_2_state;
extern volatile uint32_t g_channel_3_state;
extern I2C_HandleTypeDef hi2c1;
extern osThreadId_t SensorReadTaskHandle;

#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  g_counter=0;
  g_channel_2_state = 16;
  g_channel_3_state = 8;
}
#endif

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
	  g_channel_2_state = 0;

  }

  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
	  g_channel_3_state = 0;
  }
}


void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_ErrorCallback could be implemented in the user file
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == mpu6050int_Pin)
	{
		mpu6050_interrupt_handle(&hi2c1);
	}
}

//this is the ISR function
void mpu6050_raw_data_ready_callback()
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if(SensorReadTaskHandle)
	{
		xTaskNotifyFromISR(SensorReadTaskHandle, 0 , eNoAction ,&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

}
