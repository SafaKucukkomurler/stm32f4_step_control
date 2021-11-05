#include "stepperMotor.h"

void stepperMotorMove(uint32_t motorNumber, uint8_t direction, uint32_t step)  
{
	switch (motorNumber)
	{

	case 0:
	{
		stepperMotors[0].step = step;
		
		if(direction==0)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);	
		else if (direction==1)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	
		
		stepperMotors[0].state = 1;
		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
		break;
	}
	
	case 1:
	{
		stepperMotors[1].step = step;
		
		if(direction==0)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);	
		else if (direction==1)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);	
		
		stepperMotors[1].state = 1;
		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
		break;
	}
	
	case 2:
	{
		stepperMotors[2].step = step;
		
		if(direction==0)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);	
		else if (direction==1)
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);	
		
		stepperMotors[2].state = 1;
		HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);
		break;
	}
	
//	case 3:
//		stepperMotors[3].step = step;
//		if(direction==0)
//			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);	
//		else if (direction==1)
//			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);	
//		stepperMotors[3].state = 1;
//		HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_4);
//		break;

	}

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			TIM3->SR &= (~TIM_SR_CC1IF);
			TIM3->DIER &= (~TIM_DIER_CC1IE);
			motor1intFlag = 1;
		}
		
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			TIM3->SR &= (~TIM_SR_CC2IF);
			TIM3->DIER &= (~TIM_DIER_CC2IE);
			motor2intFlag = 1;
		}
	}

	if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			TIM4->SR &= (~TIM_SR_CC3IF);
			TIM4->DIER &= (~TIM_DIER_CC3IE);
			motor3intFlag = 1;
		}

//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//		{
//			TIM4->SR &= (~TIM_SR_CC4IF);
//			TIM4->DIER &= (~TIM_DIER_CC4IE);
//			motor4intFlag = 1;
//		}
	}

}
