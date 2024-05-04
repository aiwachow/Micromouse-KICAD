/*
 * DCmotor.c
 *
 *  Created on: 2 maj 2024
 *      Author: Adam Iwachów
 */

#include "DCmotor.h"



void MotorDriverInit(TB6612_t *dev)
{

	dev->A01_pin  = MotorR_AIN1_Pin;
	dev->A01_port = MotorR_AIN1_GPIO_Port;

	dev->A02_pin  = MotorR_AIN2_Pin;
	dev->A02_port = MotorR_AIN2_GPIO_Port;


	dev->B01_pin  = MotorL_BIN1_Pin;
	dev->B01_port = MotorL_BIN1_GPIO_Port;

	dev->B02_pin  = MotorL_BIN2_Pin;
 	dev->B02_port = MotorL_BIN2_GPIO_Port;


	ControlMotorDirection(dev, CCW, Motor_R);

	ControlMotorSpeed(0, Motor_R);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

}


void StopMotor(TB6612_t *dev, Motor_L_or_R Motor)
{

	if(Motor == Motor_L){
		HAL_GPIO_WritePin(dev->B01_port,dev->B01_pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->B02_port,dev->B02_pin , GPIO_PIN_RESET);
	}

	if(Motor == Motor_R){
		HAL_GPIO_WritePin(dev->A01_port,dev->A01_pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->A02_port,dev->A02_pin , GPIO_PIN_RESET);
	}

	if(Motor == BothMotors){
		HAL_GPIO_WritePin(dev->B01_port,dev->B01_pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->B02_port,dev->B02_pin , GPIO_PIN_RESET);

		HAL_GPIO_WritePin(dev->A01_port,dev->A01_pin , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->A02_port,dev->A02_pin , GPIO_PIN_RESET);

	}



}


void ControlMotorDirection(TB6612_t *dev, Motor_Direction Direction, Motor_L_or_R Motor)
{

if(Direction == CW){
	if(Motor == Motor_L){
			HAL_GPIO_WritePin(dev->B01_port,dev->B01_pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev->B02_port,dev->B02_pin , GPIO_PIN_RESET);
		}

		if(Motor == Motor_R){
			HAL_GPIO_WritePin(dev->A01_port,dev->A01_pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev->A02_port,dev->A02_pin , GPIO_PIN_RESET);
		}

}

else if(Direction == CCW){
	if(Motor == Motor_L){
			HAL_GPIO_WritePin(dev->B01_port,dev->B01_pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev->B02_port,dev->B02_pin , GPIO_PIN_SET);
		}

		if(Motor == Motor_R){
			HAL_GPIO_WritePin(dev->A01_port,dev->A01_pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev->A02_port,dev->A02_pin , GPIO_PIN_SET);
		}

	}
}



void ControlMotorSpeed(uint16_t speed, Motor_L_or_R Motor)
{

	if(speed > 100){
		speed = 99;
	}

	if(Motor == Motor_R){
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, speed);
	}

	else if(Motor == Motor_L){
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, speed);
	}

	else if(Motor == BothMotors){
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, speed);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, speed);
	}


}
