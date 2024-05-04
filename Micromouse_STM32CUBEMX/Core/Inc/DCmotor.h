/**
  ******************************************************************************
  * @file    DCmotor.h
  * @brief   Header file for DC motor control with TB6612 driver
  * @author  Adam Iwachów
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 OpenAI. All rights reserved.
  * </center></h2>
  *
  ******************************************************************************
  */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include <stdint.h>
#include "gpio.h"
#include "tim.h"

/**
 * @brief Enumeration for specifying the left motor, right motor, or both motors.
 */
typedef enum{
    Motor_L = 0,  /**< Left motor */
    Motor_R = 1,  /**< Right motor */
    BothMotors = 2,  /**< Both motors */
} Motor_L_or_R;

/**
 * @brief Enumeration for specifying the motor operating mode.
 */
typedef enum{
    NormalMode = 0,  /**< Normal mode */
    ShortBrake = 1,  /**< Short brake mode */
    StopMode = 2,  /**< Stop mode */
} Motor_Mode;

/**
 * @brief Enumeration for specifying the motor rotation direction.
 */
typedef enum{
    CW = 0,  /**< Clockwise rotation */
    CCW = 1,  /**< Counter-clockwise rotation */
} Motor_Direction;

/**
 * @brief Structure representing the TB6612 motor driver configuration.
 */
typedef struct{
    GPIO_TypeDef *A01_port;
    uint16_t A01_pin;

    GPIO_TypeDef *A02_port;
    uint16_t A02_pin;

    GPIO_TypeDef *B01_port;
    uint16_t B01_pin;

    GPIO_TypeDef *B02_port;
    uint16_t B02_pin;

} TB6612_t;

/**
 * @brief Initialize the motor driver with the specified pins and ports.
 * @param dev: Pointer to the motor driver structure
 * @retval None
 */
void MotorDriverInit(TB6612_t *dev);

/**
 * @brief Control the direction of the specified motor or both motors.
 * @param dev: Pointer to the motor driver structure
 * @param Direction: Specifies the direction of rotation
 * @param Motor: Specifies which motor(s) to control
 * @retval None
 */
void ControlMotorDirection(TB6612_t *dev, Motor_Direction Direction, Motor_L_or_R Motor);

/**
 * @brief Control the speed of the specified motor or both motors.
 * @param PWM: Desired PWM value
 * @param Motor: Specifies which motor(s) to control
 * @retval None
 */
void ControlMotorSpeed(uint16_t PWM, Motor_L_or_R Motor);

/**
 * @brief Stop the specified motor or both motors.
 * @param dev: Pointer to the motor driver structure
 * @param Motor: Specifies which motor(s) to stop
 * @retval None
 */
void StopMotor(TB6612_t *dev, Motor_L_or_R Motor);

#endif /* INC_DCMOTOR_H_ */
