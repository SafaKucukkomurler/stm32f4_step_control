#ifndef INC_STEPPERMOTOR_H_
#define INC_STEPPERMOTOR_H_

#include "stm32f4xx_hal.h"

#define NUMBER_OF_MOTORS 3

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile int motor1intFlag, motor2intFlag, motor3intFlag, motor4intFlag;

typedef struct {
	uint32_t step;
	uint32_t stepCounter;
	uint8_t state;
} stepperMotor_s;

stepperMotor_s stepperMotors[NUMBER_OF_MOTORS];

void stepperMotorMove(uint32_t motorNumber, uint8_t direction, uint32_t step);

#endif
