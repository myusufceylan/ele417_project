/*
 * servo.h
 *
 *  Created on: Jan 1, 2024
 *      Author: yildiray
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f3xx_hal.h"

//#define SERVO_MAX_PULSEWIDTH 2000 // Örneğin 2ms
//#define SERVO_MIN_PULSEWIDTH 1000 // Örneğin 1ms
//// Servo açılarını ve PWM değerlerini tanımla
//#define MIN_ANGLE 0
//#define MAX_ANGLE 180
//#define MIN_PWM_WIDTH 500   // Bu değerleri servo motorunuzun spesifikasyonlarına göre ayarlayın
//#define MAX_PWM_WIDTH 2500
//
//
//void Servo_Init(TIM_HandleTypeDef *htim);
//void Servo_WriteAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle);
//


typedef struct{
	GPIO_TypeDef*		servo_GPIOx;
	uint32_t 			servo_GPIO_pin;
	TIM_HandleTypeDef*	Timer_Handle;
	uint32_t			Timer_Channel;
	uint32_t 			initial_angle;
}servo_config;


typedef struct{
	GPIO_TypeDef*		motor_GPIOx;
	uint32_t 			motor_GPIO_pin;
	TIM_HandleTypeDef*	Timer_Handle_Motor;
	uint32_t			Timer_Channel_Motor;
	uint32_t 			calibrate_motor;
}motor_config;


void init_servo(	servo_config* 		servo_config,
					GPIO_TypeDef*		servo_GPIOx,
					uint32_t 			servo_GPIO_pin,
					TIM_HandleTypeDef*	Timer_Handle,
					uint32_t			Timer_Channel,
					uint32_t 			initial_angle);



void init_motor(	motor_config* 		motor_config,
					GPIO_TypeDef*		motor_GPIOx,
					uint32_t 			motor_GPIO_pin,
					TIM_HandleTypeDef*	Timer_Handle_Motor,
					uint32_t			Timer_Channel_Motor,
					uint32_t 			calibrate_motor);

uint32_t map_servo(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
uint32_t map_motor(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

void motor_speed(motor_config* motor_config, uint32_t speed);
void servo_angle(servo_config* servo_config, uint32_t angle);





#endif /* INC_SERVO_H_ */
