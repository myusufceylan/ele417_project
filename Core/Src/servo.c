
#include "servo.h"



void init_servo(	servo_config* 		servo_config,
					GPIO_TypeDef*		servo_GPIOx,
					uint32_t 			servo_GPIO_pin,
					TIM_HandleTypeDef*	Timer_Handle,
					uint32_t			Timer_Channel,
					uint32_t 			initial_angle)
{
	servo_config->servo_GPIOx 			=servo_GPIOx;
	servo_config->servo_GPIO_pin 		=servo_GPIO_pin;
	servo_config->Timer_Handle 			=Timer_Handle;
	servo_config->Timer_Channel			=Timer_Channel;

	// Initialize the servo with the initial angle
	servo_angle(servo_config, initial_angle);

}

void init_motor(	motor_config* 		motor_config,
					GPIO_TypeDef*		motor_GPIOx,
					uint32_t 			motor_GPIO_pin,
					TIM_HandleTypeDef*	Timer_Handle_Motor,
					uint32_t			Timer_Channel_Motor,
					uint32_t 			calibrate_motor)
{

	motor_config->motor_GPIOx 			=motor_GPIOx;
	motor_config->motor_GPIO_pin 		=motor_GPIO_pin;
	motor_config->Timer_Handle_Motor 	=Timer_Handle_Motor;
	motor_config->Timer_Channel_Motor	=Timer_Channel_Motor;

	motor_speed(motor_config,calibrate_motor);
	HAL_Delay(500);
}

uint32_t map_servo(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


uint32_t map_motor(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void servo_angle(servo_config* servo_config, uint32_t angle)
{



	if(0!= angle)
	{
		HAL_TIM_PWM_Start(servo_config->Timer_Handle, servo_config->Timer_Channel);
		__HAL_TIM_SET_COMPARE(servo_config->Timer_Handle,servo_config->Timer_Channel, map_servo(angle,0,180,1000,2000));
	}

}


void motor_speed(motor_config* motor_config, uint32_t speed){
	if(0!= speed)
	{
		HAL_TIM_PWM_Start(motor_config->Timer_Handle_Motor, motor_config->Timer_Channel_Motor);
		__HAL_TIM_SET_COMPARE(motor_config->Timer_Handle_Motor,motor_config->Timer_Channel_Motor, map_motor(speed,0,180,1000,2000));
	}
}
