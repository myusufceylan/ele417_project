/*
 * MPU9250_Config.h
 *
 *  Created on: Dec 31, 2023
 *      Author: yildiray
 */

#ifndef INC_MPU9250_CONFIG_H_
#define INC_MPU9250_CONFIG_H_

#include "stm32f3xx_hal.h"
//#define USE_SPI 				1

#ifdef USE_SPI
	#include "spi.h"
	#define MPU9250_SPI		hspi1
	#define	MPU9250_CS_GPIO		MPU9250_CS_GPIO_Port
	#define	MPU9250_CS_PIN		MPU9250_CS_Pin
#else
	extern I2C_HandleTypeDef hi2c2;
	#define _MPU9250_I2C	hi2c2
	#define DEVICE_ADD		208
#endif

#endif /* INC_MPU9250_CONFIG_H_ */
