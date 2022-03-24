/*
 * ADXL345 Accelerometer I2C Driver
 *
 * Datasheet: https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
 * Schematic: http://www.haoyuelectronics.com/Attachment/GY-85/GY-85-SCH.jpg
 *
 *  Created on: Feb 18, 2022
 *      Author: Matiss
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32l4xx_hal.h"

// DEFINES

#define ADXL345_I2C_ADDR	(0x53 << 1)		/* By grounding the SDO/ALT ADDRESS pin (#12) you get alternate I2C address of 0x53 (page 10 of datasheet). */

#define ADXL345_DEVICE_ID	0xE5			/* reset value (1110 0101) */
// #define ADXL345_MEMS_ID
// #define ADXL345_PART_ID


// REGISTERS (page 14 of datasheet)

#define ADXL345_REG_DEVID			0x00

#define ADXL345_REG_BW_RATE			0x2C
#define ADXL345_REG_POWER_CTL		0x2D

#define ADXL345_REG_INT_ENABLE		0x2E
#define ADXL345_REG_INT_MAP			0x2F
#define ADXL345_REG_INT_SOURCE		0x30

#define ADXL345_REG_DATA_FORMAT		0x31
#define ADXL345_REG_DATAX0			0x32
#define ADXL345_REG_DATAX1			0x33
#define ADXL345_REG_DATAY0			0x34
#define ADXL345_REG_DATAY1			0x35
#define ADXL345_REG_DATAZ0			0x36
#define ADXL345_REG_DATAZ1			0x37


// SENSOR STRUCT

typedef struct{

	I2C_HandleTypeDef *i2cHandle;	// I2C handle

	float acc_mps2[3];	// Acceleration data ( X, Y, Z) in m/s^2

}ADXL345;


// INITIALISATION

uint8_t ADXL345_Initialise(ADXL345 *dev, I2C_HandleTypeDef *i2cHandle);


// DATA ACQUISITON

HAL_StatusTypeDef ADXL345_ReadAccelerations (ADXL345 *dev);


// LOW-LEVEL FUNCTIONS

HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t lenght);

HAL_StatusTypeDef ADXL345_WriteRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data);

#endif /* INC_ADXL345_H_ */
