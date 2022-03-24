/*
 * ADXL345.c
 *
 * Datasheet: https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
 *
 *  Created on: Feb 18, 2022
 *      Author: Matiss
 */

#include "ADXL345.h"
#include "main.h"

uint8_t ADXL345_Initialise(ADXL345 *dev, I2C_HandleTypeDef *i2cHandle){

	// Set struct parameters
	dev->i2cHandle = i2cHandle;
	dev->acc_mps2[0]= 0.0f;
	dev->acc_mps2[1]= 0.0f;
	dev->acc_mps2[2]= 0.0f;

	// Store number of transaction errors (returned at the end of the function)
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// Check device ID (datasheet page 14)
	uint8_t regData;

	status = ADXL345_ReadRegister(dev, ADXL345_REG_DEVID, &regData);
	errNum += (status != HAL_OK);

	if (regData != ADXL345_DEVICE_ID){

		return 255;
	}

	// Set device bandwidth, output data rate and low power mode in BW_RATE register, in this setup it will stay as default value 0x0A (00001010)
	/*regData = 0x0A;

	status = ADXL345_WriteRegisters(dev, ADXL345_REG_BW_RATE, &regData);
	errNum += (status != HAL_OK);
	*/

	// Set device Measure Bit in POWER_CTL register to 1 so that device is in measurement mode
	regData = 0x08;

	status = ADXL345_WriteRegisters(dev, ADXL345_REG_POWER_CTL, &regData);
	errNum += (status != HAL_OK);

	// Toggle LED
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_Delay(5);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	// Return number of errors (0 if successful initialisation)
	return errNum;
}


HAL_StatusTypeDef ADXL345_ReadAccelerations(ADXL345 *dev){

	/*
	 * Justify Bit
	 * A setting of 1 in the justify bit selects left (MSB) justified mode,
	 * and a setting of 0 selects right justified mode with sign extension.
	*/

	// Read raw values from accelaration registers (x, y, z -> 16 bits each)
	uint8_t regData[6];

	HAL_StatusTypeDef status = ADXL345_ReadRegisters(dev, ADXL345_REG_DATAX0, regData, 6);

	int16_t accRawSigned[3];

	accRawSigned[0] = (int16_t) ((regData[1] << 8) | (regData[0]));
	accRawSigned[1] = (int16_t) ((regData[3] << 8) | (regData[2]));
	accRawSigned[2] = (int16_t) ((regData[5] << 8) | (regData[4]));

	/*
	 * Range Setting by default is +/- 2g
	 * Number of bits are 16 - 1 = 15, because of the two's complement (sign bit) you lose 1 bit.
	 *
	 * So the sensitivity ( (m/(s^2)) / LSB ) = ( Range / 2^(# of bits) ) * 9.81 (m/(s^2) = ( 2 / 2^15 ) * 9.81 (m/(s^2) = 0.00006103515 * 9.81 (m/(s^2))
	 */

	dev->acc_mps2[0] = 0.00006103515f * 9.81f * accRawSigned[0];
	dev->acc_mps2[1] = 0.00006103515f * 9.81f * accRawSigned[1];
	dev->acc_mps2[2] = 0.00006103515f * 9.81f * accRawSigned[2];

	return status;

}


// LOW-LEVEL FUNCTIONS

HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t lenght){

	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL345_WriteRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
