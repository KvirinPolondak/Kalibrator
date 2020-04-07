/*
 * user_dac.c
 *
 *  Created on: 26. ožu 2020.
 *      Author: Ivan
 */

#include "user_dac.h"
#include "stm32f1xx_hal.h"


extern I2C_HandleTypeDef hi2c1, hi2c2;


/* DAC5574 library */
/*
 * Writing to DAC5575
 * Adress Byte -> Control Byte -> MSB -> LSB
 * Address Byte
 * MSB																LSB
 * 	1		0		0		1		1		A1		A0		 R/~W
 *
 *
 * Control byte
 * MSB												LSB
 * 0	0	L1	L0	X	Sel1	Sel0	PD0
 *
 * L1 and L0 should be "01" to write and update DAC outputs
 *
 * Sel1 Sel0	Channel
 *  0  		0       A
 *  0  		1				B
 *  1  		0				C
 *  1  		1				D
 *
 *  PD0 should be "0" for Normal operation
 *
 *  MSB Byte
 *  Unsigned 8-bit DAC output value
 *
 *  LSB Byte
 *  8 Don't care bits
 */


void DAC_Write_value(DAC_Id_t dac_id, DAC_Channel_t dac_c, uint8_t value) {
	/* Here we should put I2C function to write into DAC output
	 * register to output some voltage.
	 */
	uint8_t data[3];
	uint8_t address, control, msb, lsb;

	if (dac_id == DAC_ID_1 ||
			dac_id == DAC_ID_2 ||
			dac_id == DAC_ID_3 ||
			dac_id == DAC_ID_4)
	{
		HAL_I2C_Master_Transmit(&hi2c1, dac_id<<1, data, 3, 100);
	} else
	{
		HAL_I2C_Master_Transmit(&hi2c2, dac_id<<1, data, 3, 100);
	}


}

















