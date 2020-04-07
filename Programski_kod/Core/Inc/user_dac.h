/*
 * user_dac.h
 *
 *  Created on: 26. ožu 2020.
 *      Author: Ivan
 */

#ifndef INC_USER_DAC_H_
#define INC_USER_DAC_H_


#include <stdint.h>


#define	DAC_1_I2C_ADDR 0x00
#define	DAC_2_I2C_ADDR 0x01
#define	DAC_3_I2C_ADDR 0x01
#define	DAC_4_I2C_ADDR 0x01
#define	DAC_5_I2C_ADDR 0x01


typedef enum dac_id_e {
	DAC_ID_1 = DAC_1_I2C_ADDR,
	DAC_ID_2 = DAC_2_I2C_ADDR,
	DAC_ID_3 = DAC_3_I2C_ADDR,
	DAC_ID_4 = DAC_4_I2C_ADDR,
	DAC_ID_5 = DAC_5_I2C_ADDR
} DAC_Id_t;


typedef enum dac_channel_e {
	DAC_CHNL_A = 0x00,
	DAC_CHNL_B = 0x00,
	DAC_CHNL_C = 0x00,
	DAC_CHNL_D = 0x00,
} DAC_Channel_t;


void DAC_Write_value(DAC_Id_t dac_id, DAC_Channel_t dac_c, uint8_t value);


#endif /* INC_USER_DAC_H_ */
