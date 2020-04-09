/*
 * user_commands.c
 *
 *  Created on: 26. ožu 2020.
 *      Author: Ivan
 */


#include <ctype.h>
#include <math.h>

#include "user_dac.h"

/* Commands from PC or Bluetooth have this definition:
 *
 * 			LED_xx_y.yyy
 *
 * where:
 * 	- xx is LED Id in format: 00, 01, 02, ... , 09, 10, 11, ... , 98, 99
 * 	- y.yyy is rational number in format: 0.000, 0.001, 0.002, ... , 0.999, 1.000
 * 		where 0.000 is completely turned off LED and 1.000 is completely turned on LED.
 *
 *
 * If commands won't have this kind of definition the commands will be ignored.
 *
 **/


void CMD_Read(uint32_t *led_id, float *led_val, const uint8_t *cmd_in) {
	/* Check if command has defined structure */
	if (cmd_in[0] != 'L' 		  ||
			cmd_in[1] != 'E' 			||
			cmd_in[2] != 'D' 			||
			cmd_in[3] != '_' 		 	||
			! isdigit(cmd_in[4]) 	||
			! isdigit(cmd_in[5]) 	||
			cmd_in[6] != '_' 		 	||
			! isdigit(cmd_in[7]) 	||
			cmd_in[8] != '.'      ||
			! isdigit(cmd_in[9]) 	||
			! isdigit(cmd_in[10]) ||
			! isdigit(cmd_in[11])
		) {
		return;	/* Failure: wrong command */
	}

	/* String to int and float */
	*led_id = cmd_in[4] * 10 + cmd_in[5];

	*led_val = 	1.0f   * cmd_in[7]  +
							0.1f   * cmd_in[9]  +
							0.01f  * cmd_in[10] +
							0.001f * cmd_in[11];

	return; /* Success */
}


void CMD_Write(uint32_t led_id, float led_val) {
	/* led_val = [0.0, 1.0], 256 is max dac register value */
	//uint8_t dac_value = roundf(led_val * 256);

	/* Select DAC id based on led_id */
	switch (led_id) {
	case 0:

		break;

	case 1:

		break;
	case 2:

		break;

	case 3:

		break;
	case 4:

		break;

	case 5:

		break;
	case 6:

		break;

	case 7:

		break;
	case 8:

		break;

	case 9:

		break;

	}

}











