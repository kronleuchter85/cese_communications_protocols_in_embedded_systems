/*
 * TC1602B.c
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#include "TC1602B.h"

void display_init(I2C_HandleTypeDef *hi2c1) {

	display_port_init(hi2c1);
}

void display_set_position(uint8_t pos_x, uint8_t pos_y) {

	switch (pos_y) {
		case 0:
			display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_SET_DDRAM_ADDR
					| (DISPLAY_20x4_LINE1_FIRST_CHARACTER_ADDRESS + pos_x));
			HAL_Delay(1);
			break;

		case 1:
			display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_SET_DDRAM_ADDR
					| (DISPLAY_20x4_LINE2_FIRST_CHARACTER_ADDRESS + pos_x));
			HAL_Delay(1);
			break;

	}
}

void display_print_string(char const *str) {

	while (*str) {
		display_port_code_write(DISPLAY_RS_DATA, *str++);
	}
}

