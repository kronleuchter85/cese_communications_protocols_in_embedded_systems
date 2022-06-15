/*
 * TC1602B.c
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#include "port.h"
#include "TC1602B.h"

void display_init(I2C_HandleTypeDef *hi2c1) {

	display_port_init(hi2c1);
}

void display_enable() {
	display_port_enable_screen(true);
}
void display_disable() {
	display_port_enable_screen(false);
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

void display_light_on() {
	display_port_light_set(1);
}
void display_light_off() {
	display_port_light_set(0);
}

void display_clean() {
	display_port_clean();
}

void display_set_cursor_mode(display_cursor_mode_t cm) {
	switch (cm) {
		case CURSOR_NONE:
			display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_DISPLAY_CONTROL
					| DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_ON
					| DISPLAY_IR_DISPLAY_CONTROL_CURSOR_OFF
					| DISPLAY_IR_DISPLAY_CONTROL_BLINK_OFF);
			break;
		case CURSOR_STATIC:
			display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_DISPLAY_CONTROL
					| DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_ON
					| DISPLAY_IR_DISPLAY_CONTROL_CURSOR_ON
					| DISPLAY_IR_DISPLAY_CONTROL_BLINK_OFF);
			break;
		case CURSOR_BLINKING:
			display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_DISPLAY_CONTROL
					| DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_ON
					| DISPLAY_IR_DISPLAY_CONTROL_CURSOR_ON
					| DISPLAY_IR_DISPLAY_CONTROL_BLINK_ON);
			break;
	}
}
