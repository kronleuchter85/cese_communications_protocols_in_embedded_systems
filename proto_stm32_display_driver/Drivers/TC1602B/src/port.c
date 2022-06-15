/*
 * TC1602B.c
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#include "port.h"

I2C_HandleTypeDef *i2c_handler;
display_t display_connector;
uint8_t initial_eigth_bit_communication_is_completed = 0;

void display_port_light_set(uint8_t s) {
	display_port_pin_write(DISPLAY_PIN_A_BACKLIGHT, s);
}

uint8_t display_port_ready() {
	return initial_eigth_bit_communication_is_completed;
}

void display_port_init(I2C_HandleTypeDef *h) {

	i2c_handler = h;

	display_connector.address = DISPLAY_ADDRESS;
	display_connector.data = 0b00000000;

	display_port_light_set(1);

	initial_eigth_bit_communication_is_completed = 0;

	HAL_Delay(50);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_FUNCTION_SET | DISPLAY_IR_FUNCTION_SET_8BITS);
	HAL_Delay(5);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_FUNCTION_SET | DISPLAY_IR_FUNCTION_SET_8BITS);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_FUNCTION_SET | DISPLAY_IR_FUNCTION_SET_8BITS);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_FUNCTION_SET | DISPLAY_IR_FUNCTION_SET_4BITS);
	HAL_Delay(1);

	initial_eigth_bit_communication_is_completed = 1;

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_FUNCTION_SET | DISPLAY_IR_FUNCTION_SET_4BITS
			| DISPLAY_IR_FUNCTION_SET_2LINES | DISPLAY_IR_FUNCTION_SET_5x8DOTS);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_DISPLAY_CONTROL
			| DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_OFF
			| DISPLAY_IR_DISPLAY_CONTROL_CURSOR_OFF
			| DISPLAY_IR_DISPLAY_CONTROL_BLINK_OFF);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_CLEAR_DISPLAY);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_ENTRY_MODE_SET
			| DISPLAY_IR_ENTRY_MODE_SET_INCREMENT
			| DISPLAY_IR_ENTRY_MODE_SET_NO_SHIFT);
	HAL_Delay(1);

	display_port_code_write(DISPLAY_RS_INSTRUCTION, DISPLAY_IR_DISPLAY_CONTROL
			| DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_ON
			| DISPLAY_IR_DISPLAY_CONTROL_CURSOR_OFF
			| DISPLAY_IR_DISPLAY_CONTROL_BLINK_OFF);
	HAL_Delay(1);

}

void display_port_code_write(uint8_t type, uint8_t dataBus) {

	if (type == DISPLAY_RS_INSTRUCTION) {
		display_port_pin_write(DISPLAY_PIN_RS, DISPLAY_RS_INSTRUCTION);
	} else {
		display_port_pin_write(DISPLAY_PIN_RS, DISPLAY_RS_DATA);
	}
	display_port_pin_write(DISPLAY_PIN_RW, DISPLAY_RW_WRITE);
	display_port_data_bus_write(dataBus);
}

void display_port_data_bus_write(uint8_t data_bus) {

	display_port_pin_write(DISPLAY_PIN_EN, 0);

	display_port_pin_write(DISPLAY_PIN_D7, data_bus & 0b10000000);
	display_port_pin_write(DISPLAY_PIN_D6, data_bus & 0b01000000);
	display_port_pin_write(DISPLAY_PIN_D5, data_bus & 0b00100000);
	display_port_pin_write(DISPLAY_PIN_D4, data_bus & 0b00010000);

	if (initial_eigth_bit_communication_is_completed) {

		display_port_pin_write(DISPLAY_PIN_EN, 1);

		HAL_Delay(1);

		display_port_pin_write(DISPLAY_PIN_EN, 0);
		HAL_Delay(1);
		display_port_pin_write(DISPLAY_PIN_D7, data_bus & 0b00001000);
		display_port_pin_write(DISPLAY_PIN_D6, data_bus & 0b00000100);
		display_port_pin_write(DISPLAY_PIN_D5, data_bus & 0b00000010);
		display_port_pin_write(DISPLAY_PIN_D4, data_bus & 0b00000001);
	}

	display_port_pin_write(DISPLAY_PIN_EN, 1);
	HAL_Delay(1);
	display_port_pin_write(DISPLAY_PIN_EN, 0);
	HAL_Delay(1);
}

void display_port_pin_write(uint8_t pin_name, uint8_t value) {

	switch (pin_name) {
		case DISPLAY_PIN_D4:
			display_connector.displayPin_D4 = value;
			break;
		case DISPLAY_PIN_D5:
			display_connector.displayPin_D5 = value;
			break;
		case DISPLAY_PIN_D6:
			display_connector.displayPin_D6 = value;
			break;
		case DISPLAY_PIN_D7:
			display_connector.displayPin_D7 = value;
			break;
		case DISPLAY_PIN_RS:
			display_connector.displayPin_RS = value;
			break;
		case DISPLAY_PIN_EN:
			display_connector.displayPin_EN = value;
			break;
		case DISPLAY_PIN_RW:
			display_connector.displayPin_RW = value;
			break;
		case DISPLAY_PIN_A_BACKLIGHT:
			display_connector.displayPin_A = value;
			break;
		default:
			break;
	}

	display_connector.data = 0b00000000;

	if (display_connector.displayPin_RS)
		display_connector.data |= 0b00000001;
	if (display_connector.displayPin_RW)
		display_connector.data |= 0b00000010;
	if (display_connector.displayPin_EN)
		display_connector.data |= 0b00000100;
	if (display_connector.displayPin_A)
		display_connector.data |= 0b00001000;
	if (display_connector.displayPin_D4)
		display_connector.data |= 0b00010000;
	if (display_connector.displayPin_D5)
		display_connector.data |= 0b00100000;
	if (display_connector.displayPin_D6)
		display_connector.data |= 0b01000000;
	if (display_connector.displayPin_D7)
		display_connector.data |= 0b10000000;

	HAL_I2C_Master_Transmit(i2c_handler, display_connector.address, &display_connector.data, 1, 100);

}
