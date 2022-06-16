/*
 * tests.c
 *
 *  Created on: 15 Jun 2022
 *      Author: gonzalo
 */

#include "TC1602B.h"

extern I2C_HandleTypeDef hi2c1;

void test_initialize() {

	//
	// inicializacion del display
	//
	display_init(&hi2c1);
}

void test_set_lights() {

	uint8_t time = 3;

	while (time--) {

		display_light_off();
		HAL_Delay(500);

		display_light_on();
		HAL_Delay(500);
	}

}

void test_print_text() {

	//
	// seteando l posicion 0,0 en el display
	//
	display_set_position(0, 0);

	//
	// imprimiendo un mensaje de prueba
	//
	display_print_string("UBA - PCSE");

	//
	// seteando l posicion 0,0 en el display
	//
	display_set_position(0, 1);
	//
	// imprimiendo un mensaje de prueba
	//
	display_print_string("Gonzalo Carreno");
}

void test_set_cursor_mode_blinking() {

	display_set_cursor_mode(CURSOR_BLINKING);
}

void test_set_cursor_mode_static() {
	display_set_cursor_mode(CURSOR_STATIC);
}

void test_set_cursor_mode_none() {
	display_set_cursor_mode(CURSOR_NONE);
}

void test_disable_enable_screen() {

	display_disable();

	HAL_Delay(2000);

	display_enable();
}

void test_clear_screen() {

	display_clean();
}
