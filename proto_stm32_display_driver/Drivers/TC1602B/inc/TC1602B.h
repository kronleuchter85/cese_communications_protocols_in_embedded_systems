/*
 * TC1602B.h
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#ifndef TC1602B_INC_TC1602B_H_
#define TC1602B_INC_TC1602B_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {

	//
	// sin cursor
	//
	CURSOR_NONE
	,
	//
	// cursor estatico sin titilar
	//
	CURSOR_STATIC
	,
	//
	// cursor titilando
	//
	CURSOR_BLINKING

} display_cursor_mode_t;

void display_enable();
void display_disable();
void display_set_cursor_mode(display_cursor_mode_t cm);
void display_light_on();
void display_light_off();
void display_clean();

void display_init(I2C_HandleTypeDef *hi2c1);
void display_print_string(char const *str);
void display_set_position(uint8_t pos_x, uint8_t pos_y);

#endif /* TC1602B_INC_TC1602B_H_ */
