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

/**
 * Permite activar el display representando por 
 * pantalla lo que tiene almacenado
 */
void display_enable();

/**
 * Permite desactivar el display sin borrar 
 * el contenido que tiene almacenado para representar
 */
void display_disable();

/**
 * Permite setear el tipo de cursor a representar
 */
void display_set_cursor_mode(display_cursor_mode_t cm);

/**
 * permite prender la backlight del display
 */
void display_light_on();
/**
 * permite apagar la backlight del display
 */
void display_light_off();

/**
 * permite limpiar el contenido del displauy
 */
void display_clean();

/**
 * permite inicializar el display
 */
void display_init(I2C_HandleTypeDef *hi2c1);

/**
 * Permite imprimir un mensaje por pantalla del display
 */
void display_print_string(char const *str);

/**
 * permite fijar la posicion en la cual se debe posicionar el cursor 
 * para comenzar a imprimir
 */
void display_set_position(uint8_t pos_x, uint8_t pos_y);

#endif /* TC1602B_INC_TC1602B_H_ */
