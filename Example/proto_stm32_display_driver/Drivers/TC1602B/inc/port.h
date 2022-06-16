/*
 * port.h
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#ifndef TC1602B_INC_PORT_H_
#define TC1602B_INC_PORT_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

#define DISPLAY_IR_CLEAR_DISPLAY   0b00000001
#define DISPLAY_IR_ENTRY_MODE_SET  0b00000100
#define DISPLAY_IR_DISPLAY_CONTROL 0b00001000
#define DISPLAY_IR_SET_DDRAM_ADDR  0b10000000

#define DISPLAY_IR_ENTRY_MODE_SET_INCREMENT 0b00000010
#define DISPLAY_IR_ENTRY_MODE_SET_NO_SHIFT  0b00000000

#define DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_ON  0b00000100
#define DISPLAY_IR_DISPLAY_CONTROL_DISPLAY_OFF 0b00000000
#define DISPLAY_IR_DISPLAY_CONTROL_CURSOR_ON   0b00000010
#define DISPLAY_IR_DISPLAY_CONTROL_CURSOR_OFF  0b00000000
#define DISPLAY_IR_DISPLAY_CONTROL_BLINK_ON    0b00000001
#define DISPLAY_IR_DISPLAY_CONTROL_BLINK_OFF   0b00000000

#define DISPLAY_IR_FUNCTION_SET    0b00100000
#define DISPLAY_IR_ENTRY_MODE_SET_DECREMENT 0b00000000
#define DISPLAY_IR_ENTRY_MODE_SET_SHIFT     0b00000001
#define DISPLAY_IR_FUNCTION_SET_8BITS    0b00010000
#define DISPLAY_IR_FUNCTION_SET_4BITS    0b00000000
#define DISPLAY_IR_FUNCTION_SET_2LINES   0b00001000
#define DISPLAY_IR_FUNCTION_SET_1LINE    0b00000000
#define DISPLAY_IR_FUNCTION_SET_5x10DOTS 0b00000100
#define DISPLAY_IR_FUNCTION_SET_5x8DOTS  0b00000000
#define DISPLAY_20x4_LINE3_FIRST_CHARACTER_ADDRESS 20
#define DISPLAY_20x4_LINE4_FIRST_CHARACTER_ADDRESS 84

#define DISPLAY_20x4_LINE1_FIRST_CHARACTER_ADDRESS 0
#define DISPLAY_20x4_LINE2_FIRST_CHARACTER_ADDRESS 64

#define DISPLAY_RS_INSTRUCTION 0
#define DISPLAY_RS_DATA        1

#define DISPLAY_RW_WRITE 0
#define DISPLAY_RW_READ  1

#define DISPLAY_ADDRESS 78
#define DISPLAY_PIN_A_BACKLIGHT 3

#define DISPLAY_PIN_RS  4
#define DISPLAY_PIN_RW  5
#define DISPLAY_PIN_EN  6
#define DISPLAY_PIN_D0  7
#define DISPLAY_PIN_D1  8
#define DISPLAY_PIN_D2  9
#define DISPLAY_PIN_D3 10
#define DISPLAY_PIN_D4 11
#define DISPLAY_PIN_D5 12
#define DISPLAY_PIN_D6 13
#define DISPLAY_PIN_D7 14

typedef struct {
	uint8_t pin_RS;
	uint8_t pin_RW;
	uint8_t pin_EN;
	uint8_t pin_A;
	uint8_t pin_D4;
	uint8_t pin_D5;
	uint8_t pin_D6;
	uint8_t pin_D7;

	uint8_t address;

	uint8_t data;
} display_t;

/**
 * permite determinar si la inizialiacion del display
 * termino y el display esta listo para ser utilizado
 */
bool display_port_ready();

/**
 * permite setear el estado de la backlight
 */
void display_port_light_set(uint8_t s);

/**
 * permite limpiar la pantalla
 */
void display_port_clean();

/**
 * permite habilitar la pantalla
 */
void display_port_enable_screen(bool en);

/**
 * permite inicializar el display pasando la instancia del I2C_HandleTypeDef
 */
void display_port_init(I2C_HandleTypeDef *hi2c1);

/**
 * permite escribir datos a los pines del display indicando el
 * numero de pin y el valor
 */
void display_port_pin_write(uint8_t pin_name, uint8_t value);

/**
 * permite escribir datos al bus de datos i2c
 */
void display_port_data_bus_write(uint8_t data_bus);

/**
 * permite escribir codigo de operaciones soportados por el display
 */
void display_port_code_write(uint8_t type, uint8_t dataBus);

#endif /* TC1602B_INC_PORT_H_ */
