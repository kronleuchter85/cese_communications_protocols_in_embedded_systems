/*
 * TC1602B.h
 *
 *  Created on: Jun 15, 2022
 *      Author: gonzalo
 */

#ifndef TC1602B_INC_TC1602B_H_
#define TC1602B_INC_TC1602B_H_

#include "port.h"

void display_init(I2C_HandleTypeDef *hi2c1);
void display_print_string(char const *str);
void display_set_position(uint8_t pos_x, uint8_t pos_y);

#endif /* TC1602B_INC_TC1602B_H_ */
