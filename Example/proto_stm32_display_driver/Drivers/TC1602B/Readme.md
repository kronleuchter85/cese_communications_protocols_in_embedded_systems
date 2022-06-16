# 
# TC1602B - Display driver
# 

#### El siguiente driver esta desarrollado para la plataforma STM32
#### Contenido del directorio

 - /inc 		(directorio de headers)
 - /inc/TC1602B.h 	(archivo a incluir para poder hacer uso del driver)
 - /inc/port.h 	(archivo de uso interno, no incluir para usar el driver)
 - /src 		(directorio de implementacion)
 - /src/TC1602B.c  	(capa de abstraccion de alto nivel que implementa TC1602B.h)
 - /src/port.c		(implementacion del driver)
  
#### Instrucciones para buildear 

1. Incluir el directorio completo TC1602B en el directorio Drivers de un proyecto STM32
2. Configurar el entorno de desarrollo para incluir headers del directorio inc agregado
3. Hacer uso de las primitivas publicadas en TC1602B.h para controlar el display
	+ display_enable()
	+ display_disable()
	+ display_set_cursor_mode( display_cursor_mode_t )
	+ display_light_on()
	+ display_light_off() 
	+ display_clean()
	+ display_init(I2C_HandleTypeDef *hi2c1)
	+ display_print_string(char const *str)
	+ display_set_position(uint8_t pos_x, uint8_t pos_y)
4. Compilar
