/*#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"

// Define o endereço I2C do seu módulo. 
// O padrão do PCF8574T é 0x27 (no STM32 escrevemos 0x4E, que é 0x27 deslocado 1 bit).
// Se for o chip PCF8574AT, o endereço será 0x7E (0x3F deslocado).
#define LCD_ADDR 0x7E

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);

#endif
*/