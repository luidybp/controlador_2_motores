/*#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c1;  // Usa o I2C1 que ativamos no CubeMX

// Função interna para pulsar o pino Enable (E) do display
void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    
    // O bit 3 (0x08) mantém o Backlight aceso
    data_t[0] = data_u | 0x0C;  // EN=1, RS=0
    data_t[1] = data_u | 0x08;  // EN=0, RS=0
    data_t[2] = data_l | 0x0C;  // EN=1, RS=0
    data_t[3] = data_l | 0x08;  // EN=0, RS=0
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    
    data_t[0] = data_u | 0x0D;  // EN=1, RS=1
    data_t[1] = data_u | 0x09;  // EN=0, RS=1
    data_t[2] = data_l | 0x0D;  // EN=1, RS=1
    data_t[3] = data_l | 0x09;  // EN=0, RS=1
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t *) data_t, 4, 100);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01); // Comando de limpar display
    HAL_Delay(5);       // AUMENTADO DE 2ms PARA 5ms (Limpar a memória RAM do display é o processo mais lento)
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd(col);
}

void lcd_init(void)
{
    HAL_Delay(50); // Aguarda a tensão do LCD estabilizar
    
    // Sequência à prova de falhas para forçar o modo 4-bits no PCF8574
    lcd_send_cmd(0x33); // Envia o nibble '3' duas vezes sincronizadas
    HAL_Delay(5);
    lcd_send_cmd(0x32); // Envia o nibble '3' e depois o '2' (Trava o modo 4 bits de vez)
    HAL_Delay(5);

    // A partir daqui, o display está 100% sincronizado e pronto para receber configurações
    lcd_send_cmd(0x28); // Function set: 2 linhas, fonte 5x8
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Desliga display para limpeza segura
    HAL_Delay(1);
    lcd_send_cmd(0x01); // Limpa o display completamente
    HAL_Delay(5);
    lcd_send_cmd(0x06); // Entry mode: Incrementa cursor automaticamente
    HAL_Delay(1);
    lcd_send_cmd(0x0C); // Liga display, cursor desligado
}
void lcd_send_string(char *str)
{
    while (*str) {
        lcd_send_data(*str++);
    }
}

*/