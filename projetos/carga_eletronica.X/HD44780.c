/* 
 * File:   HD44780.c
 * Author: Arthur Melo
 *
 * Created on 14 de Outubro de 2024, 23:28
 */

#include "HD44780.h"
#include <xc.h>
#include <stdio.h>

/* internal functions */
void pulse_enable(void) {
    LCD_EN = 0;
    __delay_ms(1);
    LCD_EN = 1;
    __delay_ms(1);
    LCD_EN = 0;
    __delay_ms(10);
}

void lcd_write_nibble(unsigned char ch, unsigned char rs) {

    ch = (ch >> 4) & 0x0F;
    LCD_D4 = ((ch >> 0) & 0x01);
    LCD_D5 = ((ch >> 1) & 0x01);
    LCD_D6 = ((ch >> 2) & 0x01);
    LCD_D7 = ((ch >> 3) & 0x01);
    LCD_RS = rs;
    
    pulse_enable();
}

void lcd_write_char(unsigned char data) {
    __delay_ms(2);
    lcd_write_nibble(data, 1);
    data = (data << 4);
    __delay_ms(1);
    lcd_write_nibble(data, 1);
}

void lcd_command(unsigned char cmd) {

    __delay_ms(2);
    lcd_write_nibble(cmd, 0);
    cmd = (cmd << 4);
    __delay_ms(1);
    lcd_write_nibble(cmd, 0);
}

void lcd_init(void) {

    LCD_D4 = 0;
    LCD_D5 = 0;
    LCD_D6 = 0;
    LCD_D7 = 0;
    __delay_ms(50);

    lcd_write_nibble(0x03, 0);
    __delay_us(4500);

    lcd_write_nibble(0x03, 0);
    __delay_us(4500);

    lcd_write_nibble(0x03, 0);
    __delay_us(150);

    lcd_write_nibble(0x02, 0);

    lcd_command(FUNCTION_SET);

    lcd_command(DISPLAY_SETUP);

    lcd_clear();

    lcd_command(ENTRY_MODE);
}

void lcd_clear(void) {
    lcd_command(LCD_CLEAR);
    __delay_ms(2);
}

void lcd_home(void) {
    lcd_command(LCD_HOME);
    __delay_ms(2);
}

void lcd_cursor(void) {

    lcd_command(LCD_CURSOR_ON);
}

void lcd_no_cursor(void) {

    lcd_command(LCD_CURSOR_OFF);
}

void lcd_blink(void) {

    lcd_command(LCD_CURSOR_BLINK);
}

void lcd_no_blink(void) {

    lcd_command(~LCD_CURSOR_BLINK);
}

void lcd_set_cursor(char col, char row) {

    if(row == 0) {
        lcd_command((LCD_CURSOR_LINE1 | col));
    }
    else if(row == 1) {
        lcd_command((LCD_CURSOR_LINE2 | col));
    }
    __delay_ms(5);
}

void lcd_print(const char *str) {

    while(*str != '\0') {
        lcd_write_char(*str++);
    }
    __delay_ms(5);
}

void lcd_print_integer(int n) {
    char buffer[10];
    sprintf(buffer, "%d", n);
    lcd_print(buffer);
}

void lcd_print_float(float f, unsigned decimal) {
    char buffer[10];
    char format[5]; 
    sprintf(format, "%%.%uf", decimal);
    sprintf(buffer, format, f);
    lcd_print(buffer);
}