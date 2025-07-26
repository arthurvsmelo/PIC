/* 
 * File:   LCD.h
 * Author: Arthur Melo
 * Version: 1.0
 * Created on 23 de Julho de 2025, 08:05
 * This header is intended to be used with a common LCD 16x2 module
 */

#ifndef LCD_H
#define	LCD_H

#ifdef	__cplusplus
extern "C" {
#endif

#define _XTAL_FREQ 16000000
/* LCD PIN LIST */
#define D0 PORTBbits.RB0
#define D1 PORTBbits.RB1
#define D2 PORTBbits.RB2
#define D3 PORTBbits.RB3
#define D4 PORTBbits.RB4
#define D5 PORTBbits.RB5
#define D6 PORTBbits.RB6
#define D7 PORTBbits.RB7
#define RS PORTDbits.RD5
#define RW PORTDbits.RD6
#define EN PORTDbits.RD7
#define BACKLIGHT PORTAbits.RA2
/* END LCD PIN LIST */
    
/* FUNCTIONS SIGNATURE */
void lcd_init(void);
void lcd_puts(const char *s);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void lcd_home(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_cursor_on(void);
void lcd_cursor_off(void);
void lcd_blink_on(void);
void lcd_blink_off(void);
void display_shift_right(void);
void display_shift_left(void);
void cursor_shift_right(void);
void cursor_shift_left(void);
void lcd_backlight(char state);
void lcd_print_float(float number, uint8_t dec);
void lcd_print_int(int number, uint8_t lenght);
/* END FUNCTIONS SIGNATURE */

#ifdef	__cplusplus
}
#endif

#endif	/* LCD_H */

