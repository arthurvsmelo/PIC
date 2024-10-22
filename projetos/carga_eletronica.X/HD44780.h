/* 
 * File:   HD44780.h
 * Author: Arthur Melo
 *
 * Created on 14 de Outubro de 2024, 23:28
 */

#ifndef HD44780_H
#define	HD44780_H

#ifdef	__cplusplus
extern "C" {
#endif

#define _XTAL_FREQ 16000000
/* Instructions */
#define LCD_CLEAR                0x01    // clear 
#define LCD_HOME                 0x02    // set the cursor to first line and first row
#define LCD_CURSOR_BACK          0x10    // moves cursor one position back
#define LCD_CURSOR_FWD           0x14    // moves cursor one position forward
#define LCD_SHIFT_LEFT           0x18    // used to scroll text left side to scroll text
#define LCD_SHIFT_RIGHT          0x1C    // used to scroll text right side to scroll text
#define LCD_CURSOR_OFF           0x0C    // stops display cursor on screen
#define LCD_CURSOR_ON            0x0E    // turns on cursor display
#define LCD_CURSOR_BLINK         0x0F    // cursor keeps blinking
#define LCD_CURSOR_LINE1         0x80    // cursor line 1
#define LCD_CURSOR_LINE2         0xC0    // cursor line 2   
#define INSTRUCTION                 0
#define DATA                        1

/* display controller setup commands from page 46 of Hitachi datasheet */
#define FUNCTION_SET             0x28    // 4 bit interface, 2 lines, 5x8 font
#define ENTRY_MODE               0x06    // increment mode
#define DISPLAY_SETUP            0x0C    // display on, cursor off, blink off

/* pin definitions (change this acordingly to your project) */
#define LCD_EN PORTDbits.RD1
#define LCD_RS PORTDbits.RD0
#define LCD_D4 PORTDbits.RD4
#define LCD_D5 PORTDbits.RD5
#define LCD_D6 PORTDbits.RD6
#define LCD_D7 PORTDbits.RD7

/* internal functions */
void lcd_command(unsigned char cmd);
void lcd_write_nibble(unsigned char ch, unsigned char rs);
void pulse_enable(void);
void lcd_write_char(unsigned char data);

/* user functions */
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_cursor(void);
void lcd_no_cursor(void);
void lcd_blink(void);
void lcd_no_blink(void);
void lcd_set_cursor(char col, char row);
void lcd_print(const char *str);
void lcd_print_integer(int n);
void lcd_print_float(float f, unsigned decimal);

#ifdef	__cplusplus
}
#endif

#endif	/* HD44780_H */

