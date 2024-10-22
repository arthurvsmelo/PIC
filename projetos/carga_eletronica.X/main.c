/* 
 * File:   main.c
 * Author: Arthur Melo
 *
 * Created on 14 de Outubro de 2024, 23:26
 */

/* CONFIGURATION BITS */
#pragma config FOSC = HS, WDTE = OFF, PWRTE = OFF, BOREN = ON, LVP = OFF, CPD = OFF, WRT = OFF, DEBUG = OFF, CP = OFF

#include <xc.h>
#include "HD44780.h"

#define _XTAL_FREQ 16000000
#define LCD_EN PORTDbits.RD1
#define LCD_RS PORTDbits.RD0
#define LCD_D4 PORTDbits.RD4
#define LCD_D5 PORTDbits.RD5
#define LCD_D6 PORTDbits.RD6
#define LCD_D7 PORTDbits.RD7

int main() {
    TRISD = 0x00;
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Hello World!");
    lcd_set_cursor(5, 1);
    lcd_blink();
    //lcd_print_float((float)25.62, 1);
    while(1) {
        
    }
}

