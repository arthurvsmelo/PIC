/* 
 * File:   main.c
 * Author: Arthur
 *
 * Created on July 22, 2025, 9:45 AM
 */

/*
 * Pinagem usada:
 * LCD D7-D0: RB7-RB0
 * LCD E: RD7
 * LCD RW: RD6
 * LCD RS: RD5
 * LCD LED: RA2
 * LM35: RA0
 * DS1307 SCK: RE0
 * DS1307 IO: RE1
 * DS1307 RST: RE2
 * BUZZER: ?
 * BTN ADJ: ?
 * BTN LCD LED: ?
 * BTN INC: ?
 * BTN DEC: ?
 */

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include "LCD.h"

#define _XTAL_FREQ 16000000

void main() {
    lcd_init();
    while(1) {
        lcd_backlight(1);
        lcd_set_cursor(0, 0);
        lcd_puts("Hello world");
        lcd_set_cursor(1, 0);
        lcd_puts("Arthur Melo");
        __delay_ms(3000);
        lcd_backlight(0);
        lcd_clear();
        __delay_ms(500);
    }
}

