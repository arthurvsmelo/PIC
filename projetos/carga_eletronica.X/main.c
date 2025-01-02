/*
 * File:   main.c
 * Author: Arthur Melo
 * 
 * Created on 14 de Novembro de 2024, 08:33
 */
/* Configuration Bits */
#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config DEBUG = OFF
#pragma config CP = OFF

#include <xc.h>
#include "lcd.h"

#define _XTAL_FREQ 20000000

void main(void) {
	
	ADCON1bits.PCFG = 0b0111; /* todos os pinos do port A como digitais */
	CMCONbits.CM    = 0b000;  /* desliga os comparadores internos */
	
	LCD lcd = {&PORTA, 0, 1, 2, 3, 4, 5}; // PORT, RS, EN, D4, D5, D6, D7
    LCD_Init(lcd);
	
	while(1){
		LCD_Clear();
        LCD_Set_Cursor(1,0);
        LCD_putrs("  HELLO WORLD!  ");

		LCD_Set_Cursor(1,1);
		for ( char c = 'A'; c < 'Q'; ++c ) { // Print characters A-P
			LCD_putc(c);
			__delay_ms(300);
		}
		__delay_ms(1000);
	}
	
}
