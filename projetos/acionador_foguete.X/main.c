/*
 * File:   main.c
 * Author: Arthur Melo
 *
 * Created on October 2, 2024, 7:56 PM
 */

/* CONFIGURTION BITS */
/* Oscilador cristal externo, 
   Watchdog ativo,
   Powerup timer desativado,
   Code Protection desativado */
#pragma config FOSC = XT, WDTE = ON, PWRTE = OFF, CP = OFF

#include <xc.h>
#include <pic16f84.h>

#define _XTAL_FREQ    4000000
#define RELAY_SIGNAL  PORTBbits.RB3
#define ENABLE_SWITCH PORTBbits.RB1
#define DEC_BUTTON    PORTBbits.RB0
#define TRIGGER       PORTBbits.RB2
#define BCD_A         PORTAbits.RA0
#define BCD_B         PORTAbits.RA1
#define BCD_C         PORTAbits.RA2
#define BCD_D         PORTAbits.RA3
#define BUZZER        PORTBbits.RB7

uint8_t counter_value = 5;
uint8_t counter = 0;

void __interrupt() isr(){
    /* não é necessário verificar a fonte de interrupção pois 
       só está configurado interrupção no pino RB0/INT0 */
    if(INTCONbits.INTE && INTCONbits.INTF){
        INTCONbits.GIE = 0;
        counter_value--;
        if(counter_value < 1) {
            counter_value = 9;
        }
        INTCONbits.INTF = 0;
        INTCONbits.GIE = 1;
    }
}

void showNumber(uint8_t n) {
    BCD_A = n & 0x01;
    BCD_B = (n >> 1) & 0x01;
    BCD_C = (n >> 2) & 0x01;
    BCD_D = (n >> 3) & 0x01;
}

void triggerCountdown(void) {
    do {
        counter = counter_value;
		INTCONbits.GIE = 0;     /* desativa as interrupções */
        showNumber(counter);
        
		while(TRIGGER) {
			showNumber(counter);
			BUZZER = 1;
			__delay_ms(200);
			BUZZER = 0;
			__delay_ms(800);
			counter--;
			
			if (counter < 1 && TRIGGER) {
				showNumber(counter);
				RELAY_SIGNAL = 0;           /* ativa o relé */
				BUZZER = 1;                 /* emite som de ativação */
				CLRWDT();
				while(TRIGGER) {
					RELAY_SIGNAL = 0;/* enquanto o botão estiver pressionado, */
					BUZZER = 1;      /* o relé e o som estarão ativos */
					CLRWDT();
				}
				RELAY_SIGNAL = 1;      /* ao soltar o botão, desliga o relé */
				BUZZER = 0;
				return;
			}
			CLRWDT();
		}
		CLRWDT();
    }
	while(ENABLE_SWITCH);
    
	INTCONbits.GIE = 1;         /* ativa novamente as interrupções */
	CLRWDT();
}

void main(void){
    TRISA = 0b00000000;            /* todos os pinos em output */
    TRISB = 0b00000111;            /* RB0, RB1 e RB2 input e RB3, RB7 output */
    OPTION_REGbits.nRBPU = 1;      /* desativa resistor de pullup */
    OPTION_REGbits.INTEDG = 1;     /* rising edge */
	OPTION_REGbits.T0CS = 0;       /* timer0 clock source: clk interno */
    INTCONbits.INTE = 1;           /* habilita interrupcao externa */
    INTCONbits.GIE = 1;            /* habilita interrupcoes */
    INTCONbits.INTF = 0;           /* limpa flag de interrupcao */
    RELAY_SIGNAL = 1;              /* relé é ativo em 0 */
    /* sinal sonoro de inicialização */
	BUZZER = 1;
	__delay_ms(100);
	BUZZER = 0;
	__delay_ms(100);
	BUZZER = 1;
	__delay_ms(100);
	BUZZER = 0;
	__delay_ms(100);
	BUZZER = 1;
	__delay_ms(100);
	BUZZER = 0;
	__delay_ms(100);
	BUZZER = 1;
	__delay_ms(800);
	BUZZER = 0;
    
    while(1) {
        /* mostra o valor da contagem no display 7 seg */
        showNumber(counter_value);
        if(ENABLE_SWITCH) {
            BUZZER = 1;
            __delay_ms(100);
            BUZZER = 0;
            __delay_ms(100);
            BUZZER = 1;
            __delay_ms(100);
            BUZZER = 0;
            __delay_ms(100);
            BUZZER = 1;
            __delay_ms(100);
            BUZZER = 0;
            __delay_ms(100);
            triggerCountdown();
        }
        __delay_ms(50);
       CLRWDT();
    }
}
