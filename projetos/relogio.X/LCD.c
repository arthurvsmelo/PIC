#include <xc.h>
#include <stdio.h>
#include "LCD.h"

/* LCD COMMANDS */
#define CLEAR                0x01
#define HOME                 0x02

#define ENTRY_MODE_BASE      0x04
#define INCREMENT_MASK       0x02 /* 1: increment | 0: decrement */
#define SHIFT_ON_MASK        0x01 /* 1: display shift | 0: display not shift */

#define DISPLAY_CONTROL_BASE 0x08 
#define DISPLAY_ON_MASK      0x04 /* 1: on | 0: off */
#define CURSOR_ON_MASK       0x02 /* 1: on | 0: off */
#define BLINK_ON_MASK        0x01 /* 1: on | 0: off */

#define SHIFT_CONTROL_BASE   0x10
#define SHIFT_CURSOR_MASK    0x08 /* 1: display shift | 0: cursor shift */   
#define RIGHT_LEFT_MASK      0x04 /* 1: shift to right | 0: shift to left */

/* END LCD COMMANDS */

static unsigned char _display_control_state;

/* INTERNAL FUNCTIONS */
static void pulse_enable() {
    EN = 1;
    __delay_us(50);
    EN = 0;
}

static void lcd_wait() {
    TRISB = 0xFF; /* INPUT */
    RS = 0; /* INSTRUCTION MODE */
    RW = 1; /* READ MODE */
    while(1) {
        pulse_enable();
        if (D7 == 0) {
            break;
        }
    }
    RW = 0; /* back to write mode */
    TRISB = 0x00; /* OUTPUT */
}

static void lcd_send(char byte, uint8_t mode) {
    TRISB = 0x00; /* OUTPUT */
    RW = 0; /* WRITE MODE */
    if (mode == 0) {
        RS = 0; /* COMMAND */
    }
    else if (mode == 1){
        RS = 1; /* DATA */
    }
    PORTB = byte;
    pulse_enable();
}

static void lcd_send_command(char cmd) {
    lcd_send(cmd, 0);
    lcd_wait();
}

static void lcd_send_data(char data) {
    lcd_send(data, 1);
    lcd_wait();
}

/* API FUNCTIONS */
void lcd_init() {
    TRISB = 0x00;            /* D7~D0 output */
    TRISDbits.TRISD5 = 0;    /* RS output */
    TRISDbits.TRISD6 = 0;    /* RW output */
    TRISDbits.TRISD7 = 0;    /* EN output */
    TRISAbits.TRISA2 = 0;    /* BACKLIGHT output */
    __delay_ms(20);          /* for power stabilization */
    lcd_send(0x38, 0);          /* 8bit mode, 2 rows, 5x8 dots; first send */
    __delay_ms(5);
    lcd_send(0x38, 0);          /* second send */
    __delay_us(150);
    lcd_send(0x38, 0);          /* third send; now, busy flag can be checked */
    lcd_send_command(0x38);  /* finally sets the function set */
    lcd_send_command(0x0C);  /* display control: display on, cursor and blinking off */
    lcd_send_command(CLEAR); /* clear display */
    lcd_send_command(0x06);  /* entry mode set: increment cursor, no shiftting */
    _display_control_state = 0x0C;
    __delay_us(100);
}

void lcd_puts(const char *s) {
    while(*s != '\0') {
        lcd_send_data(*s);
        s++;
    }
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    if (row > 1) row = 0;
    if (col > 15) col = 0;
    
    if (row == 0) {
        lcd_send_command(0x80 + col);
    }
    else {
        lcd_send_command(0xC0 + col);
    }
}

void lcd_clear() {
    lcd_send_command(CLEAR);
}

void lcd_home() {
    lcd_send_command(HOME);
}

void lcd_display_on() {
    _display_control_state |= DISPLAY_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void lcd_display_off() {
    _display_control_state &= ~DISPLAY_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void lcd_cursor_on() {
    _display_control_state |= CURSOR_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void lcd_cursor_off() {
    _display_control_state &= ~CURSOR_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void lcd_blink_on() {
    _display_control_state |= BLINK_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void lcd_blink_off() {
    _display_control_state &= ~BLINK_ON_MASK;
    lcd_send_command(DISPLAY_CONTROL_BASE | _display_control_state);
}

void display_shift_right() {
    lcd_send_command(SHIFT_CONTROL_BASE | 0x0C);
}

void display_shift_left() {
    lcd_send_command(SHIFT_CONTROL_BASE | 0x08);
}

void cursor_shift_right() {
    lcd_send_command(SHIFT_CONTROL_BASE | 0x04);
}

void cursor_shift_left() {
    lcd_send_command(SHIFT_CONTROL_BASE);
}

void lcd_backlight(char state) {
    if (state == 1) {
        BACKLIGHT = 1; /* turn on */
    }
    else {
        BACKLIGHT = 0; /* turn off */
    }
}

void lcd_print_float(float number, uint8_t precision) {
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%.*f", precision, number);
    lcd_puts(buffer);
}

void lcd_print_int(int number, uint8_t lenght) {
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%*d", lenght, number);
    lcd_puts(buffer);
}

