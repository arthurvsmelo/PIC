/* 
 * File:   main.c
 * Author: Arthur
 *
 * Created on January 2, 2025, 7:56 PM
 */

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Enter Hexadecimal value)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PLLMODE = DISABLED       // PLL Mode Selection (No PLL used; PLLEN bit is not available)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFCN = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config SOSCSEL = OFF            // SOSC Selection Configuration bits (Digital (SCLKI) mode)
#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration bit (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler bits (1:1)
#pragma config FWPSA = PR32             // Watchdog Timer Prescaler bit (1:32)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bits (WDT and SWDTEN disabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTCMX = WDTCLK          // WDT MUX Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config WDTCLK = SYSCLK          // WDT Clock Source Select bits (WDT uses system clock when active, LPRC while in Sleep mode)

// FPOR
#pragma config BOREN = OFF              // Brown Out Enable bit (Brown-out Reset is Disabled)
#pragma config LPREGEN = OFF            // Low power regulator control (Low Voltage and Low Power Regulator are not available)
#pragma config LPBOREN = DISABLE        // Downside Voltage Protection Enable bit (Low Power BOR is enabled and active when main BOR is inactive)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0xFFFF         // Deadman Timer Interval Low Word (Enter Hexadecimal value)

// FDMTIVTH
#pragma config DMTIVTH = 0xFFFF         // Deadman Timer Interval High Word (Enter Hexadecimal value)

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Deadman Timer Instruction Count Low Word (Enter Hexadecimal value)

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Deadman Timer Instruction Count High Word (Enter Hexadecimal value)

// FMDT
#pragma config DMTDIS = OFF             // Deadman Timer Enable Bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT1
#pragma config ALTCMP1 = ENABLE         // Alternate Comparator 1 Input Enable bit (AC1INC and AC3INC are on RB9)
#pragma config TMPRPIN = OFF            // Tamper Pin Enable bit (TMPRN pin function is disabled)
#pragma config SOSCHP = OFF             // SOSC High Power Enable bit (valid only when SOSCSEL = 1 (Enable SOSC low power mode)
#pragma config ALTI2C1 = ALTI2C1_OFF    // Alternate I2C pin Location (I2C1 Pin mapped to SDA1/SCL1 pins)
#pragma config ALTCMP2 = DISABLE        // Alternate Comparator 2 Input Enable bit (C2INC is on RA4 and C2IND is on RB4)
#pragma config SMB3EN = NORMAL          // SM Bus Enable (Normal SMBus input levels)

#include <xc.h>

void gpioInit(void) {
    // analog
    ANSA = 0x00; // todos os pinos digitais
    ANSB = 0x00; // todos os pinos digitais
    // output
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB14 = 0;
    // input
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    TRISBbits.TRISB13 = 1;
    // output start state
    PORTAbits.RA0 = 0;
    PORTAbits.RA1 = 0;
    PORTAbits.RA4 = 0;
    PORTBbits.RB14 = 0;
}

void interruptInit(void) {
    // set interrupt on-change for the dip switch
    PADCONbits.IOCON = 1;  // enable IOC functionality
    IOCPB = 0x237C;        // set inputs to change on low-to-high
    IOCPDB = 0x237C;       // enagle pulldown resistor
    IFS1bits.IOCIF = 0;    // clear global IOC flag
    IEC1bits.IOCIE = 1;    // enable IOC interrupt
}
/*
void uart1Init(uint16_t baud) {
    // uart LCD
    
}

void uart2Init(uint16_t baud) {
    // uart RADIO
    
}
*/
void __attribute__((__interrupt__, __auto_psv__)) _IOCInterrupt(void) {
    // verifica se houve mudança de estado em PORTB
    if(IOCSTATbits.IOCPBF) {
        // RB2
        if(IOCFBbits.IOCFB2) {
            PORTAbits.RA0 = !LATAbits.LATA0;
            IOCFBbits.IOCFB2 = 0;
        }
        // RB3
        else if(IOCFBbits.IOCFB3) {
            PORTAbits.RA0 = !LATAbits.LATA0;
            IOCFBbits.IOCFB3 = 0;
        }
        // RB4
        else if(IOCFBbits.IOCFB4) {
            PORTAbits.RA1 = !LATAbits.LATA1;
            IOCFBbits.IOCFB4 = 0;
        }
        // RB5
        else if(IOCFBbits.IOCFB5) {
            PORTAbits.RA1 = !LATAbits.LATA1;
            IOCFBbits.IOCFB5 = 0;
        }
        // RB6
        else if(IOCFBbits.IOCFB6) {
            PORTAbits.RA4 = !LATAbits.LATA4;
            IOCFBbits.IOCFB6 = 0;
        }
        // RB8
        else if(IOCFBbits.IOCFB8) {
            PORTAbits.RA4 = !LATAbits.LATA4;
            IOCFBbits.IOCFB8 = 0;
        }
        // RB9
        else if(IOCFBbits.IOCFB9) {
            PORTBbits.RB14 = !LATBbits.LATB14;
            IOCFBbits.IOCFB9 = 0;
        }
        // RB13
        else if(IOCFBbits.IOCFB13) {
            PORTBbits.RB14 = !LATBbits.LATB14;
            IOCFBbits.IOCFB13 = 0;
        }
    }
}

void main() {
    gpioInit();
    interruptInit();
    
    while(1) {
        
    }
}
