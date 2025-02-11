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

#define FCY 4000000UL // fosc / 2
#define _XTAL_FREQ 8000000UL

#include <xc.h>
#include <libpic30.h>

void gpioInit(void) 
{
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

void interruptInit(void) 
{
    // set interrupt on-change for the dip switch
    PADCONbits.IOCON = 1;  // enable IOC functionality
    IOCPB = 0x237C;        // set inputs to change on low-to-high
    IOCPDB = 0x237C;       // enable pulldown resistor
    IFS1bits.IOCIF = 0;    // clear global IOC flag
    IEC1bits.IOCIE = 1;    // enable IOC interrupt
}

void peripheral_pin_init(void) 
{
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    RPINR18bits.U1RXR = 0;    
    RPOR0bits.RP1R    = 3;   
    RPINR19bits.U2RXR = 10;
    RPOR5bits.RP11R   = 5;   
    __builtin_write_OSCCONL(OSCCON | 0x40);
}

void uart1_init(uint16_t baud) 
{
    U1MODEbits.UARTEN = 0;
 
    U1MODEbits.USIDL = 1; 
    U1MODEbits.IREN = 0;
    U1MODEbits.RTSMD = 1;
   
    U1MODEbits.UEN0 = 0;
    U1MODEbits.UEN1 = 0;
    
    U1MODEbits.WAKE = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 1;
    
    U1BRG = 103;
    
    U1MODEbits.PDSEL0 = 0;
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;
    
    U1STAbits.UTXINV = 0; 
    U1STAbits.UTXBRK = 0;

    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;
    
    U1MODEbits.UARTEN = 1;
    IEC0bits.U1RXIE = 0;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
}

void uart2_init(uint16_t baud) 
{
    // f?rmula do baud rate:
    // FCY = F_OSCILADOR/2
    // U2BRG = (FCY / (4 * baudrate)) - 1
    // U2BRG = (4000000 / (4 * 62500)) - 1 = 15
    U2MODEbits.UARTEN = 0; // Disable UART  
 
    U2MODEbits.USIDL = 1; //disable quando idle 
    U2MODEbits.IREN = 0; //disable InfraRed encoder
    U2MODEbits.RTSMD = 1; //desabilita ready-to-send
   
    U2MODEbits.UEN0 = 0; //ready-to-send e ready-to-receive desabilitados dos pinos
    U2MODEbits.UEN1 = 0;
    
    U2MODEbits.WAKE = 0; //desabilita wake no rx
    U2MODEbits.LPBACK = 0; //desabilita loopback(?)
    U2MODEbits.ABAUD = 0; //desabilita auto-baud
    U2MODEbits.URXINV = 0; //sinal rx idle com '0'
    U2MODEbits.BRGH = 1; //fast baud rate; baud = FCY/(4*(U2BRG + 1))
    
    U2BRG = (uint16_t)((FCY / (4 * baud)) - 1);
    
    //define formato dos dados
    U2MODEbits.PDSEL0 = 0; // sem bit de paridade
    U2MODEbits.PDSEL = 0; //8bits
    U2MODEbits.STSEL = 0; //1 stop bit

    //seta interrupt qnd ultimo bit ? enviado do TSR, transmiss?o completa
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0;
    
    U2STAbits.UTXINV = 0; //idle state ? 0 
    U2STAbits.UTXBRK = 0; //desabilita sync break
    
    //interrupt da recepcao qnd RSR ? transmitido para buffer.
    U2STAbits.URXISEL0 = 0;
    U2STAbits.URXISEL1 = 0;
    
    U2MODEbits.UARTEN = 1; // Enable UART
    IEC1bits.U2RXIE = 1;
    U2STAbits.UTXEN = 1; //enable transmit
    U2STAbits.URXEN = 1; //enable receive
}

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void) 
{
    uint8_t char_recebido;
    if(IEC0bits.U1RXIE){    
        if (U1STAbits.OERR) 
        { 
            U1STAbits.OERR = 0;  // Limpa erro de overrun
        }
        
        if (U1STAbits.FERR) 
        { 
            volatile uint8_t dummy = U1RXREG;  // Lê e descarta o byte corrompido
        }
        else 
        {
            char_recebido = U1RXREG;
            if(char_recebido == 0x52)
            {
                // muda estado do led assim que receber um byte
                LATAbits.LATA0 = !LATAbits.LATA0;
            }
        }
    }
    IFS0bits.U1RXIF = 0;
}

// fun??o de interrup??o UART2_RX
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void) 
{
    uint8_t char_recebido;
    if(IEC1bits.U2RXIE){    
        if (U2STAbits.OERR) 
        { 
            U2STAbits.OERR = 0;  // Limpa erro de overrun
        }
        
        if (U2STAbits.FERR) 
        { 
            volatile uint8_t dummy = U2RXREG;  // Lê e descarta o byte corrompido
        }
        else 
        {
            char_recebido = U2RXREG;
            if(char_recebido == 0x7E)
            {
                // muda estado do led assim que receber um byte
                LATAbits.LATA1 = !LATAbits.LATA1;
            }
        }
    }
    // reseta flag de interrup??o
    IFS1bits.U2RXIF = 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _IOCInterrupt(void) 
{
    // verifica se houve mudan?a de estado em PORTB
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

void main() 
{
    gpioInit();
    peripheral_pin_init();
    uart1_init(9600);
    uart2_init(62500);
    interruptInit();
    uint8_t tx1_char_to_send[] = {0x11, 0x52, 0x99, 0x2D};
    uint8_t tx2_char_to_send[] = {0x32, 0x63, 0xA7, 0x7E};
    uint8_t index;
    
    while(1) {
        for (index = 0; index < 4; index++)
        {
            U1TXREG = tx1_char_to_send[index];
            __delay_ms(50);
            U2TXREG = tx2_char_to_send[index];
            __delay_ms(50);
        }
    }
}
