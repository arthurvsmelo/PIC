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

void pin_peripheral_select(void){
    //associa pinos à funcionalidade uart, UART1 e UART2
    RPINR18bits.U1RXR = 0x0000;    // RB0  -> U1RX: UART1
    // ver datasheet na página 228 n contrast to inputs, the outputs of the Peripheral Pin Select options are mapped on the basis of the pin...
    RPINR19bits.U2RXR = 0x000A;    // RB10 -> U2RX: UART2
    RPOR5bits.RP11R   = 0x0003;    // RB11 -> U2TX: UART2  
}

void uart1_init(uint16_t baud){
    // fórmula do baud rate:
    // FCY = F_OSCILADOR/2
    // U1BRG = (FCY / (4 * baudrate)) - 1
    // U1BRG = (4000000 / (4 * 62500)) - 1 = 15
    
    U1BRG = (uint16_t)((FCY / (4 * baud)) - 1);
 
    U1MODEbits.USIDL = 1; //disable quando idle 
    U1MODEbits.IREN = 0; //disable InfraRed encoder
    U1MODEbits.RTSMD = 1; //disabilita ready-to-send
   
    U1MODEbits.UEN0 = 0; //ready-to-sendo e ready-to-receive desabilitados dos pinos
    U1MODEbits.UEN1 = 0;
    
    U1MODEbits.WAKE = 0; //desabilita wake no rx
    U1MODEbits.LPBACK = 0; //desabilita loopback(?)
    U1MODEbits.ABAUD = 0; //desabilita auto-baud
    U1MODEbits.URXINV = 0; //sinal rx idle com '0'
    U1MODEbits.BRGH = 1; //fast baud rate; baud = FCY/(4*(U1BRG + 1))
    
    //define formato dos dados
    U1MODEbits.PDSEL0 = 0; // sem bit de paridade
    U1MODEbits.PDSEL = 0; //8bits
    U1MODEbits.STSEL = 0; //1 stop bit

    //seta interrupt qnd ultimo bit é enviado do TSR, transmissão completa
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXISEL1 = 0;
    
    U1STAbits.UTXINV = 0; //idle state é 0 
    U1STAbits.UTXBRK = 0; //desabilita sync break
    
    //interrupt da recepcao qnd RSR é transmitido para buffer.
    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;
    
    U1MODEbits.UARTEN = 1; // Enable UART
    IEC0bits.U1RXIE = 1;
    U1STAbits.UTXEN = 1; //enable transmit
    U1STAbits.URXEN = 1; //enable receive
}

void uart2_init(uint16_t baud){
    // fórmula do baud rate:
    // FCY = F_OSCILADOR/2
    // U2BRG = (FCY / (4 * baudrate)) - 1
    // U2BRG = (4000000 / (4 * 62500)) - 1 = 15
    
    U2BRG = (uint16_t)((FCY / (4 * baud)) - 1);  
 
    U2MODEbits.USIDL = 1; //disable quando idle 
    U2MODEbits.IREN = 0; //disable InfraRed encoder
    U2MODEbits.RTSMD = 1; //disabilita ready-to-send
   
    U2MODEbits.UEN0 = 0; //ready-to-send e ready-to-receive desabilitados dos pinos
    U2MODEbits.UEN1 = 0;
    
    U2MODEbits.WAKE = 0; //desabilita wake no rx
    U2MODEbits.LPBACK = 0; //desabilita loopback(?)
    U2MODEbits.ABAUD = 0; //desabilita auto-baud
    U2MODEbits.URXINV = 0; //sinal rx idle com '0'
    U2MODEbits.BRGH = 1; //fast baud rate; baud = FCY/(4*(U2BRG + 1))
    
    //define formato dos dados
    U2MODEbits.PDSEL0 = 0; // sem bit de paridade
    U2MODEbits.PDSEL = 0; //8bits
    U2MODEbits.STSEL = 0; //1 stop bit

    //seta interrupt qnd ultimo bit é enviado do TSR, transmissão completa
    U2STAbits.UTXISEL0 = 1;
    U2STAbits.UTXISEL1 = 0;
    
    U2STAbits.UTXINV = 0; //idle state é 0 
    U2STAbits.UTXBRK = 0; //desabilita sync break
    
    //interrupt da recepcao qnd RSR é transmitido para buffer.
    U2STAbits.URXISEL0 = 0;
    U2STAbits.URXISEL1 = 0;
    
    U2MODEbits.UARTEN = 1; // Enable UART
    IEC1bits.U2RXIE = 1;
    U2STAbits.UTXEN = 1; //enable transmit
    U2STAbits.URXEN = 1; //enable receive
}

// função de interrupção UART1_RX
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void) {
    
    if(IFS0bits.U1RXIF && IEC0bits.U1RXIE){    
        // caso dê erros na comuncação, reseta UART
        if(U1STAbits.OERR || U1STAbits.FERR) {
            
            U1STAbits.OERR = 0;
            U1STAbits.FERR = 0;
            U1MODEbits.UARTEN = 0;
            U1MODEbits.UARTEN = 1;
        }
        else {
            // armazena mensagem na varíavel
            char_recebido = U1RXREG;
            // trata a mensagem
        }   
    }
    // reseta flag de interrupção
    IFS0bits.U1RXIF = 0;
}

// função de interrupção UART1_TX
void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    U1TXREG = 'a'; // Transmit one character
}

// função de interrupção UART2_RX
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void) {
    
    if(IFS0bits.U2RXIF && IEC0bits.U2RXIE){    
        // caso dê erros na comuncação, reseta UART
        if(U2STAbits.OERR || U2STAbits.FERR) {
            
            U2STAbits.OERR = 0;
            U2STAbits.FERR = 0;
            U2MODEbits.UARTEN = 0;
            U2MODEbits.UARTEN = 1;
        }
        else {
            // armazena mensagem na varíavel
            char_recebido = U2RXREG;
            // trata a mensagem
        }   
    }
    // reseta flag de interrupção
    IFS0bits.U2RXIF = 0;
}

// função de interrupção UART2_TX
void __attribute__((__interrupt__)) _U2TXInterrupt(void)
{
    IFS0bits.U2TXIF = 0; // Clear TX Interrupt flag
    U2TXREG = 'a'; // Transmit one character
}

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
