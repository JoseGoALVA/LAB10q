
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define LEN_MSG 9               // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor m?ximo de entrada del potenciometro
#define OUT_MIN 16               // Valor minimo de ancho de pulso de se?al PWM
#define OUT_MAX 80             // Valor m?ximo de ancho de pulso de se?al PWM

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char letras;
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolaci?n lineal
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void texto(char mensaje);
char RESULTADO();
void txt_unido();
char X;

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main (void)
{
    setup ();
    
    //---------------------- Loop principal ------------------
    while (1)
    {
        txt_unido("\r ESCOJA SU OPCION DESEADA? \r");
        txt_unido("1. POTENCIOMETRO \r");
        txt_unido("2. ASCII \r");
        
        while(PIR1bits.RCIF == 0);
        X = RCREG;
        
        switch(X)
        {
            case('1'):
                CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B0 = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                X = ADRESH;
                TXREG = X;
                txt_unido(ADRESH);
                PORTA = ADRESH;
  
                break;
            case('2'):
                txt_unido("ASCII A MOSTRAR:");
                while(PIR1bits.RCIF == 0);
                X = RCREG;
                TXREG = X;
                PORTB = X;
                txt_unido("\r\r");
                break;
        }

    }
        X = 0;
        return;
}

void texto(char mensaje){
    while (TXSTAbits.TRMT == 0);
    TXREG = mensaje;
}

void txt_unido (char *str){
    while (*str != '\0'){
        texto(*str);
        str++;
    }
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;                 // I/O digitales
    
    TRISB = 0;
    PORTB = 0;                  // PORTD como salida
    TRISA = 0;
    PORTA = 0;                  // PORTD como salida
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicaci n ascincrona (full-duplex)?
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicaci n?
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuraciones de interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepci n?
    
    // Configuraci?n ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b00;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuraci?n PWM
    TRISCbits.TRISC3 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP2
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 30;                  // periodo de 2ms
    
    // Configuraci?n CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP2CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP2CONbits.CCP2M = 0b1100;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM

    
    CCPR1L = 16>>2;
    CCPR2L = 16>>2;
    CCP1CONbits.DC1B0 = 16 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    CCP2CONbits.DC2B0 = 16 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM

    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}