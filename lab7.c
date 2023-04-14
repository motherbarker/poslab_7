/*
 * File:   lab7.c
 * Author: Carlos Daniel Valdez Coreas
 * Descripción: PIC lee entrada de potenciómetro en RA0 y 
 * controla un servo en RC2 mediante PWM
 * valores en dos contadores en los puertos C y D.
 * Created on 10 de abril de 2023
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 8000000
#define _tmr0_n 250 //TMR0 load value

/*
 * Constantes
 */



/*
 * Variables
 */
uint8_t tmr_count; // contador de timer
uint8_t width; // Ancho de pulso manual

/*
 * Prototipos de funciones
 */

void setup(void);

/*
 * Interrupciones
 */

void __interrupt() isr (void)
{
    if(PIR1bits.ADIF)     //ADC
    {   
        if (ADCON0bits.CHS == 0)
        {
            // interrupcion
            CCPR1L = (ADRESH>>1)+124;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESL>>7);
        
            PIR1bits.ADIF = 0;
        }
            else if (ADCON0bits.CHS == 1)
            {
            // interrupcion
            CCPR2L = (ADRESH>>1)+124;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = (ADRESL>>7);
            }
            else if (ADCON0bits.CHS == 2)
            {
            width = ADRESH;
            PORTD = width;
            }
        
            PIR1bits.ADIF = 0;
            }
    if(T0IF == 1)
        {  //TMR0
        TMR0 = _tmr0_n;
        tmr_count = tmr_count+25;
        PORTB = tmr_count;
        T0IF = 0;
        }
}


/*
 * Main
 */

void main (void)
{
    setup();
    ADCON0bits.GO = 1;
    while(1)
    {
        if (ADCON0bits.GO == 0)
        {
            if (ADCON0bits.CHS == 0)
                ADCON0bits.CHS = 1;
            else if(ADCON0bits.CHS == 1)
                ADCON0bits.CHS = 2;
            else if(ADCON0bits.CHS == 2)
                ADCON0bits.CHS = 0;
            
            __delay_us(50);
            ADCON0bits.GO = 1;
        }
        
        if (tmr_count > width)
            PORTCbits.RC3 = 0;
        else
            PORTCbits.RC3 = 1;
    }
}

/*
 * Funciones
 */

void setup(void)
{
    // configuración de entradas y salidas
    ANSEL = 0b00000111;
    ANSELH = 0;
    
    TRISA = 0xFF;
    TRISCbits.TRISC3 = 0; //RC3 Output
    TRISB = 0;
    TRISD = 0;
    
    // Configuración del oscilador
    OSCCONbits.IRCF = 0b0111; //8MHz
    OSCCONbits.SCS = 1;
    
    //configuración del TMR0 
    OPTION_REGbits.PS = 0b100;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.T0CS = 0;    
    TMR0 = _tmr0_n;
    
    // Configuración del ADC
    ADCON1bits.ADFM = 0;        //justificación a la izquierda
    ADCON1bits.VCFG0 = 0;       //Vref en VSS y VDD
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.ADCS = 0b10;     //FOSC/32
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    __delay_us(50);
    
    // configuración del PWM
    TRISCbits.TRISC2 = 1;       // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;       // RC1/CCP2 como entrada
    PR2 = 255;                  // configuración del periodo
    
    
    CCP1CONbits.P1M = 0;        // Configuración del modo PWM
    CCP1CONbits.CCP1M = 0b1100;
    
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 0x0f;              //ciclo de trabajo inicial
    CCP1CONbits.DC1B = 0;
    
    CCPR2L = 0x0f;              //ciclo de trabajo inicial
    CCP2CONbits.DC2B0 = 0;
    CCP2CONbits.DC2B1 = 0;
    
    PIR1bits.TMR2IF = 0;        //se toma la bandera
    T2CONbits.T2CKPS = 0b11;    //prescaler a 1:16
    T2CONbits.TMR2ON = 1; 
    
    while(PIR1bits.TMR2IF == 0);  //esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;       //Salida del PWM
    TRISCbits.TRISC1 = 0; 
    
    
    // Configuración de las interrupciones
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1;
    INTCONbits.GIE = 1;
    
    return;
}
