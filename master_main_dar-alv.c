/*
 * File:   master_main_dar-alv.c
 * Author: Genaro Alvarez y Luis Dardón
 *
 * Created on May 24, 2022, 3:58 PM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF           // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF          // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF          // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF             // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF            // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF          // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF           // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF            // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V       // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF            // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>

//DEFINICION DE FRECUENCIA PARA DELAY
#define _XTAL_FREQ 500000          // FRECUENCIA PARA DELAYS (1MHz)

//DEFINICIONES GLOBALES
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 13              // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 80  

//VARIABLES GLOBALES
uint8_t A = 1;                  // VARIABLES PARA ANTIRREBOTE 
uint8_t B = 1;
uint8_t C = 1;
uint8_t D = 1;
uint8_t E = 1;
uint8_t F = 1;
uint8_t pot1_in = 0;            // VARIABLES PARA VALORES DE POTENCIOMETROS
uint8_t pot2_in = 0;
uint8_t pot3_in = 0;
uint8_t pot4_in = 0;
uint8_t estado = 0;
unsigned short CCPR = 0;// Variable para almacenar ancho de pulso al hacer la interpolación lineal
unsigned short CCPRx = 0;


//PROTO FUNCIONES
void setup(void);                   // FUNCION DE SETUP
void data_transfer(uint8_t data);                                               // FUNCION PARA LA TRANSFERENCIA DE DATOS (SPI)
uint8_t EEPROM_read(uint8_t adress);                                            // FUNCION PARA LECTURA DE LA EEPROM
void EEPROM_write(uint8_t adress, uint8_t data);                                // FUNCION PARA ESCRITURA A LA EEPROM

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

//INTERRUPCIONES
void __interrupt() isr(void){
    
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){
            pot1_in = ADRESH;
            CCPR = map(pot1_in, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);              // EJECUCION DE INTERPOLACION PARA ANCHO DE PULSO
            CCPR1L = (uint8_t)(CCPR>>2);                                        // ASIGNAR AL CPR1L LOS 8 BITS MAS SIGNIFICATIVOS
            CCP1CONbits.DC1B = CCPR & 0b11;  
        }

        else if (ADCON0bits.CHS == 1){
            pot2_in = ADRESH;
            CCPRx = map(pot2_in, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);             // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPRx>>2);                                       // Guardamos los 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPRx & 0b01;
            CCP2CONbits.DC2B1 = CCPRx & 0b10; 
        }

        else if (ADCON0bits.CHS == 2){
            pot3_in = ADRESH;
        }

        else if (ADCON0bits.CHS == 3){
            pot4_in = ADRESH;
        }
        PIR1bits.ADIF = 0;  
    }
    
    
}

void main(void) {
    //EJECUCION CONFIG
    setup();
    
    //LOOP PRINCIPAL
    while (1){
        if (ADCON0bits.GO == 0){                // REVISAR SI EL ADC ESTA ENCENDIDO
            if (ADCON0bits.CHS == 0){           // REVISAR SI SE ENCUENTRA EN CANAL ANALOGICO 0
                ADCON0bits.CHS = 1;             // CAMBIO A CANAL ANALOGICO 1
                __delay_us(40);                 // TIEMPO DE ESTABILIZACION
            }
            else if (ADCON0bits.CHS == 1){      // REVISAR SI SE ENCUENTRA EN CANAL ANALOGICO 1
                ADCON0bits.CHS = 2;             // CAMBIO A CANAL ANALOGICO 2
                __delay_us(40);                 // TIEMPO DE ESTABILIZACION
            }
            else if (ADCON0bits.CHS == 2){      // REVISAR SI SE ENCUENTRA EN CANAL ANALOGICO 2
                ADCON0bits.CHS = 3;             // CAMBIO A CANAL ANALOGICO 0
                __delay_us(40);                 // TIEMPO DE ESTABILIZACION
            }
            else if (ADCON0bits.CHS == 3){      // REVISAR SI SE ENCUENTRA EN CANAL ANALOGICO 2
                ADCON0bits.CHS = 0;             // CAMBIO A CANAL ANALOGICO 0
                __delay_us(40);                 // TIEMPO DE ESTABILIZACION
            }
            __delay_us(40);                     // TIEMPO DE ESTABILIZACION
            ADCON0bits.GO = 1;                  // INICIADO DE CONVERSION
        }
        

        PORTAbits.RA6 = 0;
        if (PORTAbits.RA6 == 0){
            PORTDbits.RD1 = 0;
            __delay_ms(50);
            PORTDbits.RD0 = 1;
            __delay_ms(50);
            SSPBUF = pot3_in;
            while(!SSPSTATbits.BF);
            //PORTB = pot1_in;
            
            PORTDbits.RD1 = 1;
            __delay_ms(50);
            PORTDbits.RD0 = 0;
            __delay_ms(50);
            SSPBUF = pot4_in;
            while(!SSPSTATbits.BF);
            //PORTB = pot2_in;
        }
        __delay_ms(50);
    }
    
    return;
}

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0b00001111;             // PORTA AN0, AN1 y AN2 COMO ANALOGICO, RESTO COMO DIGITALES
    ANSELH = 0;                     // DEMAS PUERTOS COMO DIGITALES

    TRISA = 0b00001111;             // PORTA AN0 AN1 y AN2 COMO ENTRADA, RESTO COMO SALIDA
    TRISB = 0;
    TRISD = 0;
    PORTA = 0;                      // LIMPIEZA DEL PORTA
    PORTB = 0;
    PORTD = 0;                      // LIMPIEZA DEL PORTD

    TRISC = 0b00010000;             // ENTRADA DE DATOS COMO ENTRADA, SINCRONIZADOR DE RELOJ Y SALIDA DE DATOS COMO SALIDA
    PORTC = 0;                      // LIMPIEZA DE PORTB

    //SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;       // SPI MAESTRO ACTIVADO | FOSC/4 (250kbits/s)
    SSPCONbits.CKP = 0;             // RELOJ INACTIVO EN 0
    SSPCONbits.SSPEN = 1;           // HABILITACION DE PINES DE SPI
    //SSPSTAT <7:6>
    SSPSTATbits.CKE = 1;            // ENVIO DE DATO EN FLANCO DE SUBIDA
    SSPSTATbits.SMP = 1;            // DATO AL FINAL DE PULSO DE RELOJ
    SSPBUF = 0x00;                    // VALOR INICIAL ENVIADO AL BUFFER
    
    //CONFIG DE INTERRUPCIONES
    INTCONbits.GIE = 1;             // HABILITAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;            // HABILITAR INTERRUPCIONES EN PERIFERICOS
    PIR1bits.ADIF = 0;              // LIMPIEZA DE BANDERA DE INTERRUPCION DE ADC
    PIE1bits.ADIE = 1;              // HABILITAR INTERRUPCION DE ADC
    
    //OSCCONFIC
    OSCCONbits.IRCF = 0b0011;       // FRECUENCIA DE OSCILADOR INTERNO (500kHz)
    OSCCONbits.SCS  = 1;            // RELOJ INTERNO

    //ADC CONFIG
    ADCON0bits.ADCS = 0b00;         // FOSC/2
    ADCON1bits.VCFG0 = 0;           // USO DE VDD COMO VOLTAJE DE REFERENCIA INTERNO
    ADCON1bits.VCFG1 = 0;           // USO DE VSS COMO VOLTAJE DE REFERENCIA INTERNO

    ADCON0bits.CHS = 0b0000;        // SELECCION DE PORTA PIN0 (AN0) COMO ENTRADA DE ADC
    ADCON1bits.ADFM = 0;            // FORMATO DE BITS JUSTIFICADOS A LA IZQUIERDA
    ADCON0bits.ADON = 1;            // HABILITACION DE MODULO DE ADC
    __delay_us(320);                // TIEMPO DE LECTURA

    //PWM CONFIG
    TRISCbits.TRISC2 = 1;           // CCP1 COMO ENTRADA (SALIDA DESABILITADA)
    TRISCbits.TRISC1 = 1;           // CCP2 COMO ENTRADA (SALIDA DESABILITADA)
    PR2 = 156;                      // PERIODO DE TMR2 EN 20mS

    //CCP CONFIG
    CCP1CON = 0;                    // CCP1 APAGADO
    CCP2CON = 0;                    // CCP2 APAGADO
    CCP1CONbits.P1M = 0;            // CAMBIO DE MODO A "SINGLE OUTPUT"
    CCP1CONbits.CCP1M = 0b1100;     // PWM PARA CCP1
    CCP2CONbits.CCP2M = 0b1100;     // PWM PARA CCP2

    CCPR1L = 155>>2;                // CONFIGURACION DE ANCHO DE PULSO PARA CCP1 Y CCP2
    CCP1CONbits.DC1B = 155 & 0b11;  //
    CCPR2L = 155>>2;                //
    CCP2CONbits.DC2B0 = 155 & 0b01; //
    CCP2CONbits.DC2B1 = 155 & 0b10; //


    T2CONbits.T2CKPS = 0b11;        // RPESCALER DEL TMR2 EN 1:16
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2
    T2CONbits.TMR2ON = 1;           // TMR2 ENCENDIDO
    while(!PIR1bits.TMR2IF);        // CICLO INDIVIDUAL DE TMR2 EN ESPERA
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2

    TRISCbits.TRISC1 = 0;           // HABILITAR SALIDA DE PWM EN RC1
    TRISCbits.TRISC2 = 0;           // HABILITAR SALIDA DE PWM EN RC2
    
    return;
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

void data_transfer(uint8_t data){

    __delay_ms(50);
    SSPBUF = data;
    while(!SSPSTATbits.BF);
    __delay_ms(50);

    return;
}

// LECTURA DE LA EEPROM
uint8_t EEPROM_read(uint8_t adress){
    EEADR = adress;
    EECON1bits.EEPGD = 0;           // LEIDO DE LA EEPROM
    EECON1bits.RD = 1;              // OBTENER DATO DE LA EEPROM
    return EEDAT;                   // REGRESAR VALOR DE LA EEPROM
}

// ESCRITURA A LA EEPROM
void EEPROM_write(uint8_t adress, uint8_t data){
    EEADR = adress;
    EEDAT = data;
    EECON1bits.EEPGD = 0;           // ESCRIBIR A LA EEPROM
    EECON1bits.WREN = 1;            // HABILITAR ESCRITURA EN EEPROM

    INTCONbits.GIE = 0;             // DESHABILITAR INTERRUPCIONES GLOBALES
    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;              // INICIO DE ESCRITURA

    EECON1bits.WREN = 0;            // DESHABILITAR ESCRITURA EN EEPROM
    INTCONbits.RBIF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCIONES EN PUERTO B
    INTCONbits.GIE = 1;             // HABILITAR INTERRUPCIONES GLOBALES
}