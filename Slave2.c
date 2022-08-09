/*
 * File:   Slave2.c
 * Author: jorge
 *
 * Created on 5 de agosto de 2022, 09:51 AM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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
#include <stdio.h>
#include "spi.h"
#include "oscilador.h"
#include "adc.h"
#define _XTAL_FREQ 1000000


/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t ADRESHS = 0;
uint8_t ADRESLS = 0;
uint8_t ENVIO = 0;
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() SPI_slave(void){
    if(PIR1bits.ADIF){                      
        if(ADCON0bits.CHS == 0){                // RA0
            ADRESHS = (adc_read()>>2) & 255;    // ADRESH
            ADRESLS = adc_read()& 3;            // ADRESL
        }
        PIR1bits.ADIF = 0;                      // Limpair bandera ADC
    }
    if (PIR1bits.SSPIF){
        ENVIO = spiRead();                      // Se lee lo que se desea enviar al Master
        if (ENVIO == 72){
            spiWrite(ADRESHS);                  // "H" -> Envio de ADRESH
        } else if (ENVIO == 76) {
            spiWrite(ADRESLS);                  // "L" -> Envio de ADRESL
        }
        PIR1bits.SSPIF = 0;
    }
    return;
}

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);

void main(void) {
    int_osc_MHz(1);
    adc_init(0, 0, 0);
    setup(); 
    while(1){
        adc_start(0);
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){       
    // Configuración de puertos
    ANSEL = 0b00000001;         // RA0 como analógica, demás como I/O digitales
    ANSELH = 0b00000000;        // I/O digitales
    TRISAbits.TRISA5 = 1;       // SS como entrada
    TRISAbits.TRISA0 = 1;       // RA0 como entrada
    PORTA = 0b00000000;         // Limpiar PORTA
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
    // Configuración de interrupciones
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    PIR1bits.SSPIF = 0;         // Limpiar bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitar interrupciones de SPI
    INTCONbits.PEIE = 1;        // Habilitar interrupciones de perifericos
    return;
}
