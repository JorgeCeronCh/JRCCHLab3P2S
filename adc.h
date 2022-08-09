/* 
 * File: adc.h
 * Author: Jorge Cerón
 * Comments: Prototipo de funciones para configuración de inicialización, 
 * conversión y lectura del ADC (PIC16F887)
 * Revision history: 
 */
 
#ifndef ADC_H
#define	ADC_H

#include <xc.h>

void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
/*
La variable adc_cs tiene como posibles configuraciones de ADCS:
adc_cs == 0 -> ADCS = 00 = FOSC/2
adc_cs == 1 -> ADCS = 01 = FOSC/8 (default)
adc_cs == 2 -> ADCS = 10 = FOSC/32
adc_cs == 3 -> ADCS = 11 = F_RC (reloj derivado de un oscilador interno dedicado
                                 a 500 kHz máx.)

La variable vref_plus tiene como posibles configuraciones de VCFG0:
vref_plus == 0 -> VCFG0 = 0 = V_DD (default)
vref_plus == 1 -> VCFG0 = 1 = V_REF+ PIN

La variable vref_minus tiene como posibles configuraciones de VCFG1 
vref_minus == 0 -> VCFG1 = 0 = V_SS (default)
vref_minus == 1 -> VCFG1 = 1 = V_REF- PIN
 */

void adc_start(uint8_t channel);
/*
La variable channel tiene como posibles configuraciones de CHS segun sea el
número que coloquen de 0 a 15.
channel == 0 -> CHS = 0000 = AN0
channel == 1 -> CHS = 0001 = AN1
channel == 2 -> CHS = 0010 = AN2
channel == 3 -> CHS = 0011 = AN3
channel == 4 -> CHS = 0100 = AN4
channel == 5 -> CHS = 0101 = AN5
channel == 6 -> CHS = 0110 = AN6
channel == 7 -> CHS = 0111 = AN7
channel == 8 -> CHS = 1000 = AN8
channel == 9 -> CHS = 1001 = AN9
channel == 10 -> CHS = 1010 = AN10
channel == 11 -> CHS = 1011 = AN11
channel == 12 -> CHS = 1100 = AN12
channel == 13 -> CHS = 1101 = AN13
channel == 14 -> CHS = 1110 = CV_REF
channel == 15 -> CHS = 1111 = Referencia fija (Voltaje fijo de 0.6 V) 
 */
uint16_t adc_read(void);
/*
Se encarga de regresar el valor del ADC y de limpiar su respectiva
bandera.
 */
#endif	/* ADC_H */