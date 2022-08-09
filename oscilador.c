/*
 * File: oscilador.c
 * Author: Jorge Cerón
 *
 * Created on 22 de julio de 2022, 11:00 AM
 */

#include <xc.h>
#include "oscilador.h"

void int_osc_MHz(uint8_t freq){
    switch(freq){
        case 1:                         // 1 MHz
            OSCCONbits.IRCF = 0b100;
            break;
        case 2:                         // 2 MHz
            OSCCONbits.IRCF = 0b101;
            break;
        case 4:                         // 4 MHz
            OSCCONbits.IRCF = 0b110;
            break;
        case 8:                         // 8 MHz
            OSCCONbits.IRCF = 0b111;
            break;    
        default:                        // 8 MHz
            OSCCONbits.IRCF = 0b111;
            break;
    }
    OSCCONbits.SCS = 1;                 // Oscilador interno
}
