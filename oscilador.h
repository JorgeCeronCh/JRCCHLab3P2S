/* 
 * File: oscilador.h
 * Author: Jorge Cerón
 * Comments: Prototipo de función para configuración de oscilador interno 
 * (PIC16F887)
 * Revision history: 
 */
 
#ifndef OSCILADOR_H
#define	OSCILADOR_H

#include <xc.h>
#include <stdint.h> 

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void int_osc_MHz (uint8_t freq);
/*
La variable "freq" corresponde a la frecuencia del oscilador interno, donde se
tiene que:
freq == 8 -> IRCF = 111 = 8 MHz
freq == 4 -> IRCF = 110 = 4 MHz (default)
freq == 2 -> IRCF = 101 = 2 MHz
freq == 1 -> IRCF = 100 = 1 MHz
 */
#endif	/* OSCILADOR_H */