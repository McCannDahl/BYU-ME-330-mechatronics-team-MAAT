/*
 * File:   main.c
 * Author: mccda
 *
 * Created on October 10, 2017, 1:18 PM
 */

#pragma config ICS = PGx3 //this allows debugging
#pragma config FNOSC = FRCDIV // select 8MHz oscillator with postscaler

#include "xc.h"
#include "libpic30.h" //include library for _delay_ms() function)

int main(void) {
    
    _RCDIV = 0b010; // postscale oscillator (divide-by-4))

    Nop();  //do nothing cycle for one-instruction
    
    return 0;
}
