/*
Hardware watchdog timer based on PIC16F1825

OpenTransat
http://opentransat.com
http://github.com/opentransat
http://fb.com/opentransat

Released under the Creative Commons Attribution ShareAlike 4.0 International License
https://creativecommons.org/licenses/by-sa/4.0/
*/

#define _XTAL_FREQ 31000UL
#include "config.h"
#include <xc.h>

#define NAVRESET PORTCbits.RC1
#define GPIO PORTCbits.RC2
#define HEARTBEAT PORTCbits.RC3
#define LED PORTCbits.RC5

#define TMR1_MAX 7750 //timer1 increment after 4 seconds: 31000 / 4 / 8 * #seconds

void init_io() {
    OSCCONbits.IRCF = 0b0000; //31kHz oscillator
    TRISA = 0;
    PORTA = 0;
    TRISC = 0b00001100; //RC1 = NAVRESET (active high), RC2 = GPIO (input), RC3 = HARTBEAT (input), RC5 = LED (active high)
    PORTC = 0b00000000;
  
    //timer1 for detecting heartbeat
    T1CONbits.TMR1CS = 0b00; //increment on Fosc/4
    T1CONbits.T1CKPS = 0b11; //prescaler 1:8
    TMR1ON = 1;

    //disable all analog ports
    ANSELA = 0;
    ANSELC = 0;

    //disable some peripherals
    CM1CON0bits.C1ON = 0;
    CM2CON0bits.C2ON = 0;
    RCSTAbits.SPEN = 0;    
}

void delay_sec(unsigned int n) {
    while (n--) {
        CLRWDT();
        LED = LED?0:1;
        __delay_ms(1000);
    }
}

void navreset() {
    NAVRESET = 1;
    __delay_us(200);    
    NAVRESET = 0;
}

void main(void) {
    char hb, hb_prev;

    init_io();

    TMR1 = 0;
    
    while(1) {
        CLRWDT();
        hb = HEARTBEAT;

        if (hb != hb_prev) { //reset timer
            TMR1 = 0;
            LED = 1;
            __delay_ms(10);
            LED = 0;
        }
        
        if (TMR1 > TMR1_MAX || !GPIO) { //reset master CPU
            navreset();
            TMR1 = 0;
            LED = 1;
            __delay_ms(10);
            LED = 0;
        }
        
        hb_prev = hb;
    }
    return;
}
