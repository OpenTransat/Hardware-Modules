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

#define RELAY PORTCbits.RC0
#define NAVRESET PORTCbits.RC1
#define HEARTBEAT PORTCbits.RC3
#define LED PORTCbits.RC5

#define TMR1INIT (65535 - 31000) //timer1 increments +31000 per second

#define RESET_AFTER_SEC 60 //reset after no heartbeat
//#define RESET_AFTER_SEC 15 //TESTING reset after no heartbeat

#define RELAY_AFTER_SEC 24UL*3600UL //periodically reset
//#define RELAY_AFTER_SEC 60 //TESTING periodically reset

#define RESET_US 200

volatile unsigned long sectimer1, sectimer2; //incrementing each second in timer1 interrupt

void init_io() {
    OSCCONbits.IRCF = 0b0000; //31kHz oscillator
    TRISA = 0;
    PORTA = 0;
    TRISC = 0b00001000; //RC0 = RELAY (active high), RC1 = NAVRESET (active high), RC3 = HARTBEAT (input), RC5 = LED (active high)
    PORTC = 0b00000000;
  
    //timer1 for detecting heartbeat
    T1CONbits.TMR1CS = 0b01; //increment on Fosc
    T1CONbits.T1CKPS = 0b00; //prescaler 1:1
    T1CONbits.TMR1ON = 1;

    //enable interrupts for timer1
    PIR1bits.TMR1IF = 0; //interrupt flag should be cleared before enabling interrupts (page 195 http://ww1.microchip.com/downloads/en/DeviceDoc/41440A.pdf)
    PIE1bits.TMR1IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

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
    __delay_us(RESET_US);    
    NAVRESET = 0;
}

void interrupt timer1_interrupt(void) {
    if(PIR1bits.TMR1IF && PIE1bits.TMR1IE) {
        sectimer1++;
        sectimer2++;
        TMR1 = TMR1INIT;
        PIR1bits.TMR1IF = 0;
    }
}

void main(void) {

    init_io();

    char hb, hb_prev;
    sectimer1 = 0;
    sectimer2 = 0;
    TMR1 = TMR1INIT;
    
    while(1) {
        CLRWDT();

        hb = HEARTBEAT;

        if (hb != hb_prev) { //reset timer
            sectimer1 = 0;
            LED = 1;
            __delay_ms(10);
            LED = 0;
        }

        if (sectimer1 > RESET_AFTER_SEC) { //no heartbeat: reset master CPU (no warning)
            navreset();
            sectimer1 = 0;
            LED = 1;
            __delay_ms(10);
            LED = 0;
        }

        if (sectimer2 > RELAY_AFTER_SEC) { //reset every RELAY_AFTER_SEC seconds
            navreset();
            sectimer1 = 0;
            sectimer2 = 0;
            LED = 1;
            __delay_ms(10);
            LED = 0;
        }
        
        hb_prev = hb;
    }
    return;
}
