/* 
 * File:   main.c
 * Author: Joem Diente
 *
 * Created on March 3, 2019, 11:44 PM
 */
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include "config.h"

/* using intosc */
#define _XTAL_FREQ 1000000

/* assign pins*/
#define LED4 LATAbits.LATA5
#define LED5 LATAbits.LATA1
#define LED6 LATAbits.LATA2
#define LED7 LATCbits.LATC5
#define SW1 PORTCbits.RC4
#define mTouch PORTCbits.RC1
#define testled LATAbits.LATA4

/*
 * 
 */

/* prototypes */
void setup(void);
void loop(void);
void firstProgram(void);
void secondProgram(void);
void ADC_Start(int);
void PWM_Start(uint16_t);

/* global variables */
volatile int program_num = 1, program_limit = 2;
uint16_t duty_cycle;
int adc_msb; 
int sw1_debounce_count = 0, sw1_debounce_thres = 5;

/* Interrupt Service Routine */
void __interrupt () myIsr (void) {
    
    //TMR0IF Handle
    if(INTCONbits.TMR0IF == 1) {
        
        if(SW1 == 0) {
            sw1_debounce_count++;
            if(sw1_debounce_count == sw1_debounce_thres) {
                //debounced
                program_num++;
                if(program_num > program_limit) {
                    program_num = 1;
                }
            }
        }
        else if(SW1 == 1) {
            sw1_debounce_count = 0;
        }
    }
    
    /* Leaving ISR Clear Flags */
    INTCONbits.TMR0IF = 0;

};

/* main program */
int main() {
    setup();
    while(1) {
        loop();
        CLRWDT();
    }
    return (EXIT_SUCCESS);
}
void setup() {
    //Setting Port Directions
    TRISA  = 0b00000000;
    ANSELA = 0b00000000; 
    TRISB  = 0b00000000;
    ANSELB = 0b00000000; 
    TRISC  = 0b00010001;
    ANSELC = 0b00000001;
    
    /*              User Settings Initialization                 */
    /* ADC */
    WPUCbits.WPUC0 = 0; //disable weak-pullup in rc0
    ADCON1bits.ADFM = 0;  //left justified adres
    ADCON1bits.ADCS = 0b000; // Set ADC Conversion Clock Fosc/16 - 0000
    ADCON1bits.ADPREF = 0b00;// Vrpos connected to VDD
    ADCON0bits.CHS = 0b00100; //AN4 for RC0
    ADCON0bits.ADON = 1; //Enable ADC
    
    /*INTOSC settings HF*/
    OSCCONbits.IRCF = 0b1101 ; //4MHz Internal Osc
 
    /* TMER 0 */
    OPTION_REGbits.TMR0CS = 0;
    OPTION_REGbits.TMR0SE = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b100; //prescaler = 1:32
    
    /* TIMER 2 */
    PR2 = 255; //period register.
    T2CONbits.ON = 1;
    
    /* PWM */
    PWM3CONbits.POL = 0;
    PWM3CONbits.EN = 1;
    
    /* For interrupts */
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    INTCONbits.GIE = 1;
    
    //Clear LATs
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    __delay_ms(50);

}

void loop() {
    switch(program_num) {
        case 1:
            firstProgram();
            break;
        case 2:
            secondProgram();
            break;
    }     
}

void firstProgram() {
    /* Connect LATs to LED*/
    RA5PPS = 0b00000;
    RA1PPS = 0b00000;
    RA2PPS = 0b00000;
    RC5PPS = 0b00000;
    ADC_Start(4);
    adc_msb = ADRESH;
    LED7 = (adc_msb & 0b10000000) >> 7;
    LED6 = (adc_msb & 0b01000000) >> 6;
    LED5 = (adc_msb & 0b00100000) >> 5;
    LED4 = (adc_msb & 0b00010000) >> 4;
}
void secondProgram() {
    ADC_Start(4);
    duty_cycle = (ADRESH << 8) + ADRESL ;
    PWM_Start(duty_cycle);
}
void ADC_Start(int ch_sel) {
    ADCON0bits.CHS = ch_sel;
    __delay_ms(10);
    ADCON0bits.GO_nDONE = 1;    //start ADC
    while(ADCON0bits.GO_nDONE); //Wait till Sampling Completes
}
void PWM_Start(uint16_t dc) {
    /* Clear LATs */
    LED7 = 0;
    LED6 = 0;
    LED5 = 0;
    LED4 = 0;
    PWM3DCH = dc >> 8; 
    PWM3DCL = dc & 0b11000000;
    //Connect LED Pins to PWM module
    RA5PPS = 0b01110;
    RA1PPS = 0b01110;
    RA2PPS = 0b01110;
    RC5PPS = 0b01110;
}
