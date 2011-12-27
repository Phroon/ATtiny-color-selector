/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
 
/* Current Layout:
 *
 *               *----*
 * !Reset - PB5 =|o   |= VCC
 *   ADC3 - PB3 =|    |= PB2 - (unused)
 *   OC1B - PB4 =|    |= PB1 - OC1A
 *          GND =|    |= PB0 - OC0A
 *               *----* 
 *
 * OC0A = Red
 * OC1A = Green
 * OC1B = Blue
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/// Typedefs //////////
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

/// Defines ///////////
#define LEDON          PORTB |= _BV(PORTB4)
#define LEDOFF           PORTB &= ~_BV(PORTB4)

#define DELAYT         20000
#define nop()  __asm__ __volatile__("nop")

/// Prototypes ////////
void init (void);
void Delay (u32 count);

int main(void)
{
	//unsigned char i;
    
    init();
	
    while (1)
    {
		//LEDOFF; Delay(DELAYT);
        //LEDON; Delay(DELAYT);
		/*
		for(i=0;i<255;i++) {
            OCR1A = i;
			OCR1B = i;
            OCR0A = i;
			Delay(500);
		}
        */
		
		//OCR1A = 0;
    }
}

void init(void)
{
    DDRB |= _BV(DDB4) | _BV(DDB0) | _BV(DDB1); //Pins as output
	
	// Set up PWM on Timer 1, OC1B
	GTCCR |= _BV(COM1B1) | _BV(PWM1B);  //COM1B1 == 1, COM1B0 == 0, PWM1B == 1
								//OCB1 cleared on compare match, !OC1B not connected, Enable PWM on B
	
	TCCR1 |= _BV(CS10) | _BV(COM1A1) | _BV(PWM1A);
    //OCA1 cleared on comprare match, !OC1A
    //Clock prescaling  p.93
    
	//OCR1B = 0x10;
	
	
	// Set up Fast PWM on Timer 0, OC0A
	TCCR0A |= _BV(WGM01) | _BV(WGM00) | _BV(COM0A1);  //(WGM02 == 0,) WGM01 == 1, WGM00 == 1, COM0A1 == 1, COM0A0 == 0
                                            //Fast PWM Mode p.82, Non-inverting mode p.81
					    
	TCCR0B |= _BV(CS00);  //Clock prescaling p.82-83
	
	
	//ADC: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=56429
	// or http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=108210
	
	ADMUX |= _BV(MUX1) | _BV(MUX0);   //Vcc ADC reference, ADC3 mux selected, p.139
	
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADATE) | _BV(ADIE); // Enable ADC, clock/16 prescaling,
	//Auto Trigger ADC in free-run (ADTS[2:0] = 0 in ADCSRB p.141-142), ADC interrupt enaled p.140
	//1MHz / 16 = 62.5 kHz 
	
	sei();  // Enable Interrupts
	
	ADCSRA |= _BV(ADSC);  //Start free run of ADC conversions
}

ISR(ADC_vect)
{
	//ADC conversion complete, execute this code and stuff.
	//ADC readout is in ADCW (ADC Word), 10 bits.
    
    /*
     * OCR0A = Red
     * OCR1A = Green
     * OCR1B = Blue
     */
    
    u16 temp = (ADCW*3)>>1;
    
    if (temp < 256) {
        OCR0A = 255;
        OCR1A = temp;
        OCR1B = 0;
    } else if (temp >= 256 && temp < 512){
        OCR0A = 255 - (temp - 256);
        OCR1A = 255;
        OCR1B = 0;
    } else if (temp >= 512 && temp < 768){
        OCR0A = 0;
        OCR1A = 255;
        OCR1B = temp - 512;
    } else if (temp >= 768 && temp < 1024){
        OCR0A = 0;
        OCR1A = 255 - (temp - 768);
        OCR1B = 255;
    } else if (temp >= 1024 && temp < 1280){
        OCR0A = temp - 1024;
        OCR1A = 0;
        OCR1B = 255;
    } else if (temp >= 1280 && temp < 1536) {
        /*
        OCR0A = 255;
        OCR1A = 0;
        OCR1B = 255 - (temp-1280);
         */
        OCR0A = 255;
        OCR1A = temp-1280;
        OCR1B = 255;
    }
    
    // Todo: toggle LED on each ISR call.  Blinking will mean something else happened.
    
    //PORTB ^= _BV(PORTB2);
    
    Delay(10); // 1 and 2  cause hangups, 3 does not.  WHY!?
    
    //OCR1A = ADCW >> 2;
}

void Delay(u32 count) {
	while (count >0) {
		count--;
		nop();
	}
}
