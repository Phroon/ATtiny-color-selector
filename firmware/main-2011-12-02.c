/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
 
/* Current Layout:
 *
 *               *----*
 * !Reset - PB5 =| uu |= VCC
 *   ADC3 - PB3 =|    |= PB2 - (unused)
 *   OC1B - PB4 =|    |= PB1 - OC1A
 *          GND =|    |= PB0 - OC0A
 *               *----* 
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
	unsigned char i;
    init();
	
    while (1)
    {
		//LEDOFF; Delay(DELAYT);
        //LEDON; Delay(DELAYT);
		
		for(i=0;i<255;i++) {
			OCR1B = i;
			Delay(500);
		}
		
		//OCR1B = 0;
    }
}

void init(void)
{
    DDRB |= _BV(DDB4);  // | _BV(DDB0) | _BV(DDB1); //Pins as output
	
	// Set up PWM on Timer 1, OC1B
	GTCCR |= _BV(COM1B1) | _BV(PWM1B);  //COM1B1 == 1, COM1B0 == 0, PWM1B == 1
								//OCB1 set on compare match, !OC1B not connected, Enable PWM on B
	
	TCCR1 |= _BV(CS10);  //Clock prescaling  p.93
	
	//OCR1B = 0x10;
	
	
	// Set up Fast PWM on Timer 0, OC0A
	TCCR0A |= _BV(WGM01) | _BV(WGM00) | _BV(COM0A1);  //(WGM02 == 0,) WGM01 == 1, WGM00 == 1, COM0A1 == 1, COM0A0 == 0
                                            //Fast PWM Mode p.82, Non-inverting mode p.81
					    
	TCCR0B |= _BV(CS00);  //Clock prescaling p.82-83

	OCR0A = 125;
	
	
	//ADC: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=56429
	// or http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=108210
	
	ADMUX |= _BV(MUX1);   //Vcc ADC reference, ADC3 mux selected, p.139
	
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
}

void Delay(u32 count) {
	while (count >0) {
		count--;
		nop();
	}
}
