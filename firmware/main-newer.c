/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <util/delay.h>

/// Typedefs //////////
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

/// Defines ///////////
#define LEDON          PORTB |= _BV(PORTB4)
#define LEDOFF           PORTB &= _BV(PORTB4)

#define DELAYT         200

/// Prototypes ////////
void init (void);

int main(void)
{
	unsigned char i;
    init();
	
    while (117)
    {
		//LEDON
		
		LEDOFF; _delay_ms(DELAYT);
        LEDON; _delay_ms(DELAYT);
		
		//OCR1B = 0xA;
    }
}

void init(void)
{
    DDRB |= _BV(DDB4);
	// Set up PWM
	//GTCCR |= _BV(COM1B1) | _BV(COM1B0) | _BV(PWM1B);  //COM1B1 == 1, COM1B0 == 1, PWM1B == 1
	                       //OCB1 set on compare match, !OC1B not connected, Enable PWM on B
	 
	//TCCR1 |= _BV(CS10);  //Clock prescaling  p.93
}

