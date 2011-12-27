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
    DDRB |= _BV(DDB4);
	
	// Set up PWM
	GTCCR |= _BV(COM1B1) | _BV(PWM1B);  //COM1B1 == 1, COM1B0 == 0, PWM1B == 1
								//OCB1 set on compare match, !OC1B not connected, Enable PWM on B
	
	TCCR1 |= _BV(CS10);  //Clock prescaling  p.93
	
	//OCR1B = 0x10;
}

void Delay(u32 count) {
	while (count >0) {
		count--;
		nop();
	}
}