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
#define forever         117
#define LEDON          PORTB |= (1<<4)
#define LEDOFF           PORTB &= ~(1<<4)

#define DELAYT         200

/// Prototypes ////////
void InitPorts (void);
void Delay (u32 count);

int main(void)
{
    InitPorts();
	
    while (forever)
    {
		LEDOFF; _delay_ms(DELAYT);
        LEDON; _delay_ms(DELAYT);
    }
}

void InitPorts(void)
{
    DDRB |= 1<<DDB4;
}