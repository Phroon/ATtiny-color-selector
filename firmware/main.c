/* Name: main.c - An AVR RGB color selector
 * 
 *
 * Copyright (C) 2011-2012 Michael Wren
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 
/* Current Layout:
 *
 *               *----*
 * !Reset - PB5 =|o   |= VCC
 *Button3 - PB3 =|    |= PB2 - Button2
 *   OC1B - PB4 =|    |= PB1 - OC1A
 *          GND =|    |= PB0 - OC0A
 *               *----* 
 *
 * OC0A pin5 = Blue
 * OC1A pin6 = Green
 * OC1B pin3 = Red
 * 
 * PB2  Button2 pin7 = incriment color
 * PB3  Button3 pin2 = decriment color
 */

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

/// Typedefs //////////
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

/// Defines ///////////
#define LEDON          PORTB |= _BV(PORTB4)
#define LEDOFF           PORTB &= ~_BV(PORTB4)

#define DELAYT         20000
#define nop()  __asm__ __volatile__("nop")

#define REPEAT_MS   10
#define EEPROM_MS	5000
#define MAX_COLOR	7*256 - 1

/// Prototypes ////////
void init (void);
unsigned long millis();
//void Delay (u32 count);
void color(uint16_t temp);

uint16_t EEMEM storedColor = 0;

int main(void)
{
	//unsigned char i;
    uint8_t reading;
	uint8_t previous = 0;
	unsigned long time2 = 0;
	unsigned long time3 = 0;
	uint8_t writeFlag = 0;
    
    uint16_t i;
	i = eeprom_read_word(&storedColor);
	if (i > MAX_COLOR) {
		i = MAX_COLOR;
	}
    color(i);
    
    init();
	
    for (;;)
    {
		// Buttons on PB2, PB3
		
		// Read each time through the loop
		reading = PINB;
		
		
        if ( (reading & _BV(PB2)) != (previous & _BV(PB2)) ) {
			// Edge detected, restart counting
			time2 = millis();
		}
        
		
		if ( millis() - time2 > REPEAT_MS) {
			//PB2 reading constant for 10ms, do stuff.
			if ( ~reading & _BV(PB2) ) {
				//Button 2 pressed.
                
                if (++i > MAX_COLOR) {
                    i = MAX_COLOR;
                }
                
                color(i);
				writeFlag = 1;
                
				time2 = millis(); //Restart counting so that holding the button re-triggers
			} else {
              // Button 2 up  
            }
		}
		
		
		
		if ( (reading & _BV(PB3)) != (previous & _BV(PB3)) ) {
			// Edge detected, restart counting
			time3 = millis();
		}
        
		
		if ( millis() - time3 > REPEAT_MS) {
			//PB3 reading constant for 10ms, do stuff.
			if ( ~reading & _BV(PB3) ) {
				//Button 3 pressed.
                
                if (i != 0) {
                    i--;
                }
                
                color(i);
				writeFlag = 1;
                
				time3 = millis(); //Restart counting so that holding the button re-triggers
			} else {
              // Button 3 up  
            }
		}
		
		
		if (millis() - time2 > EEPROM_MS && millis() - time3 > EEPROM_MS) {
			if (reading & (_BV(PB2) | _BV(PB3)) && writeFlag == 1) {
				//Both buttons up and writeFlag is set
				
				eeprom_write_word(&storedColor,i);
				writeFlag = 0;
			}
		}
	
		
		previous = reading;
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
	//Only CS10 set: no prescaling
    
	//OCR1B = 0x10;
	
	
	// Set up Fast PWM on Timer 0, OC0A
	TCCR0A |= _BV(WGM01) | _BV(WGM00) | _BV(COM0A1);  //(WGM02 == 0,) WGM01 == 1, WGM00 == 1, COM0A1 == 1, COM0A0 == 0
                                            //Fast PWM Mode p.82, Non-inverting mode p.81
					    
	TCCR0B |= _BV(CS00);  //Clock prescaling p.82-83
	//Only CS00 set, no prescaling
	
	
	//ADC: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=56429
	// or http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=108210
	
    /*
     * Comment out ADC for now.
     *
	ADMUX |= _BV(MUX1) | _BV(MUX0);   //Vcc ADC reference, ADC3 mux selected, p.139
	
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADATE) | _BV(ADIE); // Enable ADC, clock/16 prescaling,
	//Auto Trigger ADC in free-run (ADTS[2:0] = 0 in ADCSRB p.141-142), ADC interrupt enaled p.140
	//1MHz / 16 = 62.5 kHz 
	
	ADCSRA |= _BV(ADSC);  //Start free run of ADC conversions
	*/
    
	sei();  // Enable Interrupts
	
	/*
	 * Time begins here.
	 */
	 
	TIMSK |= _BV(TOIE1); // Timer/Counter1 Overflow interrupt enabled.
	
    PORTB |= _BV(PORTB2) | _BV(PORTB3); //Enable internal pullup resistors on PB2, PB3
}

#define MILLIS_INC	0
#define FRACT_INC	256 >> 3
#define FRACT_MAX	1000 >> 3

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR(TIMER1_OVF_vect)
{
// Arduino millis() https://github.com/arduino/Arduino/blob/master/hardware/arduino/cores/arduino/wiring.c
// Which is LGPL

	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

#define SLOPE 0.01
#define N_CALC 21.5971
#define C_CALC -N_CALC

void color(uint16_t temp) {
	// OC1B pin3 = Red
	// OC1A pin6 = Green
	// OC0A pin5 = Blue
 
	if (temp > MAX_COLOR) {
		return;
	}
	
	uint8_t red =   0b11100011;
	uint8_t green = 0b10001111;
	uint8_t blue =  0b00111001;
	
	uint8_t index = temp / 256;     // 0 to 6, equivilent to temp >> 8
	uint8_t remainder = temp % 256;  // 0 to 255
	
	//(red >> (6-index)) & 0b11
	// Four possible transitions
    // 00 = 0 , 0
    // 01 = 1 , 0 + C_CALC * exp ( SLOPE * remainder ) + N_CALC
    // 10 = 2 , 255 - C_CALC * exp ( SLOPE * remainder ) + N_CALC
    // 11 = 3 , 255
    
	// The following three lines take 300 bytes of code.
	
	OCR1B = 255 * (red>>(7-index) & 0b1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((red>>(6-index) & 0b1) - (red>>(7-index) & 0b1));	
	OCR1A = 255 * (green>>(7-index) & 0b1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((green>>(6-index) & 0b1) - (green>>(7-index) & 0b1)); 
	OCR0A = 255 * (blue>>(7-index) & 0b1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((blue>>(6-index) & 0b1) - (blue>>(7-index) & 0b1));
	
}

/*
void Delay(u32 count) {
	while (count >0) {
		count--;
		nop();
	}
}
*/
