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

const uint8_t transition[] = {0,0,0,0,0,1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,8,8,8,9,9,9,9,10,10,10,11,11,11,12,12,12,13,13,14,14,14,15,15,15,16,16,16,17,17,18,18,18,19,19,20,20,21,21,21,22,22,23,23,24,24,25,25,25,26,26,27,27,28,28,29,29,30,30,31,32,32,33,33,34,34,35,35,36,37,37,38,38,39,40,40,41,41,42,43,43,44,45,45,46,47,47,48,49,50,50,51,52,53,53,54,55,56,56,57,58,59,60,60,61,62,63,64,65,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,93,94,95,96,97,99,100,101,102,103,105,106,107,109,110,111,113,114,115,117,118,119,121,122,124,125,127,128,130,131,133,134,136,137,139,141,142,144,146,147,149,151,153,154,156,158,160,161,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,198,200,202,204,207,209,211,214,216,218,221,223,226,228,231,233,236,238,241,244,246,249,252,255};

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

void color(uint16_t temp) {
	// OC1B pin3 = Red
	// OC1A pin6 = Green
	// OC0A pin5 = Blue
 
	if (temp > MAX_COLOR) {
		return;
	}

	uint8_t red =   0b10001111;
	uint8_t green = 0b11100011;
	uint8_t blue =  0b00111001;
	
	uint8_t index = temp / 256;     // 0 to 6, equivilent to temp >> 8
	uint8_t remainder = temp % 256;  // 0 to 255
	
	//(red >> (6-index)) & 0b11
	// Four possible transitions
    // 00 = 0 , transition[0]
    // 01 = 1 , transition[remainder]
    // 10 = 2 , transition[255-remainder]
    // 11 = 3 , transition[255]
    
	// The following three lines take 300 bytes of code.
    
    OCR1B = transition[255 * (red>>(7-index) & 0b1) + remainder * ((red>>(6-index) & 0b1) - (red>>(7-index) & 0b1))];
	OCR1A = transition[255 * (green>>(7-index) & 0b1) + remainder * ((green>>(6-index) & 0b1) - (green>>(7-index) & 0b1))];
	OCR0A = transition[255 * (blue>>(7-index) & 0b1) + remainder * ((blue>>(6-index) & 0b1) - (blue>>(7-index) & 0b1))];
	
}

/*
void Delay(u32 count) {
	while (count >0) {
		count--;
		nop();
	}
}
*/
