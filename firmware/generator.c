#include <stdio.h>
#include <math.h>

typedef unsigned char u8;
typedef unsigned char uint8_t;
typedef unsigned int u16;
typedef unsigned int uint16_t;
typedef unsigned long u32;

u8 transition[1792];

#define SLOPE 0.01
#define N_CALC 21.5972
#define C_CALC -N_CALC

int main(void){
    u16 temp;
    
    for (temp = 0; temp<256; temp++){

	uint8_t red =   227; //0b11100011;
	uint8_t green = 143; //0b10001111;
	uint8_t blue =  57; //0b00111001;
	
	uint8_t index = temp / 256;     // 0 to 6
	uint8_t remainder = temp % 256;  // 0 to 255
	
	//(red >> (6-index)) & 0b11
	// Four possible transitions
	// 00 = 0
	// 01 = 1
	// 10 = 2
	// 11 = 3
        
		//OCR1B[temp] = 255 * (red>>(7-index) & 1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((red>>(6-index) & 1) - (red>>(7-index) & 1));	
        //OCR1A[temp] = 255 * (green>>(7-index) & 1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((green>>(6-index) & 1) - (green>>(7-index) & 1)); 
        //OCR0A[temp] = 255 * (blue>>(7-index) & 1) + (N_CALC * exp ( SLOPE * remainder ) + C_CALC) * ((blue>>(6-index) & 1) - (blue>>(7-index) & 1));        
        
        transition[temp] = (N_CALC * exp ( SLOPE * remainder ) + C_CALC);
        
        //printf("temp:%u R:%u G:%u B:%u\n",temp,OCR0A,OCR1A,OCR1B);
    }
    
    
    printf("const uint8_t transition[] = {%u",transition[0]);
    for (temp = 1; temp<256; temp++){
        printf(",%u",transition[temp]);
    }
    printf("};\n");
    
    return;
}
