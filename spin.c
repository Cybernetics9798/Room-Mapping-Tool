#include <stdint.h>
#include <stdio.h>
#include "spin.h"
#include "tm4c1294ncpdt.h"
#include "SysTick.h"
#include "onboardLEDs.h"


//Use PortL pins (PL0-PL3) for motor output
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;						// activate clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0x0F;        										// configure Port L pins (PL0-PL3) as output
  GPIO_PORTL_AFSEL_R &= ~0x0F;     										// disable alt funct on Port L pins (PL0-PL3)
  GPIO_PORTL_DEN_R |= 0x0F;        										// enable digital I/O on Port L pins (PL0-PL3)
  GPIO_PORTL_AMSEL_R &= ~0x0F;     										// disable analog functionality on Port L	pins (PL0-PL3)	
	return;
}

void spin(int inner_rotation, int clockwise){			// Complete function spin to implement the Full-step Stepping Method
	uint32_t delay = 1;															// 2ms is the minimum time delay required between steps for the full step mode
	if(clockwise){
		for(int i=0; i<inner_rotation; i++){          // What should the upper-bound of i be for one complete rotation of the motor shaft?												
			GPIO_PORTL_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);										// What if we want to reduce the delay between steps to be less than 10 ms?
			GPIO_PORTL_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
			GPIO_PORTL_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTL_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
		}
	}
	else
	{
		for(int i=0; i<inner_rotation; i++){												// What should the upper-bound of i be for one complete rotation of the motor shaft?
			GPIO_PORTL_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
			GPIO_PORTL_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTL_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTL_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
		}
	}
}

void spin45(){
	int delay = 2;
	GPIO_PORTL_DATA_R = 0b00000011;
	SysTick_Wait1ms(delay);
	GPIO_PORTL_DATA_R = 0b00001001;
	SysTick_Wait1ms(delay);
	GPIO_PORTL_DATA_R = 0b00001100;
	SysTick_Wait1ms(delay);
	GPIO_PORTL_DATA_R = 0b00000110;
	SysTick_Wait1ms(delay);
}

void home(){
	int delay = 2;
	for(int i=0; i<512; i++){
		GPIO_PORTL_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);
		GPIO_PORTL_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTL_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		GPIO_PORTL_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);
	}
}

void motorOff(){
	GPIO_PORTL_DATA_R = 0b00000000;
}