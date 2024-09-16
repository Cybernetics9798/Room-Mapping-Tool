#include "I2C_Init.h"
#include "tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>

//ToF Receiver & I2C Init
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

	GPIO_PORTB_AFSEL_R |= 0x0C;           															// 3) enable alt funct on PB2,3       0b00001100
	GPIO_PORTB_ODR_R |= 0x08;             															// 4) enable open drain on PB3 only

	GPIO_PORTB_DEN_R |= 0x0C;             															// 5) enable digital I/O on PB2,3
	GPIO_PORTB_AMSEL_R &= ~0x0C;          															// 7) disable analog functionality on PB2,3

                                                                      // 6) configure PB2,3 as I2C
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    	//TED
	I2C0_MCR_R = I2C_MCR_MFE;                      											// 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;                    // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
	I2C0_MTPR_R = 0x3B;                                        					// 8) configure for 100 kbps clock      
}