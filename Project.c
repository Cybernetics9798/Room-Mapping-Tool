#include <stdint.h>
#include <math.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "I2C_Init.h"
#include "spin.h"
#include "onboardLEDs.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
#define TOTALSCAN								3						// change this to alter the total scan number

//Initialization Section----------------------------------------------------------------------------------------------------------------------
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;												// activate clock for Port G
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};								// allow time for clock to stabilize
	GPIO_PORTG_DIR_R &= ~0x01;																			// make PG0 input (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;																		// disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;																				// enable digital I/O on PG0											
	GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;	// configure PG0 as GPIO
	GPIO_PORTG_AMSEL_R &= ~0x01;                        						// disable analog functionality on PN0
	return;
}

void VL53L1X_XSHUT(void){
	GPIO_PORTG_DIR_R |= 0x01;								// make PG0 out
	GPIO_PORTG_DATA_R &= 0b11111110;        //PG0 = 0
	FlashAllLEDs();
	SysTick_Wait10ms(10);
	GPIO_PORTG_DIR_R &= ~0x01;             	// make PG0 input (HiZ)
}

// Button Port
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0b00000011;    							// Make PJ1 & PJ0 input 
	GPIO_PORTJ_DEN_R |= 0b00000011;    							// Enable digital I/O on PJ1 & PJ0
	GPIO_PORTJ_PCTL_R = 0b11111111;	 								// Configure PJ1 & PJ0 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											// Disable analog functionality on PJ1 & PJ0		
	GPIO_PORTJ_PUR_R |= 0x03;													// Enable weak pull up resistor on PJ1 & PJ0
}

//Port M is used as output
void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;						// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        										// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     										// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        										// enable digital I/O on Port M pins (PM0-PM3)
	GPIO_PORTM_AMSEL_R &= ~0x0F;     										// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

//-------------------------------------------------------------------------------------------------------------------------------
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady = 0;

uint8_t guessPush = 0;
uint8_t active = 0;
uint16_t dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
uint8_t status = 0;
uint32_t currState = 0b00000000;
uint8_t sensorState = 0;

double pi;
double x;
double y;
uint8_t layer = 0;
double coordinate[TOTALSCAN][8][3];

void getCoord(int distance, int degree, double* x, double* y){
	double radians = degree * pi / 180;
	*x = distance * cos(radians);
	*y = distance * sin(radians);
}

void readVal(int step, double stepDeg){
	//status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value
	//status = VL53L1X_GetSignalRate(dev, &SignalRate);
	//status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
	//status = VL53L1X_GetSpadNb(dev, &SpadNum);
  
	getCoord(Distance, stepDeg * step, &x, &y);
	coordinate[layer][step][0] = x;
	coordinate[layer][step][1] = y;
	coordinate[layer][step][2] = layer*150;
}

void bootSensor(){
	//Check status of the sensor
	status = VL53L1X_GetSensorId(dev, &wordData);
	//sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n", wordData);
	//UART_printf(printf_buffer);
	
	// 1 Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	// 2 Initialize the sensor with the default setting
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
	//status = VL53L1X_StartRanging(dev);
	
	// 4 Starts Ranging
	status = VL53L1X_StartRanging(dev);
}

void act(){
	int guessPush = 0;
	for(int i=0; i < 8; i++){
		for(int j = 0; j < 64; j++){
			spin45();
			//Dectection and Debounce - Can't be put at the beginning of spin!
//			if((GPIO_PORTJ_DATA_R & 0b00000010) == 0){
//				SysTick_Wait10ms(5);
//				if((GPIO_PORTJ_DATA_R & 0b00000010) == 0){
//					active = active ^ 1;
//					guessPush = 1;
//				}
//			}
			//If button is pushed
			if(guessPush == 1){
				SysTick_Wait10ms(1);
				return;
			}
		}
		//Transmit Data
		while(dataReady == 0){
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
			FlashLED4(1);
			VL53L1_WaitMs(dev, 5);
		}
		dataReady = 0;
		readVal(i,45);
		//sprintf(printf_buffer, "%f,%f\n", coordinate[i][0], coordinate[i][1]);
		//UART_printf(printf_buffer);
		FlashLED3(4);
	}
	layer++;
	home();
}

int main(void){
	//Initialize all the necessary ports
	PLL_Init();
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortL_Init();
	PortJ_Init();
	PortM_Init();
	
	//GPIO_PORTM_DATA_R = 0b00001000;
	uint8_t byteData=0;
	char input = ' ';
	pi = acos(-1);	//obtain pi using acos
	
	bootSensor();
	
	//Data collecting stage
	int terminate = 0;
	//while(terminate != 1 || layer < TOTALSCAN){
	while(layer < TOTALSCAN){
		//Debounce process
		while(guessPush == 0){
			if((GPIO_PORTJ_DATA_R & 0b00000010) == 0 || (GPIO_PORTJ_DATA_R & 0b00000001) == 0){
				SysTick_Wait10ms(3);
				if((GPIO_PORTJ_DATA_R & 0b00000010) == 0 && (GPIO_PORTJ_DATA_R & 0b00000001) == 1){
					active = active ^ 1;
					guessPush = 1;
				}else if((GPIO_PORTJ_DATA_R & 0b00000001) == 0 && (GPIO_PORTJ_DATA_R & 0b00000010) == 1){
					terminate = 1;
					guessPush = 1;
				}
			}
		}
		guessPush = 0;
		//Start spinning the motor
		if(active == 1){
			act();
			//GET YOUR HANDS OFF THE BUTTON!
			while((GPIO_PORTJ_DATA_R & 0b00000010) == 0){}
		}
	}
	
	//Data transmitting stage
	motorOff();
	status = VL53L1X_StopRanging(dev);
	input = UART_InChar();
	while(input == 's'){
		FlashAllLEDs();
		int transmit = 0;
		if((GPIO_PORTJ_DATA_R & 0b00000010) == 0){
			//sprintf(printf_buffer, "%d\n", layer);
			//UART_printf(printf_buffer);
			transmit = 1;
			while((GPIO_PORTJ_DATA_R & 0b00000010) == 0){}
		}
		if(transmit == 1){
			for(int i = 0; i < TOTALSCAN; i++){
				for(int j = 0; j < 8; j++){
					sprintf(printf_buffer, "%f,%f,%f\n", coordinate[i][j][0], coordinate[i][j][1], coordinate[i][j][2]);
					UART_printf(printf_buffer);
					SysTick_Wait1ms(1);
				}
			}
			transmit = 0;
		}
	}
	return 0;
}