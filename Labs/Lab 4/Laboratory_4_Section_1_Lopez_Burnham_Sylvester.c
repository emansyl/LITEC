/*
Names: Christian Lopez, Samuel Burnham and Emmanuel Sylvester
Section: 1 Side: A
Due: 11/16/18 

This program demonstrates the functionality of a smart car
with combined steering and driving manipulation in response
to user interface.
*/
#include <c8051_SDCC.h>
#include <stdlib.h>// needed for abs function
#include <stdio.h>
#include <i2c.h>

#define PW_NEUT 2769 //Motor PW from Lab 3-3
#define PW_MIN 2030
#define PW_MAX 3507
#define PW_RIGHT 3329 //Steering PW from Lab 3-3
#define PW_LEFT 1909
#define PW_CENTER 2769
#define PCA_start 28614
#define brightness_threshold 200
//-----------------------------------------------------------------------------
// 8051 Initialization Functions
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
void XBR0_Init();
void SMB_Init(void);
void ADC_Init(void);
void Interrupt_Init(void);
void PCA_ISR(void) __interrupt 9;
int read_compass(void);
void set_servo_PWM(void);
unsigned char read_AD_input(unsigned char pin_number);
int read_ranger(void); // new feature - read value, and then start a new ping
void pick_heading(void); // function which allow operator to pick desired heading
void pick_range(void);
void forward_mode(void); // new feature - adjust steering/speed control for forward mode
void set_speed_adj(void);
void reverse_mode(void); // new feature - adjust steering/speed control for reverse mode
void print(void);
//define global variables
unsigned int PW_Init; //Initial pulsewidth set by pot value, add tp PW_NEUT for forward speed
unsigned int MOTOR_PW;

unsigned int SERVO_PW = 0; //Servo from Lab 3-3

//unsigned int PCA_start = 28614;
unsigned char new_heading = 0; // flag for count of compass timing
unsigned char new_range = 0; // flag for count of ranger timing
unsigned char print_flag = 0; // flag for count of printing

unsigned int heading;
unsigned int desired_heading; //Compass direction, set at start 

unsigned int range;
unsigned int desired_range; //Distance threshold, set at start
unsigned char AD; //Pot value converted from analog to digital
unsigned char light = 0;
//unsigned int brightness_threshold = 200; //Ballpark, based on average light value of a cell phone
unsigned char addr = 0xE0; //Address of 
unsigned char Data[3]; //Data array read and written on for SMBus
int compass_adj = 0; // correction value from compass
int range_adj = 0; // correction value from ranger
unsigned char r_count; // overflow count for range
unsigned char h_count; // overflow count for heading
unsigned char print_count; // overflow count for printing
unsigned int Counts; //normal counter for time
unsigned int gain_heading; //kps, controls how fast the car turns
unsigned int gain_speed; //kpr, controls how fast the car slows down
unsigned char reverse; //Boolean to determine if car is in reverse, used to flip steering
unsigned char run_stop; //Boolean to allow one data initialization

__sbit __at 0xB7 RUN;

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
	Sys_Init(); // initialize board
	putchar(' ');
	Port_Init();
	PCA_Init();
	XBR0_Init();
	SMB_Init();
	ADC_Init();
	Interrupt_Init();
	ADC_Init();
	r_count = 0;
	h_count = 0;
	MOTOR_PW = PW_NEUT;
	//Code to set the servo motor in neutral for one second
	PCA0CPL2 = 0xFFFF - MOTOR_PW;
	PCA0CPH2 = (0xFFFF - MOTOR_PW) >> 8;
	Counts = 0;
	//Wait 1 second
	while (Counts < 50);
	//printf("First, rotate potentiometer to set forward speed:\r\n");
	while (1)
	{
		run_stop = 0;
		while (!RUN)
		{ 
		//Continuously read A/D value to set forward speed until switch is in run position
			AD = read_AD_input(6);
			PW_Init = AD*2.89;
			if (!run_stop)
			{
				pick_range();
				pick_heading();
				lcd_clear();
				lcd_print("Desired range: %d, Kpr: %d, Desired heading: %d, Kps: %d\r\n",
				 desired_range, gain_speed, desired_heading, gain_heading);
				run_stop = 1;
			}
 		}
		if (new_heading) // enough overflows for a new heading
		{
			//printf("compass \r\n");
			heading = read_compass();
			new_heading = 0;
			set_servo_PWM(); // if new data, adjust servo PWM for compass & ranger
		}
		if (new_range) // enough overflow for a new range
		{
			range = read_ranger(); // get range, also stores light value into light
			// read_ranger() must start a new ping after a read
			if (light > brightness_threshold) // if bright light is detected
			{
				reverse = 1;
				//printf("REVERSE");
				reverse_mode(); // adjust steering control variables for reverse mode
				// and drive PWM for reverse direction
			}
			else
			{
				reverse = 0;
				forward_mode(); // adjust steering control variables for forward mode
												// and drive PWM for forward direction
				if (range < desired_range) // if an obstacle is detected
				{
					set_speed_adj(); //adjust drive PWM for obstacle
				}
			}
			//printf("MOTOR_PW: %d\r\n", MOTOR_PW);
			new_range = 0;
		}
		PCA0CPL0 = 0xFFFF - SERVO_PW;
		PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
		PCA0CPL2 = 0xFFFF - MOTOR_PW;
		PCA0CPH2 = (0xFFFF - MOTOR_PW) >> 8;
		print();
	}
}
//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
	//Set output pin for both CEX0 and CEX2
	P1MDOUT |= 0x04; //set output pin (Pin 0) for CEX2 in push-pull mode
	P3MDOUT &= ~0x80;	//Set pin 3.5 I/O for SS
	P3 |= 0x80;

	
	 //Analog input & output
	P1MDIN &= ~0x40; //Set pin 6 of Port 1 for analog input
	P1MDOUT &= ~0x40; //Set pin 6 of Port 1 as "open drain"
	P1 |= 0x40; //Send logic 1 to pin 6 of Port 1
}
//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Interrupt_Init()
{
	// IE and EIE1
	EIE1 = 0x08;
	EA = 1;
}
//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{
	XBR0 = 0x27;
}
//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
	// reference to the sample code in Example 4.5 - Pulse Width Modulation implemented using
	//the PCA (Programmable Counter Array, p. 50 in Lab Manual.
	// Use a 16 bit counter with SYSCLK/12.
	PCA0MD = 0x81; //Enable CF interrupt for SYSCLK/12
	PCA0CPM0 = 0xC2;
	PCA0CPM2 = 0xC2; // 16 bit, enable compare for CCM2/CEX2, enable PWM
	PCA0CPM3 = 0xC2;
	PCA0CN = 0x40; //Enable PCA counter
}
//-----------------------------------------------------------------------------
// SMBus_Init
//-----------------------------------------------------------------------------
//
void SMB_Init(void)
{
	SMB0CR = 0x93; // set SCL to 100KHz
	ENSMB = 1; // bit 6 of SMB0CN enable the SMBUS
}
//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR(void) __interrupt 9
{
	if (CF)
	{
		PCA0 = PCA_start;
 		CF = 0; // clear overflow indicator
 		h_count++;
	 	if (h_count>=2)
		{
			new_heading=1;
			h_count = 0;
		}
		r_count++;
		if (r_count>=4)
		{
			new_range = 1;
			r_count = 0;
		}
		if (print_count>=25)
		{
			print_flag = 1;
			print_count = 0;
		}
	}
	// handle other PCA interrupt sources
	PCA0CN &= 0xC0;
	//Simple time checker: 50 Counts = 1s
	Counts++;
}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//
// Set up A/D conversion
//

void ADC_Init(void)
{
	REF0CN = 0x03; /* Set Vref to use internal reference voltage (2.4V) */
	ADC1CN = 0x80; /* Enable A/D converter (ADC1) */
	ADC1CF &= ~0x03; // Set bits 1-0 to 0;
	ADC1CF |= 0x01; /* Set A/D converter gain to 1 */
}
unsigned char read_AD_input(unsigned char pin_number)
{
	AMX1SL = pin_number; /* Set P1.n as the analog input for ADC1 */
	ADC1CN = ADC1CN & ~0x20; /* Clear the “Conversion Completed” flag */
	ADC1CN = ADC1CN | 0x10; /* Initiate A/D conversion */
	while ((ADC1CN & 0x20) == 0x00); /* Wait for conversion to complete */
	return ADC1; /* Return digital value in ADC1 register */
}
//-----------------------------------------------------------------------------
// All other routines ...
//-----------------------------------------------------------------------------
//

 int read_compass()
{
	unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data_1[2];      // Data is an array with a length of 2
	unsigned int heading;       // the heading returned in degrees between 0 and 3599
	i2c_read_data(addr,2,Data_1,2); // read two byte, starting at reg 2
	heading =(((unsigned int)Data_1[0] << 8) | Data_1[1]); //combine the two values
	                           //heading has units of 1/10 of a degree
	//printf("Heading %d \r\n",heading);
	return heading;            // the heading returned in degrees between 0 and 3599
}

int read_ranger(void)
{
	i2c_read_data(addr,1,Data,3);
	range = (((unsigned int)Data[1] << 8) | Data[2]);
	//Read light value
	light = Data[0];
	Data[0] = 0x51; //start ping
	i2c_write_data(addr,0,Data,1);
	//printf("Distance frpm ranger: %d \r\n",range);
	//printf("Light: %d\r\n", light);
	return range;
}

void set_servo_PWM(void)
{
	int error=0;
	int PW=0;
	error=desired_heading - heading;
	if (error > 1800 )
		error =  error - 3600;
	else if (error < -1800)
		error = error + 3600;
	if (reverse == 1)
		error *= -1;
	PW=gain_heading*error + PW_CENTER;//code to set new PW
	//printf("Servo PW Before: %d \r\n",PW);
	if (PW>PW_RIGHT) 
		PW=PW_RIGHT;
	if (PW<PW_LEFT)
		PW=PW_LEFT;

	SERVO_PW=PW;
	//printf("Heading: %d\r\n", SERVO_PW);
	//printf("Error: %d \r\n",error);
	//printf("Servo PW: %d \r\n",PW);
	//printf("SERVO_PW: %u\n", SERVO_PW);
}

//Set error
void pick_heading(void)
{
	lcd_clear();
	lcd_print("Enter desired heading:");
	desired_heading = kpd_input(1);	
	lcd_clear();
	lcd_print("Enter a Kps value (1 - 5) into the keypad:");
	gain_heading = kpd_input(1);
}

void pick_range(void)
{	
	lcd_clear();
	lcd_print("Enter a desired obstacle threshold:");
	desired_range = kpd_input(1);
	lcd_clear();
	lcd_print("Enter a Kpr value (1 - 50) into the keypad:");
	gain_speed = kpd_input(1);
}
void forward_mode()
{
	//Continuously read the A/D value of Port 1, Pin 6
	//Calculated slope so that speed increases linearly from AD = 0 to AD = 255
	MOTOR_PW = PW_Init+PW_NEUT;
	//printf("Forward speed: %d\r\n", MOTOR_PW);
	//Need to adjust steering variables, call steering keypad input
	// function?
}
//Linearly slow down the car from distance = 60 to distance = 30
//Equation: y = k(x-30) + PW_NEUT
//
void set_speed_adj()
{
	//printf("Obstacle detected: Distance = %d\r\n", range);
	if (range > desired_range/2)
		MOTOR_PW = PW_NEUT-gain_speed*(desired_range-range)+PW_Init;
	else if (range <= desired_range/2)
		MOTOR_PW = PW_NEUT;
	//set_steering();
}
void reverse_mode()
{
	MOTOR_PW = PW_MIN; // will make car go in reverse
	//printf("CAR IS IN REVERSE MODE:");
	//printf("SERVO PW: %d\r\n", SERVO_PW);
}
void print()
{
	if (!(Counts % 20))
	{
		lcd_clear();
		lcd_print("H: %d, D: %d", heading, range);
	}
}