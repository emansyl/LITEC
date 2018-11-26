/* Sample code for main function to read the compass and ranger */
#include <c8051_SDCC.h>
#include <stdlib.h>// needed for abs function
#include <stdio.h>
#include <i2c.h>

#define PW_NEUT 2769 //PW from Lab 3-3, same for servo and speed
#define PW_MIN 2030
#define PW_MAX 3507
#define PW_RIGHT 3329
#define PW_LEFT 1909
#define PW_CENTER 2769
#define PCA_start 28614
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
void Accel_Init_C(void);

unsigned char read_AD_input(unsigned char pin_number);
void set_gains(void); // function which allow operator to set feedback gains
void read_accels(void); //Sets global variables dx & dy
void set_servo_PWM (void);
void set_drive_PWM(void);
void updateLCD(void);

//define global variables
unsigned int MOTOR_PW;

unsigned int SERVO_PW = 0; //Servo from Lab 3-3

unsigned char AD;
//Boolean to check if currently in forward or reverse, 0 = forward, 1 = reverse
unsigned int Counts; //normal counter for time
unsigned int kds; //Heading gain
unsigned int kdx;
unsigned int kdy;
unsigned char reverse;

unsigned char new_accels = 0; // flag for count of accels timing
unsigned char new_lcd = 0; // flag for count of LCD timing
unsigned int range;
unsigned char a_count; // overflow count for acceleration
unsigned char lcd_count; // overflow count for LCD updates

int dx=0;
int dy=0;

__sbit __at 0xB7 RUN;
__sbit __at 0xB5 BILED0;
__sbit __at 0xB6 BILED1;



//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
	unsigned char run_stop;
	putchar(' ');
	Sys_Init(); // initialize board
	Port_Init();
	PCA_Init();
	XBR0_Init();
	SMB_Init();
	ADC_Init();
	Interrupt_Init();
	Accel_Init_C();
	a_count = 0;
	lcd_count = 0;
	while (1)
	{
		run_stop = 0;
		while (!RUN)
		{
			if (!run_stop)
			{
				set_gains(); // function adjusting feedback gains
				run_stop = 1; // only try to update once
			}
 		}
		if (new_accels) // enough overflows for a new reading
		{
			read_accels();
			set_servo_PWM(); // set the servo PWM
			set_drive_PWM(); // set drive PWM
			new_accels = 0;
			a_count = 0;
		}
		if (new_lcd) // enough overflow to write to LCD
		{
			updateLCD(); // display values
			new_lcd = 0;
			lcd_count = 0;
		}
		PCA0CPL0 = 0xFFFF - SERVO_PW;
		PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
		PCA0CPL2 = 0xFFFF - MOTOR_PW;
		PCA0CPH2 = (0xFFFF - MOTOR_PW) >> 8;
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
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{
	//Enables UARTO, SMB0, CEX0-3
	XBR0 = 0x25;
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
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR(void) __interrupt 9
{
	if (CF)
	{
		CF = 0; // clear overflow indicator
		a_count++;
		//Use guess and check values between 1 and 20 to fill in the blanks
		if (a_count>= 1)
		{
			new_accels=1;
			a_count = 0;
		}
		lcd_count++;
		if (lcd_count>= 20)
		{
			new_lcd = 1;
			lcd_count = 0;
		}
		PCA0 = PCA_start;
	}
	// handle other PCA interrupt sources
	PCA0CN &= 0xC0;
	//Simple time checker: 50 Counts = 1s
	Counts++;
}
//-----------------------------------------------------------------------------
// All other routines ...
//-----------------------------------------------------------------------------
//


unsigned char read_AD_input(unsigned char pin_number)
{
	AMX1SL = pin_number; /* Set P1.n as the analog input for ADC1 */
	ADC1CN = ADC1CN & ~0x20; /* Clear the “Conversion Completed” flag */
	ADC1CN = ADC1CN | 0x10; /* Initiate A/D conversion */
	while ((ADC1CN & 0x20) == 0x00); /* Wait for conversion to complete */
	return ADC1; /* Return digital value in ADC1 register */
}

//Use keypad to set the gain values for steering, pitch speed, and roll speed
//Do we need to set pot gain?
void set_gains()
{
	lcd_clear();
	lcd_print("Enter a kds value (1 - 10) into the keypad:");
	kds = kpd_input(1);
	lcd_clear();
	lcd_print("Enter a kdx value (1 - 50) into the keypad:");
	kdx = kpd_input(1);
	lcd_clear();
	lcd_print("Enter a kdy value (1 - 50) into the keypad:");
	kdy = kpd_input(1);
	lcd_clear();
	lcd_print("Values: %d, %d, %d", kds, kdx, kdy);
}

void read_accels(void)
{
	char i=0;
	unsigned char addr = 0x3A; // the address of the Accelerometer, 0xC0 for the compass
	unsigned char Data[5];      // Data is an array with a length of 5

	int avg_dx=0;
	int avg_dy=0;
	while (i<9)
	{
		i2c_read_data(addr,0x27 | 0x80,Data,5); // reads all 5 bytes, starting at reg 0x27(status register)
		//x-axis stored in registers 0x28 and 0x29 
		//y-axis stored in registers 0x2A and 0x2B
		if ((Data[0] & 0x03)==0x03)
		{
			avg_dx +=((Data[2]<<8|Data[1])>>4); 
			avg_dy +=((Data[4]<<8]|Data[3]>>4));
			i++;
		}
	}
	avg_dx=avg_dx>>3;
	avg_dy=avg_dy>>3;
	dx=avg_dx;
	dy=avg_dy;
}

void set_servo_PWM()
{
	int PW=0;
	PW=PW_CENTER-(kds*dx)
	if (PW>PW_RIGHT) 
		PW=PW_RIGHT;
	if (PW<PW_LEFT)
		PW=PW_LEFT;
	SERVO_PW=PW;
}

void set_drive_PWM()
{
	MOTOR_PW = PW_NEUT + kdy * dy;
	MOTOR_PW += kdx * abs(dx);
	if (MOTOR_PW > PW_NEUT)
	{
		if (MOTOR_PW > PW_MAX)
			MOTOR_PW = PW_MAX;
		BILED0 = 0;
		BILED1 = 1;
	}
	else if (MOTOR_PW < PW_NEUT)
	{
		if (MOTOR_PW < PW_MIN)
			MOTOR_PW = PW_MIN;
		BILED0 = 1;
		BILED1 = 0;
	}
	else //car is stopped, BILED off
		BILED0 = 1;
		BILED1 = 1;
}

//Update_Gains/Values function to be able to change gains without recompiling?

//Prints current values: heading & range? 
//Other potential values: Pitch, if obstacle detected?
void updateLCD()
{
	lcd_clear();
	lcd_print("dx: %d, dy: %d \n kds: %d, kdx: %d, kdy: %d", dx, dy, kds, kdx, kdy);
}

//Gain steps:
// Initially, the user sets the heading gain and roll gain
// Then, user sets pitch gain via potentiometer until slide switch is flipped
// 
// 
// 
// 
// 