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

#define PW_NEUT 2765 //Motor PW from Lab 3-3
#define PW_MIN 2028
#define PW_MAX 3502

//Not used
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
int read_ranger(void); // new feature - read value, and then start a new ping
void pick_heading(void); // function which allow operator to pick desired heading
void pick_range(void);
void forward_mode(void); // new feature - adjust steering/speed control for forward mode
void set_speed_adj(void);
void reverse_mode(void); // new feature - adjust steering/speed control for reverse mode
void print(void);
void set_tail_PWM(void);
unsigned char random(void);
//define global variables
unsigned int MOTOR_PW;

unsigned int Tail_PW = 0; //Motor PW for the Tail Motor 

//unsigned int PCA_start = 28614;
unsigned char new_heading = 0; // flag for count of compass timing
unsigned char new_range = 0; // flag for count of ranger timing
unsigned char print_flag = 0; // flag for count of printing

unsigned int heading;
unsigned int desired_heading = 1200; //Compass direction 

unsigned int range;
unsigned int desired_range; //Distance threshold, set at start
unsigned char r_count; // overflow count for range
unsigned char h_count; // overflow count for heading
unsigned char print_count; // overflow count for printing
unsigned int Counts; //normal counter for time
unsigned int gain_heading; //kps, controls how fast the car turns
unsigned int gain_speed; //kpr, controls how fast the car slows down
unsigned char run_stop; //Boolean to allow one data initialization
int previous_error = 0;

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
	PCA0CPL2 = 0xFFFF - Tail_PW;
	PCA0CPH2 = (0xFFFF - Tail_PW) >> 8;
	Counts = 0;
	//Wait 1 second
	while (Counts < 50);
	//srand(1) is implicitly used since srand() is not called
	run_stop = 0;
	while (1)
	{
		if (!run_stop) //No point in doing this, just do outside while(1) loop
		{
			//Initial setter code
			run_stop = 1;
		}
		if (new_heading) // enough overflows for a new heading
		{
			//printf("compass \r\n");
			heading = read_compass();
			new_heading = 0;
			set_tail_PWM();
			//printf("Heading: %d,Motor PW: %d \r\n",heading,Tail_PW);
			//set_servo_PWM(); // if new data, adjust servo PWM for compass & ranger
		}
		/*if (new_range) // enough overflow for a new range
		{
			range = read_ranger(); // get range, also stores light value into light
			// read_ranger() must start a new ping after a read
			new_range = 0;
		}*/
		PCA0CPL0 = 0xFFFF - TAIL_PW;
		PCA0CPH0 = (0xFFFF - TAIL_PW) >> 8;//Set motor pw for rudder fan
		/*
		PCA0CPL1 = 0xFFFF - MOTOR_PW;
		PCA0CPH1 = (0xFFFF - MOTOR_PW) >> 8;//Set servo pw for thrust angle
		PCA0CPL2 = 0xFFFF - Tail_PW;
		PCA0CPH2 = (0xFFFF - Tail_PW) >> 8;//Set motor pw for left thrsut power fan
		PCA0CPL3 = 0xFFFF - MOTOR_PW;
		PCA0CPH3 = (0xFFFF - MOTOR_PW) >> 8;//Set motor pw for right thrust power fan
		*/
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
	//Set output pin for both CEX0,CEX1,CEX2 and CEX3
	P1MDOUT |= 0xF0; //set output pin (Pin 4,5,6,7) for CEX0-CEX3 in push-pull mode

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
	XBR0 = 0x25;
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
	PCA0CPM0 = 0xC2; // 16 bit, enable compare for CCM0/CEX0, enable PWM 
	PCA0CPM1 = 0xC2; // 16 bit, enable compare for CCM1/CEX1, enable PWM
	PCA0CPM2 = 0xC2; // 16 bit, enable compare for CCM2/CEX2, enable PWM
	PCA0CPM3 = 0xC2; // 16 bit, enable compare for CCM3/CEX3, enable PWM

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

unsigned int Read_Ranger(void)
{
    unsigned int range;
    unsigned int Data[2]
    i2c_read_data(0xE0, 2, Data, 2);
    range = (Data[0] << 8) + Data[1];
    Data[0] = 0x51;
    i2c_write_data(0xE0, 0, Data, 1);   // ping for next read range
    return range;
}
/*
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
}*/
//Set steering via tail fan
void set_tail_PWM(void)

{
	int PW=0;
	int kp=5;
	int kd=100;
	int error=0;
	int TailPWM=0;
	unsigned int headings[4] = {0,500,1200,2500};
	if (!(Counts % 150))
	{
		unsigned int index = random();
		desired_heading = headings[index];
	}
	error = desired_heading - heading;

	if (error > 1800 )
		error =  error - 3600;
	else if (error < -1800)
		error = error + 3600;

	TailPWM = (signed long)PW_NEUT+(signed long)kp*(signed long)error+
	(signed long)kd*(signed long)(error-previous_error);
	//printf("TailPWM: %d \r\n", TailPWM);
	Tail_PW=TailPWM;

	//printf("Tail PWM: %d \r\n",TailPWM);

	previous_error=error;
	if (TailPWM>PW_MAX) 
		Tail_PW=PW_MAX;
	if (TailPWM<PW_MIN)
		Tail_PW=PW_MIN;
	//Tail_PW=TailPWM;

	//printf("AdjustedPWM %d \r\n",TailPWM);
	//if (!(Counts % 20))
	
	if(heading == 0 || heading==450 || heading==900 || heading == 1350 ||
	heading == 1800 || heading == 2250 || heading == 2700 || heading ==3150)
	{
		printf("Heading: %d,TailPWM: %d,CorrectedPW: %d,Error: %d \r\n",heading/10,TailPWM,Tail_PW,error);
	}
	//}
}
/*
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
*/
void print()
{
	if (!(Counts % 20))
	{
		lcd_clear();
		lcd_print("H: %d, D: %d", heading, range);
	}
}

//Function to use ranger to adjust the thruster fan angle
//holding a fixed range value for ~5 seconds locks in the corresponding angle.
void Set_Angle(void)
{
    char set_angle, count;
    unsigned int adj, previous_adj, angle;

    count = 0;
    previous_adj = 0;
    set_angle = 1;

    while(set_angle)
    {
        while(!new_range);                          //new_range is global 80ms flag
		new_range = 0;
        adj = Read_Ranger();
        angle = 2765 + (adj - 40)*10;               //May change: 40 nominal height
                    	                             //            10 gain on rotation
        if (angle < PWMIN) angle = PWMIN;
        else if (angle > PWMAX) angle = PWMAX;
        PCA0CP1 = 0xFFFF - angle;                   //CEX1 is thrust fan servo

        if(abs(previous_adj - adj) < 8) count++;    //Adjust depending on how noisy data is
	else count = 0;
        if(count > 62) set_angle = 0;               //Assuming ranger reads every 80ms
	previous_adj = adj;                         //
	printf("\r Range = %u   ", adj);
    }
}

void SetMaxMin()
{	
	char input;
	printf("Set the Max pulsewidth\r\n");
	while(1)
	{
		
		
		printf("Press r to increase pulsewidth\r\n");

		input = getchar();
		if (input =='r')
		{
			SERVO_PW = SERVO_PW + 10; //increase the steering pulsewidth by 10
			printf("SERVO_PW: %u\n", SERVO_PW);
			PCA0CPL0 = 0xFFFF - SERVO_PW;
			PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;	
		}

		else if(input == 'l') //if 'l' is pressed by the user
		{
			SERVO_PW = SERVO_PW - 10; //decrease the steering pulsewidth by 10
			printf("SERVO_PW: %u\n", SERVO_PW);
			PCA0CPL0 = 0xFFFF - SERVO_PW;
			PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
		}

		else if(input == ' ')
		{
			PW_RIGHT=SERVO_PW;
			printf("%u is the maximum pulsewidth\r\n",PW_RIGHT );
			break;
		}
	
	}

	printf("Set the Min pulsewidth\r\n");
	while(1)
		
	{
		
		printf("Press l to decrease pulsewidth\r\n");

		input = getchar();
		if (input =='l')
		{
			SERVO_PW = SERVO_PW - 10; //increase the steering pulsewidth by 10	
			printf("SERVO_PW: %u\n", SERVO_PW);
			PCA0CPL0 = 0xFFFF - SERVO_PW;
			PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
		}

		else if(input == 'r') //if 'l' is pressed by the user
		{
			SERVO_PW = SERVO_PW + 10; //decrease the steering pulsewidth by 10
			printf("SERVO_PW: %u\n", SERVO_PW);
			PCA0CPL0 = 0xFFFF - SERVO_PW;
			PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
		}

		else if(input == ' ')
		{
			PW_LEFT=SERVO_PW;
			printf("%u is the minimum pulsewidth\r\n",PW_LEFT );
			break;
		}
	}
	printf("SERVO_PW: %u\n", SERVO_PW);
	PCA0CPL0 = 0xFFFF - SERVO_PW;
	PCA0CPH0 = (0xFFFF - SERVO_PW) >> 8;
}

/*return a random integer number between 0 and 3*/
unsigned char random(void)
{
  return (rand() % 4);	// rand returns a random number between 0 and 32767.
                    		// the mod operation (%) returns the remainder of 
                      	// dividing this value by 5 and returns the result,
                      	// a value of either 0, 1, 2, or 3.
}