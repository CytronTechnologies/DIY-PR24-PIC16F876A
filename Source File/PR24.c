//============================================================================================================
//	Author			:  Cytron Technologies
//	Project Description	:  DIY-PID Motor Controller
//      Project description     :  PIC16F876A is use in this project.
//                                 This sample source code should be compiled with HI-TECH C Compiler and XC8 with MPLABX
//                                 and using HI-TECH compiler with MPLAB
//============================================================================================================


//	Include
//============================================================================================================
#if defined(__XC8)
   #include <xc.h>
   #pragma config CONFIG = 0x3F32
#else
#include <htc.h>			//include the PIC microchip header file

//===========================================================================
// 	configuration
//============================================================================
__CONFIG (0x3F32);
//FOSC = HS        // Oscillator Selection bits (HS oscillator)
//WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
//PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
//BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
//LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
//CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
//WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
//CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)



#endif

//	Define
//============================================================================================================
#define	RS			RB7				// RS pin of the LCD.
#define	E			RB6				// E pin of the LCD.
#define	D7			RC3				// DB7 pin of the LCD.
#define	D6			RC4				// DB6 pin of the LCD.
#define	D5			RC5				// DB5 pin of the LCD.
#define	D4			RB1				// DB4 pin of the LCD.
#define	D3			RB2				// DB3 pin of the LCD.
#define	D2			RB3				// DB2 pin of the LCD.
#define	D1			RB4				// DB1 pin of the LCD.
#define	D0			RB5				// DB0 pin of the LCD.

#define MOTOR_1		RC1				// 1st terminal of the motor.
#define MOTOR_2		RC0				// 2nd terminal of the motor.
#define MOTOR_SPEED	CCPR1L                          // The PWM value to control motor's speed.
#define SW1			RB0				// RUN switch.


//	Global Variables
//============================================================================================================
unsigned char kp = 0;
unsigned char ki = 0;
unsigned char kd = 0;


//	Function Prototypes
//===========================================================================================================
	

unsigned char ai_read(unsigned char an_sel);
void motor_right(unsigned char speed);
void motor_left(unsigned char speed);
void motor_stop(void);

void delay(unsigned long data);	
void e_pulse(void);
void lcd_clr(void);
void lcd_goto(unsigned char data);
void send_char(unsigned char data);
void send_config(unsigned char data);
void send_string(const char *s);
void send_dec(unsigned long data,unsigned char num_dig);


//	Main Function
//===========================================================================================================
void main(void)
{
	unsigned long rec_data=0;

	// Set the I/O direction.
	TRISB = 0b00000001;			// Configure PORTB I/O direction.
	TRISC = 0b00000000;			// Configure PORTC I/O direction.
	TRISA = 0b11111111;			// Configure PORTA I/O direction.		

	// Initialize the ports.
	PORTB = 0;
	PORTC = 0;

	// Setup PWM mode.
	CCP1CON = 0b00001100;		// Set as PWM Mode.
	T2CON = 0b00000101;			// Turn on Timer 2 with prescaler = 4.
	PR2 = 0xFF;					// Set the PWM period.
	CCPR1L = 0;					// No PWM Duty Cycle during startup.

	// Setup ADC.
	ADCON1 = 0b01000000;			// Set ADx pin digital I/O.
	ADCON0 = 0b00000001;			// A/D converter module powered up.

	//Configure the LCD
	send_config(0b00000001);		// Clear the LCD.
	send_config(0b00000010);		// Return cursor to home. 
	send_config(0b00000110);		// Entry mode - cursor increase 1.
	send_config(0b00001100);		// Display on, cursor off and cursor blink off.
	send_config(0b00111000);		// Function set.

	lcd_clr();				// Clear the LCD.
	lcd_goto(0);			// Put cursor on position 0.
	send_string("ANGLE=");  // Send string "ANGLE=".
	lcd_goto(11);           // Put cursor on position 11.
	send_string("P=");		// Send string "P=".
	lcd_goto(20);			// Put cursor on position 20.
	send_string("I=");		// Send string "I=".
	lcd_goto(26);			// Put cursor on position 26.
	send_string("D=");		// Send string "D=".


	while(1)
	{
		rec_data = (unsigned long)ai_read(0) * 3600 / 255;	// The angle is limited from 0 to 3600.
		lcd_goto(6);						// Put cursor on position 6.
		send_dec(rec_data, 4);					// Put 4 decimal data on position 6.
										
		kp = ai_read(2);			// Read the value of kp.
		lcd_goto(13);				// Put cursor on position 13.
		send_dec(kp, 3);			// Put the 3 decimal kp value on position 13.

		ki = ai_read(3);			// Read the value of ki.
		lcd_goto(22);				// Put cursor on position 22.
		send_dec(ki, 3);			// Put the 3 decimal ki value on position 13.

		kd = ai_read(4);			// Read the value of kd.
		lcd_goto(28);				// Put cursor on position 28.
		send_dec(kd, 3);			// Put the 3 decimal kd value on position 28.

		//run when SW1 pressed
		if(!SW1)
		{	
			while(!SW1);
			
			//setup Timer1
			TMR1H = 0;			//initialize timer 1
			TMR1L = 0;
			T1CON = 0b00110001;		//Enables Timer1 with 1:8 prescale value
			
			//setup timer1 interrupt
			TMR1IF=0;				//Clear the TMR1 overflow flag.
			TMR1IE=1;				//Enables the TMR1 overflow interrupt
			PEIE=1;					//Peripheral Interrupt Enable bit is set, Enables all unmasked peripheral interrupts
			GIE=1;					//global interrupt enable bit is set, enable all unmasked interupts
			
			while(1);				
		}
	}			
}


// Read the ADC.
unsigned char ai_read(unsigned char an_sel)
{
	if(an_sel == 0)			ADCON0 = 0b00000001;	// AN0 is selected, set ADCON0.
	else if(an_sel == 1)	ADCON0 = 0b00001001;            // AN1 is selected, set ADCON0.
	else if(an_sel == 2)	ADCON0 = 0b00010001;            // AN2 is selected, set ADCON0.
	else if(an_sel == 3)	ADCON0 = 0b00011001;            // AN3 is selected, set ADCON0.
	else if(an_sel == 4)	ADCON0 = 0b00100001;            // AN4 is selected, set ADCON0.
	
	delay(100);						// Wait until the sampling capacitor is charged.
	
	GO=1;							// Start conversion and wait until it is completed.
	while(GO==1);
	
	if(an_sel == 0 || an_sel == 1)	return ADRESH;
	else return ADRESH * 100 / 255;				// Limit the kp,ki and kd to the value of 0 to 100.	
}


// Rotate the motor to the right.
void motor_right(unsigned char speed)
{	
	MOTOR_1	 = 0;			// Set the direction of the motor to right.
	MOTOR_2 = 1;
	MOTOR_SPEED = speed;            // Set the PWM value to control the speed.
}


// Rotate the motor to the left.
void motor_left(unsigned char speed)
{	
	MOTOR_1	 = 1;			// Set the direction of the motor to left.
	MOTOR_2 = 0;
	MOTOR_SPEED = speed;            // Set the PWM value to control the speed.
}


// Brake the motor.
void motor_stop(void)
{
	MOTOR_1	 = 1;			// Brake the motor.
	MOTOR_2 = 1;
	MOTOR_SPEED = 0;		// Set the PWM value to 0.
}


// Delay function. The delay time depends on the given value.
void delay(unsigned long data)
{
	for( ; data > 0; data -= 1);
}


// Send a pulse to the E pin of the LCD.
void e_pulse(void)
{
	E = 1;
	delay(50);
	E = 0;
	delay(50);
}


// Send the configuration the the LCD.
void send_config(unsigned char data)
{
	unsigned char test;
	unsigned char i;
	
	RS = 0;								// Clear RS pin for config mode.
	for(i = 0; i < 8; i++)						// Loop for 8 times. 
	{
		test = (data >> i) & 0b00000001;                        // Shift data to right.
		switch(i)						// Detect a byte of data one bit by one bit.
		{
			case 0:
				if(test == 1)	D0 = 1;
				else			D0 = 0;
			case 1:
				if(test == 1)	D1 = 1;
				else			D1 = 0;
			case 2:
				if(test == 1)	D2 = 1;
				else			D2 = 0;
			case 3:
				if(test == 1)	D3 = 1;
				else			D3 = 0;
			case 4:
				if(test == 1)	D4 = 1;
				else			D4 = 0;
			case 5:
				if(test == 1)	D5 = 1;
				else			D5 = 0;
			case 6:
				if(test == 1)	D6 = 1;
				else			D6 = 0;
			case 7:
				if(test == 1)	D7 = 1;
				else			D7 = 0;
		}
	}
	delay(50);
	e_pulse();
}


// Clear the LCD.
void lcd_clr(void)
{
 	send_config(0x01);
	delay(600);	
}


// Set the location of the LCD cursor.
// If the given value is (0-15) the cursor will be at the upper line.
// If the given value is (20-35) the cursor will be at the lower line.
// Location of the lcd cursor (2X16):
// -----------------------------------------------------
// | |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15| |
// | |20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35| |
// -----------------------------------------------------
void lcd_goto(unsigned char data)
{
 	if(data < 16)
	{
	 	send_config(0x80 + data);
	}
	else
	{
	 	data = data - 20;
		send_config(0xc0 + data);
	}
	delay(200);
}


// Send a character to the LCD.
void send_char(unsigned char data)
{
	unsigned char test;
	unsigned char i;
	RS = 1;						// Set rs for data mode.
	for(i = 0; i < 8; i++)				// Loop for 8 times.
	{
		test = (data >> i) & 0b00000001;        // Shift data to right.
		switch(i)				// Detect data one bit by one bit.
		{
			case 0:
				if(test == 1)	D0 = 1;
				else			D0 = 0;
			case 1:
				if(test == 1)	D1 = 1;
				else			D1 = 0;
			case 2:
				if(test == 1)	D2 = 1;
				else			D2 = 0;
			case 3:
				if(test == 1)	D3 = 1;
				else			D3 = 0;
			case 4:
				if(test == 1)	D4 = 1;
				else			D4 = 0;
			case 5:
				if(test == 1)	D5 = 1;
				else			D4 = 0;
			case 6:
				if(test == 1)	D6 = 1;
				else			D6 = 0;
			case 7:
				if(test == 1)	D7 = 1;
				else			D7 = 0;
		}
	}
	delay(50);
	e_pulse();
}


// Send a string to the LCD.
void send_string(const char *s)
{          
	unsigned char i = 0;
  	while (s && *s)	send_char (*s++);
	delay(300);
}


// Send a decimal number to the LCD.
void send_dec(unsigned long data, unsigned char num_dig)
{	
	if(num_dig >= 10)
	{
		data = data % 10000000000;
		send_char(data / 1000000000 + 0x30);
	}	
	if(num_dig >= 9)
	{
		data = data % 1000000000;
		send_char(data / 100000000 + 0x30);
	}	
	if(num_dig >= 8)
	{
		data = data % 100000000;
		send_char(data / 10000000 + 0x30);
	}	
	if(num_dig >= 7)
	{
		data = data % 10000000;
		send_char(data / 1000000 + 0x30);
	}	
	if(num_dig >= 6)
	{
		data = data % 1000000;
		send_char(data / 100000 + 0x30);
	}	
	if(num_dig >= 5)
	{
		data = data % 100000;
		send_char(data / 10000 + 0x30);
	}	
	if(num_dig >= 4)
	{
		data = data % 10000;
		send_char(data / 1000 + 0x30);
	}
	if(num_dig >= 3)
	{
		data = data % 1000;
		send_char(data / 100 + 0x30);
	}
	if(num_dig >= 2)
	{
		data=data % 100;
		send_char(data / 10 + 0x30);
	}
	if(num_dig >= 1)
	{
		data = data % 10;
		send_char(data + 0x30);
	}
}


//	Interrupt function
//==========================================================================
static void interrupt isr(void)
{
	signed int set_value = 0;
	signed int feedback_value = 0;
	signed long error_value = 0;
	static signed long pre_error = 0;
	static signed long integral = 0;
	signed long derivative = 0;
	signed long output = 0;
	signed int motor_direction = 0;
	
	
	unsigned char pwm_value=0;	

	if(TMR1IF == 1)                                            // If TMR1 register overflowed.
	{
		TMR1IF = 0;                                        // Clear TMR1 overflow flag.
		
		feedback_value = ai_read(1);                       // Get the feedback value.
		set_value = ai_read(0);                            // Get the set value.
		
		error_value = set_value - feedback_value;          // Calculate the error.
		integral = integral + error_value;                 // Calculate integral.
		derivative = error_value - pre_error;              // Calculate derivative.
		
		output = (kp * error_value) + (ki * integral) + (kd * derivative);  // Calculate the output, pwm.
		
		if (output > 255) output = 255;					// Limit the output to maximum 255.
		else if (output < -255) output = -255;
		
		if (output > 0)	motor_right((unsigned char)output);		// When output is positive, turn right.
		else if (output < 0) motor_left((unsigned char)(-output));	// When error is negative, turn left.
		else motor_stop();						// When ouput is 0, stop the motor.
		
		pre_error = error_value;					// Save as previous error.
	}
}

