

// ============================== includes ================================ //
//#include <limits.h>
//#include <ctype.h>
//#include <stdarg.h>
#include <stdio.h>
//#include <avr/io.h>
//#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#define  F_CPU 15000000L     				// required by delay.h to set proper timing (remaining
											//   CPU MIPS are used by interrupt)
#include <util/delay.h>
#include "5x7font.h"						// font required for graphic LCD
#include "stdlib.h"							// required for rand() function

// ============================== defines ================================= //

// Firmware revision: "1.002"
#define FIRM_REV_MAJ    1
#define FIRM_REV_MIN    2

// Firmware status
// Must be:
//  'a' - Alpha
//  'b' - Beta
//  'T' - Test Build
//  'R' - Release
#define FIRM_REV_STAT   'R'


#define MAGIC_NUMBER 42

// I/O related
#define set_A0  PORTB |= (1<<PB3)      		// -->A0 = 1
#define clear_A0 PORTB &= ~(1<<PB3)    		// -->A0 = 0
#define Red_LED_On PORTD |= (1<<PD5)		// turn on red LED
#define Red_LED_Off PORTD &= ~(1<<PD5)		// turn off red LED
#define Red_LED_Toggle PORTD ^= (1<<PD5)	// toggle red LED
#define Green_LED_On PORTD |= (1<<PD6)		// turn on green LED
#define Green_LED_Off PORTD &= ~(1<<PD6)	// turn off green LED
#define Green_LED_Toggle PORTD ^= (1<<PD6)	// toggle green LED
#define Speaker_Toggle PORTD ^= (1<<PD7)	// toggle speaker output

// DDS related
#define DDS_DIVISOR			12500			// 1/2 of the timer 0 interrupt frequency; used to
											//   determine when a DDS accumulator should overflow
											//   and flag a DDS event
#define DDS_BRAKE_FREQ		200				// Frequency that PWM is applied in brake mode

// Motor related
#define MOTOR_ZERO_POSITION 0x80000000		// "zero" position for motors

// BUMP BOT related
#define BUMP_TIME		2750				// Used to time how long to back the bot and turn the bot
#define BUMP_FORWARD	0					// Display mode for bump bot
#define BUMP_RIGHT		1					// Display mode for bump bot
#define BUMP_LEFT		2					// Display mode for bump bot
#define BUMP_BACKUP		3					// Display mode for bump bot


// Menu related
#define MODE_MAX        0x02				// the maximum number of modes
#define MENU_MODE       0x00				// Brings up the menu screen
#define BUMP_MODE       0x01				// runs the bot in bump bot mode
#define TANK_MODE       0x02				// uses PSX controller to run the bot in tank mode
#define CAR_MODE        0x03				// uses PSX controller to run bot in car mode
#define SINGLE_MODE     0x04				// Uses a single left stick to control bot
#define DEAD_MODE       0x05				// Lets the User select motor, speed, time of travel

#define DISPLAY_MAX     0x06				// The maximum number of display options
#define BATT_DISP       0x00				// Displays battery information
#define INST_DISP       0x01				// Displays instructions for the current mode
#define MODE_DISP       0x02				// Displays Mode it is in
#define CTRL_DISP       0x03				// Displays PSX controller information
#define WHEEL_DISP      0x04				// Displays wheel revolutions
#define SPEED_DISP      0x05				// Displays analog/digital speed info
#define FIRMWARE_DISP   0x06				// Displays firmware levels


#define NUMBER_MAX 255


// LCD related
#define LCD_COLUMN_SET_H    0X10
#define LCD_COLUMN_SET_L    0X00
#define LCD_PAGE_SET        0xB0

#define LCD_X_MAX 128						// setting for a 128 pixel wide LCD, can be changed
											//   for larger or smaller screens
#define LCD_Y_MAX 4							// setting for a 4 byte (32 pixel) high LCD, can be changed
											//   for larger or smaller screens
#define BATT_0BAR		0x80				// battery 0%
#define BATT_1BAR		0x81				// battery 17%
#define BATT_2BAR		0x82				// battery 33%
#define BATT_3BAR		0x83				// battery 50%
#define BATT_4BAR		0x84				// battery 67%
#define BATT_5BAR		0x85				// battery 83%
#define BATT_6BAR		0x86				// battery 100%
#define BATT_CHG 		0x87				// battery charging (up arrow)
#define BATT_AC 		0x88				// AC powered (small AC)
#define BATT_NEEDCHG	0x89				// battery needs charging (double !!)


// Battery related
#define RESET				0x00
#define BATT_WARNING		0x01
#define BATT_SHUTDOWN		0x02
#define CHARGING		 	0x03
#define CHARGED				0x04
#define AC_POWERED			0x05

#define BATT6V0				12738			// correlates to 6.0V: 0.000471V/ADC LSB
#define BATT6V4				13588			// correlates to 6.4V: 0.000471V/ADC LSB
#define BATT6V8				14437			// correlates to 6.8V: 0.000471V/ADC LSB
#define BATT7V2				15287			// correlates to 7.2V: 0.000471V/ADC LSB
#define BATT7V6				16136			// correlates to 7.6V: 0.000471V/ADC LSB
#define BATT8V0				16985			// correlates to 8.0V: 0.000471V/ADC LSB
#define BATT10V0			21231			// correlates to 10.0V: 0.000471V/ADC LSB
#define BATT10V8			22930			// correlates to 10.8V: 0.000471V/ADC LSB
#define BATT11V0			23355			// correlates to 11.0V: 0.000471V/ADC LSB

#define BATT6V0LONG			26092723		// correlates to 6.0V: 0.223uV/ADC LSB (for added 16-bit avg)

#define CHG6V0				14286			// correlates to 6.0V: 0.00042V/ADC LSB


//#define MAX_PWM_VAL		0x0226			// roughly correlates to 2A charge w/3A charger when fully charged (9.1V)
//#define START_PWM_VAL		0x0120			// 
//#define MIN_PWM_VAL		0x0080			// roughly correlates to xxmA charge w/3A charger
#define MAX_PWM_VAL			0x0420			// roughly correlates to 2A charge w/3A charger when fully charged (9.1V)
#define START_PWM_VAL		0x0229			// 
#define MIN_PWM_VAL			0x00f6			// roughly correlates to xxmA charge w/3A charger

#define MAX_BATT_CHG		4800			// 4800 = 4800mAH
#define MAX_CHG_TIME		480				// 480min = 8 hrs
#define BATT_WARN_TIME		1				// 1 min
#define MIN_CHG_CURRENT 	0x3f20			// min charge current ~ 50mA idle charge
#define RISE_CHG_CURRENT	0x46b5			// approx 500mA
#define FAST_CHG_CURRENT	0x4f81			// approx 1000mA
#define MAX_CHG_CURRENT		0x536c			// max charge current values for uiADC_History[0]
											// 0x4f13=-0.997A, 0x4f39=-1.005a, 0x4f8d=-1.021A,
											// 0x51a0=-1.144A, 0x536c=-1.25a, 0x57bc=-1.5A,
											// 0x5c0c=-1.75A,  0x605c = -2.00a
#define ZERO_CURRENT		0x3de9			// if less than ZERO_CURRENT then battery not present
											// 0x3e66=-0.010a, 0x3e3b=0.001a, 0x3e0f=0.011a,
											// 0x3de9=0.020a, 0x3d20=0.067a


// PSX remote related
#define DIGITAL_MODE_BYTE   0x41	        // Byte indicator for 'Digital-Mode' data.
#define ANALOG_MODE_BYTE    0x73        	// Byte indicator for 'Analog-Mode' data. (Red Mode).
#define DIGITAL_TURN_SPEED  0x73            // Set a digital speed for bump bot and digital PSX mode
#define MIN_ANALOG_SPEED    0x0a			// Minimum analog bot speed
#define MAX_ANALOG_SPEED    0xc8			// Maximum analog bot speed
#define MIN_DIGITAL_SPEED   0x0a			// Minimum digital bot speed
#define MAX_DIGITAL_SPEED   0x96			// Maximum digital bot speed
#define ANALOG_SPEED        0x73            // Set an analog speed for tank mode
#define PSX_LOW_THRESHOLD	0x78			// Define dead band for PSX analog stick
#define PSX_HIGH_THRESHOLD	0x8e			// Define dead band for PSX analog stick

// Controller bit definitions. (5th byte response 'response[ 4 ]').
#define L2_BIT              0x01
#define R2_BIT              0x02
#define L1_BIT              0x04
#define R1_BIT              0x08
#define TRI_BIT             0x10
#define CIR_BIT             0x20
#define X_BIT               0x40
#define SQR_BIT             0x80

// Other button bit definitions. (4th byte response 'response[ 3 ]').
// Except for the 'ANALOG ONLY 'mode, these bits are present in both digital
// mode and analog mode (Red Mode).
#define SLCT_BIT            0x01
#define JOYR_BIT            0x02        	// Analog mode only
#define JOYL_BIT            0x04        	// Analog mode only
#define STRT_BIT            0x08
#define DPUP_BIT            0x10
#define DPRGT_BIT           0x20
#define DPDWN_BIT           0x40
#define DPLFT_BIT           0x80

// ========================== function prototypes ========================= //
void ATmega324_init(void);
void SPI_Transmit( unsigned char, unsigned char ); // transmit a byte to addr (master mode) (addr, data)
unsigned char SPI_Receive( unsigned char, unsigned char ); // receive a byte from addr, dummy transmit data (master mode)
void Set_Slave_Addr(unsigned char);			// set 74HC138 slave select address lines
void Spkr_chirp( void );					// make a chirp, chirp sound from speaker
void RC_Servo(unsigned char, unsigned char, unsigned char, unsigned char, unsigned char); // send servo data
void PWMcharger(unsigned int);				// 
void (*funcptr)( void ) = 0x0000; 			// Set up function pointer to RESET vector.
unsigned int Calc_Analog_Speed( unsigned char ); // calculate bot analog speed

// Bump Bot related
void Turn_Bot( unsigned char );             // Turns the bot for BUMP_TIME ms
void Back_Bot( void );                      // Backs the bot up for BUMP_TIME ms

// LCD related
void LCD_init(void);
void LCD_write_cmd(unsigned char cmd);		// primitive for writing commands to LCD
void LCD_write_data(unsigned char dat);		// primitive for writing data to LCD
void LCD_BL(unsigned char); 				// set LCD backlight intensity
static int LCD_putchar(char c, FILE *stream); // Write a character directly to the LCD without using
											//   the 324 LCD buffer
void LCD_putcharXY(unsigned char, unsigned char, unsigned char); // Write a character at X,Y directly to the LCD without using
											//   the 324 LCD buffer
static FILE mystdout = FDEV_SETUP_STREAM(LCD_putchar, NULL, _FDEV_SETUP_WRITE); // to support printf stream redirection
void LCD_clear( void );						// calls LCDbuf_clear and sends to LCD
void LCD_splash( void );					// display splash intro
void Display_data( void );                  // display data to the screen


// CPU clock related
void clk_rate ( char );						// set CPU clock rate divisor (freq MHz)

// PSX controller related
void Process_PSX( void );					// perform different functions based on PSX response[] global array
void Read_PSX( void );						// read PSX controller and return data in response[] global array
void Set_PSX_Analog( void );				// force PSX into anaog mode

// ATtiny slave I/O related
void Process_Sensor_Data( void );			// Process ATtiny switch presses
unsigned char Read_Sensors( void );			// read sensor data from ATtiny
unsigned int Read_Tiny_Rev( void );			// read revision level from ATtiny


// ================================ globals =============================== //
// LCD related
volatile char current_column = 0;			// keeps track of the current LCD column address
volatile char current_page = 3;				// keeps track of the current LCD row (page 3=top, 0=bottom)

// Menu related
volatile unsigned char ucOpMode = 0;		// keeps track of the mode that the bot is in
											//    Mode 0: menu select
											//    Mode 1: Battery Stats
											//    Mode 2: Bump Bot
											//    Mode 3: PSX controlled
											//    Mode 4: Sleep Mode
volatile unsigned char ucDisplayMode = 2;	// keeps track of the display the bot is in
volatile unsigned char ucBumpMode;			// 0=forward, 1=turn, 2=back up

// Interrupt related
volatile unsigned char ucInt2Phase;			// timer 2 interrupt phase counter 0..7

// DDS Related
volatile unsigned int DDS_Motor_L_Accum;	// DDS accumulator for Motor L
volatile unsigned int DDS_Motor_R_Accum;	// DDS accumulator for Motor R
volatile unsigned int DDS_Brake_Accum;		// DDS brake accumulator
volatile unsigned int DDS_L_Pulse_Timer;	// DDS count down timer for Motor L;
											//   when 0, turn off motor to save power
volatile unsigned int DDS_R_Pulse_Timer;	// DDS count down timer for Motor R;
											//   when 0, turn off motor to save power
volatile unsigned int DDS_spkr_Accum = 0;	// DDS accumulator for speaker
volatile unsigned char DDS_scratch_reg;		// scratch pad register for DDS functions

// Speaker Related
volatile unsigned int Spkr_Freq = 0; 		// speaker frequency (Hz)
volatile unsigned int uiSpkr_Timer = 0;		// length of time to allow speaker beep

// Stepper Motor-Related
// Stepper phase pattern LUT.  It holds the pattern used to control the H-bridges that
//   control the Bipolar Stepper Motor.
const char Motor_L_LUT[] = { 0b00010000,
						 	 0b00000100,
							 0b00001100,
							 0b00011000 };
const char Motor_R_LUT[] = { 0b11000000,
							 0b01100000,
						 	 0b00100000,
							 0b10000000 };
volatile int Motor_PWM_LUT[] = { 1700, 1600, 1500, 1400, 1300, 1200, 1100, 1000, 900,
							 800, 700, 600, 500 }; 

volatile unsigned char Motor_Accel_Rate;	// Motor acceleration rate								  
volatile unsigned char Motor_Decel_Rate;	// Motor deceleration rate								  
volatile unsigned char ucMotor_L_Speed;		// speed of Motor L in pulses / sec
volatile unsigned char ucMotor_R_Speed;		// speed of Motor R in pulses / sec
volatile unsigned int Motor_L_Accel_Reg;	// internal variable - not for general use
											//   current motor speed based on acceleration
volatile unsigned int Motor_R_Accel_Reg;	// internal variable - not for general use
											//   current motor speed based on acceleration
volatile unsigned long ulMotor_L_Pos;		// position Motor L is to travel to
volatile unsigned long ulMotor_R_Pos;		// position Motor R is to travel to
volatile unsigned long ulMotor_L_Pos_Reg;	// internal variable - not for general use
											//   current motor position
volatile unsigned long ulMotor_R_Pos_Reg;	// internal variable - not for general use
											//   current motor position
volatile unsigned char ucMotor_L_Dir;		// Motor L direction 1=CW, 0=CCW
volatile unsigned char ucMotor_R_Dir;		// Motor R direction 1=CW, 0=CCW
volatile unsigned char ucMotor_L_State;		// Motor L phase state (0x00-0x03)
volatile unsigned char ucMotor_R_State;		// Motor R phase state (0x00-0x03)
volatile unsigned long Motor_L_Revs;		// Motor L revolutions
volatile unsigned long Motor_R_Revs;		// Motor R revolutions
volatile unsigned char ucBrakeMode;			// 1=brakes, 0=no brakes

// Battery and Power related
volatile unsigned int	uiPWMval;				// charger PWM value seeking 1A charge
volatile unsigned char	ucADC_Read_State = 0; // ADC Channel currently being sampled (0x00-0x02)
volatile unsigned int	uiADC_History[4];	// ADC accumulated history of channel 0-3
volatile unsigned int	uiADC_Instant[4];	// ADC instantaneous value for channel 0-3
volatile unsigned int	uiADC_avg_reg;
volatile unsigned long	lBatt_History;		// 2^11 * 2^5 oversampling of battery voltage
volatile long 			lBatt_PrevHistory;	// comparison value of 2^11 * 2^5 oversampling of battery voltage
volatile long			lBatt_Difference;	
volatile unsigned long ulADC_avg_reg;
volatile unsigned int  uiBattCtr;			// Counter/timer for displaying battery status and analyzing system power
volatile double 		dMaH_ctr;			// mAh counter
volatile unsigned char ucChgChkTime; 		// ADC Channel currently being sampled (0x00-0x02)
volatile unsigned char ucChgSecCtr;			// Timeout second counter for charge system
volatile unsigned int  uiChgMinCtr;			// Timeout minute counter for charge system
volatile unsigned char ucBattDispState = 0x80; // Battery charger disp state for battery bargraph
volatile unsigned char ucBattChgState = RESET; // Battery charger state
volatile unsigned int  uiBattCurrent;		// instantaneous battery current (not averaged)
volatile unsigned char ucPowerStatus; 		// System power status (1=yes, 0=no)
											//    bit 7 = battery present
											//    bit 6 = battery power switch on
											//    bit 5 = charger present
											//    bit 4 = charging
											//    bits 3-0 not used
//volatile unsigned char Batt_int_timer = 62; // Timer for rate of battery check

//volatile double Batt_threshold_low = 6.0;	// if battery is initially below 6V then it needs charging

// SPI related
volatile unsigned char SPI_data;
volatile unsigned int uiTinyRev;			// ATtiny firmware revision level

// PSX remote related
volatile uint8_t response[ 21 ];    		// Data storage buffer for incoming PSX data
volatile unsigned char analog = 0;			// status byte: analog=1, digital=0
volatile unsigned int uiPSX_TimeoutCtr; 	// PSX inactivity timeout counter (start with non-zero count)
											//   0 = valid data
											//   <> 0 = invalid data
volatile unsigned char ucPSX3Change;		// PSX byte 3 status: if any bit is a "1", this bit has changed
volatile unsigned char ucPSX4Change;		// PSX byte 4 status: if any bit is a "1", this bit has changed
volatile unsigned char ucPSX3History;		// PSX byte 3 history: value of PSX byte 3 from previous read
volatile unsigned char ucPSX4History;		// PSX byte 4 history: value of PSX byte 4 from previous read
volatile unsigned char ucBotAnalogSpeed;	// max bot speed (analog)
volatile unsigned char ucBotDigitalSpeed;	// max bot speed (digital)


// PSX Poll command (with padded data).
volatile const uint8_t poll[ 21 ] = {		//main poll cmd
	0x01,0x42,0x00,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00};
volatile const uint8_t poll3[ 5 ] = {		//Enter Config Mode
	0x01,0x43,0x00,0x01,0x00 };
volatile const uint8_t poll4[ 9 ] = {		//Exit Config Mode
	0x01,0x43,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00};
volatile const uint8_t poll2[ 9 ] = {		//Force Analog On
	0x01,0x44,0x00,0x01,0x03,
    0x00,0x00,0x00,0x00};
/*
volatile const uint8_t poll5[ 9 ] = {		// Turn Motors On  
	0x01,0x4D,0x00,0x00,0x01,               // Haven't Figured this out yet
    0xFF,0xFF,0xFF,0xFF};
*/

// volatile unsigned char x_temp = 0;		// keeps track if X was pushed or not

// ATtiny slave I/O related
volatile unsigned char ucSensorStatus;		// switch and IR bump sensor data


// misc
volatile unsigned char ucTest;				// 

