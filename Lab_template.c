    // FIRMWARE VER: 1.0T (Test)
//
// NOTE:  Make sure the firmware version above is also reflected in the 
//        '#define' section below.  THESE MUST MATCH!
// ============================================================================
// Auth: Dennis Deyen   (Original Author, Initial Revision)
//
// Desc: Firmware code for the ATmega324 Smart IR Sensor
//
// Target Device:   ATmega324
// MCU Clock Speed: 20MHz (xtal)
//
// ============================================================================
//
// Revision History:
// =================
//
// 10/3/09 - by D. Deyen
// ----------------------
//
// 1/13/10 - by J. Lind
// ----------------------
//
// 07/06/10 - by W. Wang 
//
// ============================================================================
/* 
 Pin Definitions and Assignments on ATmega324
// ============================================================================
PA0		Battery current monitor
PA1		Battery voltage monitor
PA2		Charger voltage monitor
PA3		general purpose I/O and ADC4 pin on J2
PA4		general purpose I/O and ADC4 pin on J2
PA5		general purpose I/O and ADC5 pin on J2
PA6		general purpose I/O and ADC6 pin on J2
PA7		general purpose I/O and ADC7 pin on J2
 -------------------------------------------
PB0		SPI Slave Select address 0 - fed to a 74HC138 demultiplexer
PB1		SPI Slave Select address 1 - fed to a 74HC138 demultiplexer
PB2		SPI Slave Select address 2 - fed to a 74HC138 demultiplexer
		PB2  PB1  PB0
		 0    0    0	SPI Slave Select 0 - graphic LCD display
		 0    0    1	SPI Slave Select 1 - PSX remote control
		 0    1    0	SPI Slave Select 2 - ATtiny48 auxiliary microcontroller
		 0    1    1	SPI Slave Select 3 - available on expansion connector J2
		 1    0    0	SPI Slave Select 4 - smart peripheral J4, also available on expansion connector J2
		 1    0    1	SPI Slave Select 5 - smart peripheral J5, also available on expansion connector J2
		 1    1    0	SPI Slave Select 6 - not wired out
		 1    1    1	SPI Slave Select 7 - not wired out
PB3		graphic LCD display A0; selects between register and data modes on LCD
PB4		1=enable 5V high speed operation, 0=low power standby mode (2.7V processor voltage)
PB5		SPI MOSI: serial data output pin
PB6		SPI MISO: serial data input pin
PB7		SPI SCLK: serial clock output pin
 -------------------------------------------
PC2		Motor 1 output pin A
PC3		Motor 1 output pin B
PC4		Motor 1 output pin C
PC5		Motor 2 output pin A
PC6		Motor 2 output pin B
PC7		Motor 2 output pin C
 -------------------------------------------
PD5		Red LED 1=on, 0=off
PD6		Green LED 1=on, 0=off
*/

// ============================== includes ================================ //
#include "324V221HW.h"          // this include defines all of the prototypes
#include "oliverHeader.h"	      //   required for the this file


/******************************************************************************
*                                                                             *
*  Function: Main                                                             *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Main function                                                *
*                Interrupts take about 22.5% of CPU execution resources,      *
*                leaving 77.5% for main (user code).                          *
******************************************************************************/
int main (void)
{
	//----------------Initialization Begin, Do Not Delete----------------------------
	int i;
	char * string_array = "-CEENBoT-";
	clk_rate(20);							// go to 20MHz operation
	LCD_splash();							// display splash screen
	LCD_clear();							// clear LCD display

	ucMotor_L_Dir = 0;						// set motor L direction
	ucMotor_R_Dir = 0;						// set motor R direction
	ucMotor_L_Speed = 0;					// set motor L speed
	ucMotor_R_Speed = 0;					// set motor R speed

	ucOpMode = TANK_MODE;
	ucDisplayMode = BATT_DISP;				// was MODE_DISP
	LCD_BL(31);								// set LCD backlight brightness
	//----------------Initialization Finish, Do Not Delete----------------------------


	//--------Motion Operations should be here-------------------------------------------------------
const int SPEED = 75;


ucMotor_L_Speed = SPEED;
ucMotor_R_Speed = SPEED;      //Move forward (downward) [left vertical strip of "U"]
_delay_ms(5000);

TurnLeft(SPEED);
ucMotor_L_Speed = SPEED;
ucMotor_R_Speed = SPEED;

ucMotor_L_Speed = SPEED;
ucMotor_R_Speed = SPEED;      //Move forward (upward) [right vertical strip of "U"]
_delay_ms(5000);

backTheHellUp(SPEED);
_delay_ms(5000);

turn90(R, 90);
ucMotor_L_Speed = SPEED;
ucMotor_R_Speed = SPEED; 


//After finishing the U, play song
playRingTheBell();  //Play "Ring the Bell"




}


/******************************************************************************
*                                                                             *
*  Function: Display_data                                                     *
*                                                                             *
*  Input: None                                                                *
*                                                                             *
*  Output: Writes Data to the screen                                          *
*                                                                             *
*  Globals Read: display                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Uses the variable display to write certain information       *
*                   to the screen. Changes when O is pressed on the PSX       *
*                   Controller.                                               *
*                                                                             *
******************************************************************************/
void Display_data(void)
{
	current_column = 0;						// home cursor position
	current_page = 3;

    switch(ucDisplayMode)					// Checks to see what needs to display
    {
        case BATT_DISP:						// If dipslay == BATT_DISP, then output battery data
			printf( "   Battery Monitor\t" );
//			printf( "Vb Long: %2.6fV\t", (lBatt_History * 0.00000023 ));
//			printf( "%ld\t", lBatt_Difference );


			// ADC[1]:  Vin = 4.97V*(1475k/475k)/1024 = 15.433V/1024 = 15.0715mV * ADC 
			//   since we are oversampling by a factor of 2^5, divide 15.0715mV by 32
			// ADC[0]:  Vin = 4.97V*(14.75k/4.75k)/1024 = 15.433V/1024 = 15.0715mV * ADC 
			//   since we are oversampling by a factor of 2^5, divide 15.0715mV by 32
			printf( "Bat: %2.3fV %1.3fA\t", uiADC_History[1] * 0.000471, (int)( (498 << 5) - uiADC_History[0] ) * 0.0002306 );


			switch(ucBattChgState)
			{
				case RESET:
					printf("RUN MODE: %dm %2ds\t", uiChgMinCtr, ucChgSecCtr);
					printf( "Power Usage: %.0fmAh", dMaH_ctr );
					break;
				case BATT_WARNING:
					printf("LOW BATT!\t");
					printf("Shutdown %dsec", 60-ucChgSecCtr);
					break;
				case BATT_SHUTDOWN:
					printf("SHUTDOWN!\t");
					break;
				case CHARGING:
					printf("CHARGING: %dm %2ds\t", uiChgMinCtr, ucChgSecCtr);
					printf( "Chg: %.0fmAh (P%d)", dMaH_ctr, uiPWMval );
					break;
				case CHARGED:
					printf("CHARGED\t");
					printf( "Charged: %.0fmAh", dMaH_ctr );
					break;
				case AC_POWERED:
					printf("AC POWER\t");
					break;
			}
			printf( "\t" );
			break;

        case CTRL_DISP:									// If display == CTRL_DISP, then output PSX controller data
			printf(" PSX Ctrlr Read Data   ");
			for( uint8_t i = 3 ; i < 6 ; ++i )			// Poll the PSX controller for data.
			{
				printf("%d:%x ", i, response[ i ]);
	    	} // end for()
			printf("\t  ");
			for( uint8_t i = 6 ; i < 9 ; ++i )			// Poll the PSX controller for data.
    	    {
  				printf("%d:%x ", i, response[ i ]);
		    } // end for()
			printf("\t");								// clear to EOL
			if (uiPSX_TimeoutCtr)						// if invalid PSX data, display blanks
				printf( "\t" );							// clear to EOL
			else
				if ( response[ 1 ] == DIGITAL_MODE_BYTE )
					printf( "  Set to Digital Mode" );
				else
					printf( "  Set to Analog Mode " );
            break;
        case MODE_DISP:									// IF display == MODE_DISP, then output the current mode
            switch(ucOpMode)
            {
//				case CAR_MODE:
//					printf( "<<<    Car Mode   >>>" );
//					break;
				case TANK_MODE:
					printf( "------Tank Mode------" );
					break;
//				case SINGLE_MODE:
//					printf( "<<< Joystick Mode >>>" );
//					break;
				case BUMP_MODE:
					printf( "--Bump Bot: " );
					if (ucBumpMode == BUMP_FORWARD)
						printf("FORWARD--");
					else if (ucBumpMode == BUMP_RIGHT)
						printf("TURN RT--");
					else if (ucBumpMode == BUMP_LEFT)
						printf("TURN LT--");
					else
						printf("REVERSE--");
					break;
				default:
//					printf( "<<< UNKNOWN MODE  >>>" );
					break;
            }
            printf( " <X> Switch Activity " );
            printf( " <L1> and <R1> Switch" );
            printf( "      displays       " );
            break;
        case INST_DISP:     							// IF display == INST_DISP, then output instructions of the current mode
            switch(ucOpMode)
            {
//				case CAR_MODE:
//					printf( "Left stick controls  " );
//					printf( "the speed. Right     " );
//					printf( "stick controls the   " );
//					printf( "direction.           " );
//					break;
				case TANK_MODE:
					printf( "Left stick controls  " );
					printf( "the left motor.      " );
					printf( "Right stick controls " );
					printf( "the right motor.     " );
					break;
//				case SINGLE_MODE:
//					printf( "Left stick controls  " );
//					printf( "the whole bot.       " );
//					printf( "Up/Down is speed.    " );
//					printf( "Left/Right is dir.   " );
//					break;
				case BUMP_MODE:
					printf( "Bot will go forward  " );
					printf( "until it detects     " );
					printf( "something in its path" );
					printf( "Then it will turn.   " );
					break;
				default:
//					printf( "    UNKNOWN INST     " );
//					printf( "                     " );
//					printf( "                     " );
//					printf( "                     " );
					break;
			}
			break;
        case WHEEL_DISP:								// IF display == WHEEL_DISP, then display wheel revolutions
				printf( "Wheel Revolutions\t" );
				printf( "Decimal=Binary\t" );
				printf( "L: %4ld=", Motor_L_Revs/200 );
				for( uint8_t i = 0 ; i < 10 ; i++ )		// Print binary of wheel revs
    		    {
					if ((Motor_L_Revs / 200) & (1<<(9 - i)))
						printf("1");
					else
						printf("0");
		    	} // end for()
				printf( "\t" );

				printf( "R: %4ld=", Motor_R_Revs/200 );
				for( uint8_t i = 0 ; i < 10 ; i++ )		// Print binary of wheel revs
    		    {
					if ((Motor_R_Revs / 200) & (1<<(9 - i)))
						printf("1");
					else
						printf("0");
		    	} // end for()
				printf( "\t" );
				break;
        case SPEED_DISP:     							// IF display == SPEED_DISP, then display analog/digital speeds
				printf( "Speed Settings\t" );
				printf( " Analog: %d\t", ucBotAnalogSpeed );
				printf( " Digital: %d\t", ucBotDigitalSpeed );
				printf( "\t" );
				break;
        case FIRMWARE_DISP:    							// IF display == FIRMWARE_DISP, then display firmware levels
				printf( "Firmware Level \t" );
				printf( " ATmega: %d.%03d%c\t", FIRM_REV_MAJ, FIRM_REV_MIN, FIRM_REV_STAT );
				printf( " ATtiny: %d.%03dR\t", ( uiTinyRev >> 8 ), uiTinyRev & 0xff );
				printf( "\t" );
				break;
		default:
		break;
	}
	Set_Slave_Addr( 7 );								// set to something other than display addr
}														//   to give interrupts time to display stuff


/******************************************************************************
*                                                                             *
*  Function: Back_Bot                                                         *
*                                                                             *
*  Input: None                                                                *
*                                                                             *
*  Output: None                                                               *
*                                                                             *
*  Globals Read: (None)                                                       *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Moves the bot backwards for a certain amount of time         *
*                   BUMP_TIME is the amount of time moved backwards           *
*                   BUMP_TIME = 200                                           *
*                                                                             *
******************************************************************************/
void Back_Bot( void )
{
	unsigned int j;
    ucMotor_L_Speed = 0;
    ucMotor_R_Speed = 0;
    _delay_ms(BUMP_TIME/2000);
    ucMotor_L_Dir = 1;
    ucMotor_R_Dir = 1;									// Set the motors to backwards
    ucMotor_L_Speed = ucMotor_R_Speed = ucBotDigitalSpeed;	// Set bot speed

	ucBumpMode = BUMP_BACKUP;
	Display_data();

	// delay during backup using PSX read.  If <X> pressed, exit time delay early
	for( j = 0 ; j < 266 ; j++ )
	{
		ucSensorStatus = Read_Sensors();				// switch and IR bump sensor data
		Process_Sensor_Data();							// determine what to do with pressed switches

		Read_PSX();										// Read PSX controller data 
		if (ucOpMode == TANK_MODE)
			j = 1000;
	}

    ucMotor_L_Speed = 0;                  
    ucMotor_R_Speed = 0;									// Turn the motors off
}


/******************************************************************************
*                                                                             *
*  Function: Turn_Bot                                                         *
*                                                                             *
*  Input: a char telling which way to turn the bot                            *
*                                                                             *
*  Output: None                                                               *
*                                                                             *
*  Globals Read: BUMP_TIME                                                    *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Turns the bot a direction based on it                        *
*                   if the input mod 2 is 0, turn bot right                   *
*                   else turn it left                                         *
*                                                                             *
******************************************************************************/
void Turn_Bot(unsigned char dir)
{
	unsigned int j;

    if(dir%2 == 0)										// Turning Right
    {
        ucMotor_L_Dir = 0;								// Set the left motor to move forward 
        ucMotor_R_Dir = 1;								// Set the right motor to move backward
		ucBumpMode = BUMP_RIGHT;
    }
    else //if(dir%2 == 1)      			    	 		// Turning Left
    {
        ucMotor_L_Dir = 1;								// Set the left motor to move backward
        ucMotor_R_Dir = 0;								// Set the right motor to move forward
		ucBumpMode = BUMP_LEFT;
    }
    ucMotor_L_Speed = ucMotor_R_Speed = DIGITAL_TURN_SPEED;	// Set bot speed for 90 degree turn

	Display_data();

	// delay during turn using PSX read.  If <X> pressed, exit time delay early
	for( j = 0 ; j < 200 ; j++ )
	{
		ucSensorStatus = Read_Sensors();				// switch and IR bump sensor data
		Process_Sensor_Data();							// determine what to do with pressed switches

		Read_PSX();										// Read PSX controller data 
		if (ucOpMode == TANK_MODE)
			j = 1000;
	}

//    _delay_ms(BUMP_TIME/4);							// Wait for the bot to finish turning (200 ms)
    ucMotor_L_Speed = 0;
    ucMotor_R_Speed = 0;									// Turn the motors off
}



/******************************************************************************
*                                                                             *
*  Function: LCD_write_cmd                                                    *
*                                                                             *
*  Input: unsigned char LCD command                                           *
*                                                                             *
*  Output: Sends a command to the graphic LCD display                         *
*                                                                             *
*  Globals Read: (None)                                                       *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Sends a command to the graphic LCD display                   *
*                                                                             *
******************************************************************************/
void LCD_write_cmd(unsigned char cmd)
{
    clear_A0; 											// <--0 for command
    _delay_us(1);										// delay to stabilize A0
    SPI_Transmit(0x00, cmd);							// send to SPI (address, data) LCD=addr 0
}


/******************************************************************************
*                                                                             *
*  Function: LCD_write_data                                                   *
*                                                                             *
*  Input: unsigned char LCD data                                              *
*                                                                             *
*  Output: Sends a byte of data to the graphic LCD display                    *
*                                                                             *
*  Globals Read: (None)                                                       *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Sends a byte of data to the graphic LCD display              *
*                                                                             *
******************************************************************************/
void LCD_write_data(unsigned char dat)
{
	set_A0;    											// <--1 for data
	_delay_us(1);										// delay to stabilize A0
	SPI_Transmit(0x00, dat);							// send to SPI (address, data) LCD=addr 0
    clear_A0;
}


/******************************************************************************
*                                                                             *
*  Function: LCD_clear                                                        *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: Writes blank bytes of data directly to LCD.                        *
*                                                                             *
*  Globals Read: (None)                                                       *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Clears graphic LCD display                                   *
*                                                                             *
******************************************************************************/
void LCD_clear(void)
{
	unsigned char i,j;
	LCD_write_cmd(0x40);
	for(i=0; i<4; i++)									// clear page 0~3
	{
		LCD_write_cmd(LCD_PAGE_SET+i);					// set page
		LCD_write_cmd(LCD_COLUMN_SET_L);				// set column low nibble
		LCD_write_cmd(LCD_COLUMN_SET_H);				// set column high nibble
		for(j=0; j<LCD_X_MAX; j++)						// clear all columns up to 128
		{     
			LCD_write_data(0x00);
			if ((i==3) && (j > (LCD_X_MAX-7)))			// don't clear battery status at row=3 and last 5 columns
				break;   
		}
	}
	current_column = 0;
	current_page = 3;
}


/******************************************************************************
*                                                                             *
*  Function: LCD_putchar                                                      *
*                                                                             *
*  Input:    unsigned char c          -- ASCII char to print to LCD           *
*                                                                             *
*  HWInput:  (None)                                                           *
*                                                                             *
*  HWOutput: (SPI) SPDR, SPCR, SPSR, MISO, MOSI, SCK, SS                      *
*                                                                             *
*  Globals Read:  PROGMEM Char_Table  -- corresponds to the font chosen       *
*                                       by the user                           *
*                                                                             *
*  Globals Write: char current_column -- an 8-bit value to keep track of      *
*                                        LCD column being printed to          *
*                 char current_page   -- an 8-bit value to keep track of      *
*                                        LCD character line being printed to  *
*                                                                             *
*  Description:  Transfers a character from a stream input to the LCD         *
*                display character routine.                                   *
*                                                                             *
*  Special characters: \a = 07, bell                                          *
*                      \b = 08, backspace                                     *
*                      \t = 09, clear to end of line                          *
*  defn's in ctype.h   \n = 10, new line: next line, justify left             *
*                      \v = 11, vertical tab:                                 *
*                      \f = 12, form feed: next line                          *
*                      \r = 13, carriage return: same line, justify left      *
*                      \e = 27, escape (in our case erase screen)             *
*                                                                             *
*  formatting:         %d - decimal                                           *
*                      %u - unsigned decimal                                  *
*                      %o - octal                                             *
*                      %x - hex                                               *
*                      %c - character                                         *
*                      %s - strings                                           *
*                                                                             *
******************************************************************************/
static int LCD_putchar(char c, FILE *stream)
{
	LCD_write_cmd( LCD_PAGE_SET + current_page );
	LCD_write_cmd( LCD_COLUMN_SET_H + ( current_column >> 4 ));
	LCD_write_cmd( LCD_COLUMN_SET_L + ( current_column & 0x0f ));
	if (( current_column + char_width ) > LCD_X_MAX )	// condition for char wrap
	{
		current_column = 0;
		current_page--;
		LCD_write_cmd( LCD_PAGE_SET + current_page - (char_height - 1 )); //accounts for if characters take up more than one page
		LCD_write_cmd( LCD_COLUMN_SET_H + ( current_column >> 4 ));
		LCD_write_cmd( LCD_COLUMN_SET_L + ( current_column & 0x0f ));
	}
	if( c == '\n' ) 									// condition for newline characters
	{
		current_column = 0;
		current_page--;
		LCD_write_cmd( LCD_PAGE_SET + current_page - ( char_height - 1 ));  //accounts for if characters take up more than one page
		LCD_write_cmd( LCD_COLUMN_SET_H + ( current_column >> 4 ));
		LCD_write_cmd( LCD_COLUMN_SET_L + ( current_column & 0x0f ));
	}
	else if( c == '\e' )								// condition for escape - in this case clear display
	{
		LCD_clear();									// clear LCD display
	}
	else if( c == '\t' )								// condition for clear to end of line
	{
		for( ; current_column<LCD_X_MAX; current_column++ )
		{
			if ((current_page == 3) && (current_column > (LCD_X_MAX-7))) // don't clear battery status at row=3 and last 5 columns
				break;   
			LCD_write_data( 0x00 );
    	}
		current_column = 0;
		current_page--;
	}
	else if(c == '\b' )									// condition for backspace characters
	{
		current_column-=char_width+1;    
	}
	else if(c == '\a' )									// condition for bell
	{
		Spkr_Freq=500;
		_delay_ms(50);
		Spkr_Freq=0;
	}
	else
	{
		for(unsigned char column=0;column<char_width;column++)
		{
			if ((current_page == 3) && (current_column > (LCD_X_MAX-12))) // don't clear battery status at row=3 and last 5 columns
				break;
			LCD_write_data(pgm_read_byte( &Char_TableA[((c - 0x20) * char_width ) + column + 3] ));
    	}
		LCD_write_data( 0x00 );
		current_column += char_width + 1;
	}
	current_page &= 3;									// keep in range of 0-3

	return 0;
}


/******************************************************************************
*                                                                             *
*  Function: LCD_putcharXY                                                    *
*                                                                             *
*  Input:    unsigned char c          -- ASCII char to print to LCD           *
*            unsigned char Xpos       -- X position in pixels                 *
*            unsigned char Ypos       -- Y position in lines (0=top, 3=bot)   *
*                                                                             *
*  HWInput:  (None)                                                           *
*                                                                             *
*  HWOutput: (SPI) SPDR, SPCR, SPSR, MISO, MOSI, SCK, SS                      *
*                                                                             *
*  Globals Read:  PR fOGMEM Char_Table  -- corresponds to the font chosen       *
*                                       by the user                           *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Transfers a single character to the LCD at a X,Y position.   *
*                                                                             *
******************************************************************************/
void LCD_putcharXY(unsigned char Xpos, unsigned char Ypos, unsigned char c)
{
	LCD_write_cmd( LCD_PAGE_SET + Ypos );
	LCD_write_cmd( LCD_COLUMN_SET_H + ( Xpos >> 4 ));
	LCD_write_cmd( LCD_COLUMN_SET_L + ( Xpos & 0x0f ));

	for(unsigned char column = 0; column < char_width; column++)
	{
		LCD_write_data(pgm_read_byte( &Char_TableA[((c - 0x20) * char_width ) + column + 3] ));
   	}
}


/******************************************************************************
*                                                                             *
*  Function: SPI_Transmit (master mode)                                       *
*                                                                             *
*  Input:    unsigned char addr, unsigned char data -- Data to send to addr   *
*                                                                             *
*  Output:   (None)                                                           *
*                                                                             *
*  HWInput:  (None)                                                           *
*                                                                             *
*  HWOutput: (SPI) SPDR, SPCR, SPSR, MISO, MOSI, SCK, SS                      *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  This function will set the slave address, send a SPI char    *
*                then wait until the character is completely sent.            *
*                                                                             *
******************************************************************************/
void SPI_Transmit(unsigned char addr, unsigned char data)
{
	Set_Slave_Addr( addr );								// set slave select address pins

	SPDR = data; 								   		// Send the data out

	while( !( SPSR & ( 1<<SPIF ) ) );  					// Wait for empty transmit buffer

} // end SPI_Transmit()


/******************************************************************************
*                                                                             *
*  Function: SPI_Receive (master mode)                                        *
*                                                                             *
*  Input:    unsigned char addr	                                              *
*                                                                             *
*  Output:   (None)                                                           *
*                                                                             *
*  HWInput:  (None)                                                           *
*                                                                             *
*  HWOutput: (SPI) SPDR, SPCR, SPSR, MISO, MOSI, SCK, SS                      *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  This function will send a dummy byte (0x00) just to kick     *
*                off the SPI process and to receive a byte from the slave.    *
*                                                                             *
******************************************************************************/
unsigned char SPI_Receive( unsigned char addr, unsigned char dummy_data )
{
	Set_Slave_Addr( addr );								// set slave select address pins

	SPDR = dummy_data;									// send dummy data

	while( !( SPSR & (1 << SPIF ) ) );					// wait for SPI to complete

	return SPDR;										// receive byte is returned in SPI buffer

} // end SPI_Receive()


/******************************************************************************
*                                                                             *
*  Function: Set_Slave_Addr                                                   *
*                                                                             *
*  Input:    unsigned char addr                                               *
*                                                                             *
*  Output:   (None)                                                           *
*                                                                             *
*  HWInput:  (None)                                                           *
*                                                                             *
*  HWOutput: PB2-0                                                            *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  This function sets the slave select line for 74HC138 demux   *
*                                                                             *
*******************************************************************************/
void Set_Slave_Addr(unsigned char addr)
{
	addr = (addr & 0x07);								// mask off invalid slave addresses

	PORTB = (PORTB & 0xf8) + addr;						// set slave select address pins and keep
														//   rest of PORTB the same

	if (addr == 2)										// ATtiny needs a slower clock to respond to commands
	{
		SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0); // Initialize SPI port as master
		SPSR = 0;										// Makes Fosc/128
		_delay_us(100);
	}
	if (addr == 1)										// PSX needs a slower clock, LSB first, inverted polarity & clk
	{
		SPCR = (1<<SPE) | (1<<DORD) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA) | (1<<SPR1) | (1<<SPR0);
		SPSR = 0;										// Makes Fosc/128
		_delay_us(100);
	}
	else
	{
		SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1)  | (1 << SPR0); // Initialize SPI port as master
		SPSR = (1 << SPI2X);							// SPI clk: Fosc/64
		_delay_us( 2 );									// allow to stabilize just a bit
	}
}


/******************************************************************************
*                                                                             *
*  Function: ATmega324_init                                                   *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description: Performs basic initialization sequence on ATmega48.           *
*                                                                             *
*******************************************************************************/
void ATmega324_init( void )
{
   	cli();												// disable global interrupts so we are not interrupted

    DDRA = 0b00000000;									// MSB->LSB: all inputs
    PORTA = 0b00000000;									// disable pull-ups for ADC

    DDRB = 0b10111111;									// MSB->LSB: port PB6 input, all others outputs
														// PB0-2: XX138 slave select address lines
														// PB3: A0 or register select for LCD
														// PB4: 5V=1, 2.7V low power operation=0
														// PB5: SPI MOSI master output pin
														// PB6: SPI MISO master input pin
														// PB7: SPI SCLK master serial clock output pin

    DDRC = 0b11111100;									// MSB->LSB: port PC2-7 output, all others inputs
														// PC7: Motor_2C_pin
														// PC6: Motor_2B_pin
														// PC5: Motor_2A_pin
														// PC4: Motor_1C_pin
														// PC3: Motor_1B_pin
														// PC2: Motor_1A_pin

    DDRD = 0b11110000;									// MSB->LSB: port PD5-7 output, all others inputs
														// PD4: charger PWM output
														// PD5: red LED
														// PD6: green LED
														// PD7: speaker LED
	
	PORTB = 0b00010111;									// Select slave device 07 and SS=1 to be able to enter
														//   SPI master mode

	_delay_ms(250);



	PWMcharger(0xffff);									// initialize PWM port
	uiPWMval = MAX_PWM_VAL;								// set to max in case no battery is present and
	PWMcharger(uiPWMval);								//   operation is req'd from DC supply
	PWMcharger(0x0020);									//   operation is req'd from DC supply

// Initialize SPI port
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1)  | (1 << SPR0); // Initialize SPI port as master
	SPSR = (1 << SPI2X);								// SPI clk: Fosc/64

// Initialize timer 0 interrupt for DDS
	TCCR0A |= (1<<WGM01);								// WGM01=1: count to OCRA instead of 0xff
												
	TCCR0B |= (1<<CS01);								// CS01=1: use clkI/O with 1:8 prescaler
	OCR0A = 100;				   						// count up to 100, interrupt then restart at 0; this
														//   divisor plus the 1:8 prescaler yield: 20MHz/8/100 = 25kHz
														//   interrupt rate; this leave about 20MHz/25000kHz or 800
														//   instructions that can be executed within the timer 0
														//   interrupt before we run out of CPU cycles
	TIMSK0 |= (1<<OCIE0A);								// OCIE0A=1: enable Timer/Counter0 Output Compare Match A Interrupt

// Initialize timer 2 interrupt to provide a timer-based motor acceleration constant
	TCCR2A |= (1<<WGM21);								// WGM21=1: count to OCRA instead of 0xff
											
	TCCR2B |= (1<<CS20) | (1<<CS22);					// CS20, CS22=1: use clkI/O with 1:128 prescaler
	OCR2A = 93;	   										// count up to 93, interrupt then restart at 0; this
														//   divisor plus the 1:128 prescaler yield: 20MHz/128/93/8 phases ~ 210Hz
														//   interrupt rate or approx 4.76ms between each interrupt process.  This is
														//   used to linearly add speed to an acceleration register, 
														//   accelerating the motors to their final speed
	TIMSK2 |= (1<<OCIE2A);								// OCIE2A=1: enable Timer/Counter2 Output Compare Match A Interrupt

// Initialize ADC for slowest sampling rate
	ADMUX = (1<<REFS0); 								// select AVCC as top ADC reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // set ADC clk to 20MHz / 128 = 156.25kHz
	DIDR0 = (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D); // disable digital inputs on AD2..0

	uiBattCtr = 840;									// this correlates to 4 seconds of delay on reset before the battery
														//   state and charge is analyzed.  After this initial point, the
														//   TMR2 interrupt is checked every 1 second.
	ucInt2Phase = 0;									// timer 2 interrupt phase counter 0..7
	ucChgSecCtr = 0;									// Timeout second counter for charge system
	uiChgMinCtr = 0;									// Timeout minute counter for charge system
	lBatt_History = BATT6V0LONG;						// set to approx 6V initially;
														//   2^11 * 2^5 oversampling of battery voltage
	lBatt_PrevHistory = BATT6V0LONG - 3;				// set to approx 6V initially (slightly less than lBatt_History);
														//   comparison value of 2^11 * 2^5 oversampling of battery voltage
	Motor_Accel_Rate = 2;								// Motor acceleration rate
	Motor_Decel_Rate = 8;								// Motor deceleration rate
	uiPSX_TimeoutCtr = 5;								// PSX inactivity timeout counter (start w/non-zero value)
	ucPSX3Change = 0;									// PSX byte 3 status: if any bit is a "1", this bit has changed
	ucPSX4Change = 0;									// PSX byte 4 status: if any bit is a "1", this bit has changed
	ucPSX3History = 0xff;								// PSX byte 3 history: value of PSX byte 3 from previous read
														//   (initially set to 0xff - no buttons pressed)
	ucPSX4History = 0x7f;								// PSX byte 4 history: value of PSX byte 4 from previous read
														//   (initially set to 0xff - no buttons pressed)
	Motor_L_Revs = 0;									// reset motor revolutions
	Motor_R_Revs = 0;									// reset motor revolutions
	ucBrakeMode = 0;									// brakes initially off
	ucBotAnalogSpeed = ANALOG_SPEED;					// initialize default bot speed (analog)
	ucBotDigitalSpeed = DIGITAL_TURN_SPEED;				// initialize default bot speed (digital) 
	ulMotor_L_Pos = MOTOR_ZERO_POSITION;				// position Motor L is to travel to
	ulMotor_R_Pos = MOTOR_ZERO_POSITION;				// position Motor R is to travel to
	ulMotor_L_Pos_Reg = MOTOR_ZERO_POSITION; 			// internal variable - not for general use - current motor position
	ulMotor_R_Pos_Reg = MOTOR_ZERO_POSITION; 			// internal variable - not for general use - current motor position
	uiTinyRev = Read_Tiny_Rev( );						// ATtiny firmware revision level
	Read_PSX( );										// get one read out of the way to init controller & history vars

	stdout = &mystdout;									// redirect stdout to LCD display

	LCD_init();											// LCD initialization
	_delay_ms(30);
	LCD_putcharXY(123,3,BATT_0BAR);	 					// display initial battery status (empty)

	sei();				    							// Enable Global Interrupts

// Ready to rock and roll.    
} // end initialize_ATmega48()


/******************************************************************************
*                                                                             *
*  Function: PWMcharger();		                                              *
*                                                                             *
*  Input: (PWM value)                                                         *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Set charger PWM value                                        *
*                                                                             *
******************************************************************************/
void PWMcharger(unsigned int uiPWMval_local)
{

	if (uiPWMval_local == 0xffff)						// if 0xffff, init PWM port
	{
		TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10); // WGM11=1,WGM10=1: count to OCRA instead of 0xffff
														// WGM11=1,WGM10=1: count to OCRA instead of 0xffff,
														// CS10=1: use clkI/O with 1:1 prescaler
		TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);

		OCR1AH = 0x04;									// count up to 0x271 (625 for 32kHz PWM)
		OCR1AL = 0xb0; 									// count up to 0x271 (625 for 32kHz PWM)

		OCR1BH = 0x01; 									// % duty cycle
		OCR1BL = 0x00;

	}
	else if (uiPWMval_local == 0x0000)					// if 0x0000, disable PWM port
	{
		TCCR1B = 0;										// turn off timer 1 clk source
	}
	else
	{
		OCR1BH = (uiPWMval_local >> 8) & 0xff; 			// peel off high byte
		OCR1BL = uiPWMval_local & 0xff; 				// peel off low byte
	}
}

/******************************************************************************
*                                                                             *
*  Function: LCD_init();		                                              *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Initialize graphic LCD display registers                     *
*                                                                             *
******************************************************************************/
void LCD_init(void)
{ 
   	LCD_write_cmd (0xA2);								// LCD bias set
    LCD_write_cmd( 0xA0);								// ADC select
    _delay_ms(10);
    LCD_write_cmd (0xC0);								// common output mode select -->normal
    LCD_write_cmd( 0x2f);								// power control vol reg ckt ON
//--------Electronic VOLUME set ---> 2byte instruction
    LCD_write_cmd (0x81);								// eletronic MODE set  -- vol control
    LCD_write_cmd (0x16);								// eletronic vol register set
//-----power control setting ------
    LCD_write_cmd (0x22);								// voltage regulator internal Resistor ratio set
    _delay_ms(10);
	LCD_write_cmd(0xA6);								// Sets LCD to normal display (reverse=0xA7)
	LCD_write_cmd(0xA4);								// Display all points normally
    LCD_write_cmd(0xAF);								// Display ON
	LCD_write_cmd(0xB3);								// set the page to start displaying at the top one
	LCD_write_cmd(0x40);								// set the display RAM to start from the beginning
	LCD_write_cmd(LCD_COLUMN_SET_H);					// set the high byte of the LCD column address
	LCD_write_cmd(LCD_COLUMN_SET_L);					// set the low byte of the LCD column address
	LCD_clear();										// clear LCD
}


/******************************************************************************
*                                                                             *
*  Function: LCD_splash();                                                    *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Display copyright and revision levels                        *
*                                                                             *
******************************************************************************/
void LCD_splash( void )
{
	Spkr_chirp();
	LCD_clear();
	printf("CEENBoT, Inc.\n(C) 2010");
	_delay_ms(500);

	Spkr_chirp();
	_delay_ms(1000);
	LCD_clear();
}


/******************************************************************************
*                                                                             *
*  Function: Spkr_chirp();                                                    *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: Chirp, chirp sound from speaker.                                   *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Chirp, chirp sound from speaker.                             *
*                                                                             *
******************************************************************************/
void Spkr_chirp( void )
{
	Spkr_Freq=4000;										// chirp, chirp
	_delay_ms(4);
	Spkr_Freq=0;
	_delay_ms(20);
	Spkr_Freq=6000;
	_delay_ms(4);
	_delay_ms(20);
	Spkr_Freq=4000;
	_delay_ms(4);
	Spkr_Freq=0;
}

/******************************************************************************
*                                                                             *
*  Function: TIMER0_Interrupt                                                 *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Process Direct Digital Synthesis Frequency generator events. *
*                                                                             *
*******************************************************************************/
ISR(TIMER0_COMPA_vect)
{
    cli();												// disable global interrupts so we are not interrupted
														//   in this routine
// ========================= DDS Explained: ================================ //
//  
//  First, lets start with an explanation of a more traditional way to generate Frequencies by using
//	a clock Frequency of 1MHz for example and divide this clock by n.  The output Frequencies are:
//
//  Fout = clk / n
//       = 1MHz / 1 = 1MHz
//       = 1MHz / 2 = 500kHz       	change in Frequency = 500kHz
//       = 1MHz / 3 = 333.333kHz   	change in Frequency = 166.666kHz
//       = 1MHz / 4 = 250kHz       	change in Frequency = 83.333kHz
//       = 1MHz / 5 = 200kHz		change in Frequency = 50kHz
//
//	Note the difference in Frequencies by each successive change in n is non-linear and the changes in
//	Frequency are large.  For each increase in the divisor n, the output Frequency decreases by a smaller
//  amount and not constant.  When n is a larger divisor each change in n produces very little change in
//  Frequency and certainly much different than the changes when n is small.
//
//  Fout = clk / n
//       = 1MHz / 65531 = 15.2599533045Hz
//       = 1MHz / 65532 = 15.2597204419Hz  	change in Frequency = 0.0002328626Hz
//       = 1MHz / 65533 = 15.2594875864Hz  	change in Frequency = 0.0002328555Hz
//       = 1MHz / 65534 = 15.2592547379Hz   change in Frequency = 0.0002328485Hz
//       = 1MHz / 65535 = 15.2590218966Hz	change in Frequency = 0.0002328413Hz
//
//	Changes in n when n~65530 produce changes in Frequency of 230microHertz.  These changes in Frequency
//	are not very useful when generating tones for a speaker, for example.  It would be much more useful
//	to change the Frequency applied to a speaker by 0.1Hz or possible 1Hz at a time.  To produce audio in
//  fairly even steps we can generate a lookup table of divisors for the output Frequencies we are looking
// 	for.  This kind of table would be large.  Alternatively we can use C math functions to determine
//	the value of n for a divisor to generate a Frequency, f.  This also takes quite a bit of code and 
//  the computation is required every time we want to change Frequencies.
//
//  This is where Direct Digital Synthesis (DDS) can be used to simplify Frequency generation and provide
//  a linear relationship between an output Frequency and the variable controlling it.  To provide this
//  linear and proportional relationship we need to move the Frequency determining variable into the
//  numerator position.
//
//  Fout = clk * (n / divisor constant)
//
//	Lets start with some assumptions for the divisor constant and list for values of n.  The divisor
//	constant is typically the size of a register: 8-bits, 16-bits, 24-bits or 32-bits.  We will use
//	16-bits for this example.
//
//  Fout = 1MHz * (n / 2^16)
//       = 1MHz * (0 / 65536) = 0Hz
//       = 1MHz * (1 / 65536) = 15.2587890625Hz   change in Frequency = 15.2587890625Hz
//       = 1MHz * (2 / 65536) = 30.5175781250Hz	  change in Frequency = 15.2587890625Hz
//       = 1MHz * (3 / 65536) = 45.7763671875Hz   change in Frequency = 15.2587890625Hz
//       = 1MHz * (4 / 65536) = 61.0351562500Hz   change in Frequency = 15.2587890625Hz
//       ...
//       = 1MHz * (65532 / 65536) = 99.9938964843kHz
//       = 1MHz * (65533 / 65536) = 99.9954223632kHz   change in Frequency = 15.2587890625Hz
//       = 1MHz * (65534 / 65536) = 999969.482421kHz   change in Frequency = 15.2587890625Hz
//       = 1MHz * (65535 / 65536) = 999984.741210kHz   change in Frequency = 15.2587890625Hz
//       = 1MHz * (65536 / 65536) = 1MHz               change in Frequency = 15.2587890625Hz
//
//	Note for every change in n the change in Frequency is constant (about 15Hz).  Now we can
//  generate a linear Frequency sweep from 0Hz to 1500Hz by simply changing n from 0 to 100.
//
//  Now for the software implementation...
//  DDS techniques use an accumulator which is just a variable or register that is incremented by a
//  value, n.  If n is small or zero the accumulator takes a long time to overflow.  If n is large
//  and approaches the size of the accumulator, it does not take long for the accumulator to
//  overflow.
//
//  The overflow event is what triggers the output Frequency.  When the DDS accumulator rolls over
//  it can generate a carry flag (accessible in assy language) or when the accumulator exceeds some
//  arbitrary number we can also generate an artificial overflow event and trigger a change in
//  the output.  In C we don't have an easy way to access the overflow flag if a register overflows
//  but that is not really a problem.  We can use this practical example to generate 1Hz increments
//  in Frequency output.
//
//	clk = 8MHz / 256 (timer0 set to generate an interrupt every time it generates an overflow) 
//  This produces an interrupt Frequency of 31,250Hz.  If we choose the divisor constant to be the
//  same number, 31,250 then the following happens:
//
//  Fout = clk * (n / divisor constant) 
//  Fout = 31,250Hz * (n / 31,250Hz)
//  or
//  Fout = n Hz
//
//  This is truw when changing the phases of a stepper motor but the following will produce a
//  Frequency only half of what we want if driving a speaker output:
//
//  DDS overflow event...
//  Toggle speaker output
//  DDS overflow event...
//  Toggle speaker output
//  DDS overflow event...
//  Toggle speaker output
//
//  For every toggle of the speaker output, we are only producing 1/2 of the wave to make a complete
//  cycle so a setting of n=1 will produce an output=1 for 1 second then an output=0 for 1 second
//  which equates to 0.5Hz.  If ew want n to be representative of the true Frequency output then we
//  need a higher overflow rate of the DDS accumulator by exactly 2 times.  So lets choose the 
//  divisor constant to be 31,250 / 2 or 15,625.  Now the equation is:
//
//  Fout = clk * (n / divisor constant) 
//  Fout = 31,250Hz * (n / 15,625Hz)
//  or
//  Fout = 2 * n Hz
//
//  When toggling a speaker out with a n=1, now the output is high for 1/2 second and the output
//  is low for 1/2 second and the resulting output Frequency is 1Hz for n=1.
//
//  Now for the software implementation:
//  Set up the system clock for 8MHz operation.
//  Use timer 0 to generate interrupts when timer 0 overflows at max value of 0xff.
//  
//  Timer 0 interrupt code:
//  Accumulator += n;
//  if (Accumulator >= divisor_constant)
//	{
//		Accumulator -= divisor_constant;				// subtract off divisor constant but keep
//														//   remainder because it is needed for fractional
//														//   Frequenct synthesis.  Do not set Accumulator to
//														//   zero on an overflow event or the Frequency output
//														//   will have some non-linear stair stepping.
//      PORTD ^= Speaker_Output;						// generate some I/O event (move motor, etc).
//	}
//
//	That is it.  The DDS accumulator is incremented by n for each pass throuth the interrupt
//  and this is the part that provides a proportional output Frequency to n.  The larger n is, the
//  faster the DDS accumulator approaches an overflow point.  Next the accumulator is compared to
//  the divisor constant.  If the accumulator exceeds the divisor constant then the divisor constant
//  is subtracted from the accumulator and this prevents a real overflow event within C.  We don't
//  want to use overflow interrupts to handle a variable exceeding it's 16-bit value.
//
//  Let's look at the interrupt in action.  If n=0 the accumulator does not increment and is not
//  given the opportunity to overflow.  No event happens.  If n=1 it takes 15,625 times through the
//  interrupt for our software to capture the overflow event.  This occurs in 1/2 second and the speaker
//  output is toggled.  Since we were counting by n=1 we reached 15,625 exactly and exactly 15,625
//  was subtracted from the accumulator leaving the accumulator = 0.  And the process repeats.
//
//  Now let's choose n = 100.  The accumulator will reach 15,600 on the 156th interrupt and still be
//  less than the divisor constant.  On the 157th interrupt the accumulator will equal 15,700
//  exceeding the 15,625 divisor constant.  After subtracting 15,625 from 15,700 we end up with
//  the accumulator = 75.  This remainder is important because it get's us to the overflow state
//  a little faster than the last time.  Now it only takes 156 interrupts for the accumulator to
//  equal 15,675 which exceeds the divisor constant.  Now subtracting 15,625 from the accumulator
//  leaves the accumulator = 50.  Again this remainder helps to get to an overflow state faster
//  that if the accumulator started at 0 and requiring 157 interrupts to overflow.  The sequence
//  continues:
//
//  Overflow event #1--> 157 interrupts--> accumulator = 15700 before subtraction, 75 after
//  Overflow event #2--> 156 interrupts--> accumulator = 15675 before subtraction, 50 after
//  Overflow event #3--> 156 interrupts--> accumulator = 15650 before subtraction, 25 after
//  Overflow event #4--> 156 interrupts--> accumulator = 15625 before subtraction, 0 after
//
//  Now the cycle repeats: 157 interrupts, 156, 156, 156, 157, 156, 156, 156...
//  Since n = 100 and the divisor constant 15,625 is not evenly divisible by 100, we rely on
//  the accumulator remainder (75, 50, 25, 0) to provide a dithering effect to generate the desired
//  output Frequency.  This will be noticeable with an oscilloscope but not detectable when flashing
//  an LED, driving a stepper motor, etc.
//
//  Since the DDS method takes little code, multiple software DDS timers can be implemented
//  side by side to create timing for different purposes.  I.e.
//
//  Flash an LED from 0.1Hz to 20Hz.
//  Increment a stepper motor from 0.1 steps/second to 1000 steps/second.
//  Drive a speaker from 0.1Hz to 15kHz.
//

// now process Motor Left
//
	if ( Motor_L_Accel_Reg )
		DDS_Motor_L_Accum += Motor_L_Accel_Reg;			// increment accumulator by n
	else if (ucBrakeMode)
	{
		PORTC &= 0b11100011;							// if brakes = on, turn motor coil on
		PORTC |= 0b00000100;							// if brakes = on, turn motor coil on
	}
	else
		PORTC &= 0b11100011;							// if speed = 0, turn off this motor
 
	if ( DDS_Motor_L_Accum > DDS_DIVISOR )			 	// check to see if we exceed the divisor constant
	{
		DDS_Motor_L_Accum -= DDS_DIVISOR;				// if yes, subtract off divisor constant
		Motor_L_Revs++;									// increase motor revolution counter
		if ( Motor_L_Revs > 102300)						// don't let number get too big
			Motor_L_Revs = 0;

		if ( ucMotor_L_Dir )
			ucMotor_L_State++;							// DDS rolled over, if forward, increment stepper phase		
		else
			ucMotor_L_State--;							// DDS rolled over, reverse, decrement stepper phase		
		ucMotor_L_State &= 0x03;						// mask lower 2 bits to end up with 0x00-0x03
		DDS_scratch_reg = PORTC & 0b11100011;			// keep other PORTC output data but clear Motor L bits
		PORTC = DDS_scratch_reg | Motor_L_LUT[ucMotor_L_State];
		if ( ucMotor_L_Speed < 50 )
			DDS_L_Pulse_Timer = Motor_PWM_LUT[ucMotor_L_Speed>>2];// this equates to 5ms (25.6uS * 195 = 5ms); when this
														//   timer expires, turn off Motor L phase bits so we don't
														//   draw too much current
		else
			DDS_L_Pulse_Timer = 125;					// this equates to 5ms (25.6uS * 195 = 5ms); when this
														//   timer expires, turn off Motor L phase bits so we don't
														//   draw too much current
	}



// now process Motor Right
//
	if ( Motor_R_Accel_Reg )
	{
		DDS_Motor_R_Accum += Motor_R_Accel_Reg;			// increment accumulator by n
	}
	else if (ucBrakeMode)
	{
		PORTC &= 0b00011111;							// if brakes = on, turn motor coil on
		PORTC |= 0b00100000;							// if brakes = on, turn motor coil on
	}
	else
		PORTC &= 0b00011111;							// if speed = 0, turn off this motor
  	if ( DDS_Motor_R_Accum > DDS_DIVISOR ) 				// check to see if we exceed the divisor constant
    {
        DDS_Motor_R_Accum -= DDS_DIVISOR;				// if yes, subtract off divisor constant
		Motor_R_Revs++;									// increase motor revolution counter
		if ( Motor_R_Revs > 102300)						// don't let number get too big
			Motor_R_Revs = 0;

		if ( ucMotor_R_Dir )
			ucMotor_R_State++;							// DDS rolled over, if forward, increment stepper phase		
		else
			ucMotor_R_State--;							// DDS rolled over, reverse, decrement stepper phase		
		ucMotor_R_State &= 0x03;						// mask lower 2 bits to end up with 0x00-0x03
		DDS_scratch_reg = PORTC & 0b00011111;			// keep other PORTC output data but clear Motor R bits
		PORTC = DDS_scratch_reg | Motor_R_LUT[ucMotor_R_State];
		if ( ucMotor_R_Speed < 50 )
			DDS_R_Pulse_Timer = Motor_PWM_LUT[ucMotor_R_Speed>>2];
		else
			DDS_R_Pulse_Timer = 125;					// this equates to 5ms (25.6uS * 195 = 5ms); when this
														//   timer expires, turn off Motor R phase bits so we don't
														//   draw too much current
														// 
	}


// now process Brakes (DDS frequency controlled)
//
	if (ucBrakeMode)									// if brakes = on and
	{
		DDS_Brake_Accum += DDS_BRAKE_FREQ;				// increment accumulator by n
 
		if ( DDS_Brake_Accum > DDS_DIVISOR )			// check to see if we exceed the divisor constant
		{
    	    DDS_Brake_Accum -= DDS_DIVISOR;				// if yes, subtract off divisor constant

			if (Motor_L_Accel_Reg == 0)					//   Motor L speed = 0 (stopped), enable braking
			{
				PORTC &= 0b11100011;				
				PORTC |= 0b00000100;					// turn on one coil
				DDS_L_Pulse_Timer = 35;					// this equates to Xms (25.6uS * 195 = 5ms); when this
														//   timer expires, turn off Motor R phase bits so we don't
			}											//   draw too much current
			if (Motor_R_Accel_Reg == 0)					//   Motor R speed = 0 (stopped), enable braking
			{
				PORTC &= 0b00011111;				
				PORTC |= 0b00100000;					// turn on one coil
				DDS_R_Pulse_Timer = 35;					// this equates to 5ms (25.6uS * 195 = 5ms); when this
														//   timer expires, turn off Motor R phase bits so we don't
			}											//   draw too much current
		}
	}


// now process Motor Pulse Length Timeout Counters
//
	if ( DDS_L_Pulse_Timer )							// if pulse timer is positive then decrement
		DDS_L_Pulse_Timer--;
	if ( DDS_L_Pulse_Timer == 0x00 )
		PORTC &= 0b11100011;							// timer expired, turn off Motor L phase bits so we don't

	if ( DDS_R_Pulse_Timer )							// if pulse timer is positive then decrement
		DDS_R_Pulse_Timer--;
	if ( DDS_R_Pulse_Timer == 0x00 )
		PORTC &= 0b00011111;							// timer expired, turn off Motor R phase bits so we don't
														//   draw too much current


	if ( uiSpkr_Timer )									// if speaker timer was set, decrement timer
	{
		uiSpkr_Timer--;
		if ( uiSpkr_Timer == 0 )						// if timer now = 0, turn off speaker
			Spkr_Freq = 0;
	}

	DDS_spkr_Accum += Spkr_Freq;						// increment accumulator by n
  	if ( DDS_spkr_Accum > DDS_DIVISOR ) 				// check to see if we exceed the divisor constant
    {
        DDS_spkr_Accum -= DDS_DIVISOR;					// if yes, subtract off divisor constant

		Speaker_Toggle;
	}

	sei();				    							// Enable Global Interrupts
}


/******************************************************************************
*                                                                             *
*  Function: TIMER2_Interrupt                                                 *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  Process motor acceleration if required and capture ADC       *
*                inputs.  Interrupts at 210Hz rate (every 4.76ms).            *
*                                                                             *
*******************************************************************************/

ISR(TIMER2_COMPA_vect)
{
    cli();												// disable global interrupts so we are not interrupted

	ucInt2Phase++;										// timer 2 interrupt phase counter 0..7
	if (ucInt2Phase > 7)								// keep in 0..7 range					
		ucInt2Phase = 0;

	switch ( ucInt2Phase )								// perform individual time-slice processes
	{
		case 0:											// process motor acceleration
			if (Motor_L_Accel_Reg > ucMotor_L_Speed)
			{
				if (Motor_L_Accel_Reg > (ucMotor_L_Speed + Motor_Decel_Rate))
					Motor_L_Accel_Reg -= Motor_Decel_Rate;
				else
					Motor_L_Accel_Reg = ucMotor_L_Speed;
			}

			if (ucMotor_L_Speed > Motor_L_Accel_Reg)
			{
				Motor_L_Accel_Reg += Motor_Accel_Rate;
				if (Motor_L_Accel_Reg > ucMotor_L_Speed)
					Motor_L_Accel_Reg = ucMotor_L_Speed;
			}

			if (Motor_R_Accel_Reg > ucMotor_R_Speed)
			{
				if (Motor_R_Accel_Reg > (ucMotor_R_Speed + Motor_Decel_Rate))
					Motor_R_Accel_Reg -= Motor_Decel_Rate;
				else
					Motor_R_Accel_Reg = ucMotor_R_Speed;
			}
	
			if (ucMotor_R_Speed > Motor_R_Accel_Reg)
			{
				Motor_R_Accel_Reg += Motor_Accel_Rate;
				if (Motor_R_Accel_Reg > ucMotor_R_Speed)
					Motor_R_Accel_Reg = ucMotor_R_Speed;
			}
			break;

		case 1:											// process ADC reading/averaging
			// process ADC reads by averaging 31/32 from past samples and 1/32 from current ADC sample
			// ADC0 = battery current
			// ADC1 = battery voltage
			// ADC2 = charger voltage
			//
			uiADC_avg_reg = ( uiADC_History[ucADC_Read_State] >> 5 ); // compute old average
			uiADC_History[ ucADC_Read_State ] -= uiADC_avg_reg;
			uiADC_avg_reg = ADCL;						// copy lower 8-bits first to freeze access to ADCL & ADCH
			uiADC_avg_reg |= (ADCH << 8); 				//   then copy ADCH to re-enable updating of ADC
			uiADC_History[ ucADC_Read_State ] += uiADC_avg_reg;
			uiADC_Instant[ ucADC_Read_State ] = uiADC_avg_reg;

			
			if (ucADC_Read_State == 1)					// process 2^5 * 2^11 oversampling for 15-minute Vbatt sampling
			{
				ulADC_avg_reg = ( lBatt_History >> 11 ); // compute old average
				lBatt_History -= ulADC_avg_reg;
				lBatt_History += uiADC_History[ 1 ];
			}

			ucADC_Read_State++;							// select next ADC input channel
			if (ucADC_Read_State > 3)
				ucADC_Read_State = 0;					// cycle between ADC channels 0-2
		
			ucADC_Read_State &= 0b00000011;				// go to next state 
			ADMUX = (1<<REFS0) | ucADC_Read_State;		// select AVCC as top ADC reference and mux to 0x00-0x02
			ADCSRA |= (1 << ADSC); 						// start conversion with current channel

			uiBattCtr--;								// update 1 sec timer



			if (uiBattCtr == 0) 						// check battery state every 210/210 times/sec or 1 sec
				uiBattCtr = 209;
			break;			

		case 2:
			if (uiBattCtr == 200)						// process RESET state every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if (ucBattChgState == RESET)
				{
				PWMcharger(MIN_PWM_VAL);				// turn PWM to lowest usable value

					if (uiADC_History[2] > CHG6V0)		// if chg > 6.0V, go to CHARGING state
					{
						ucBattChgState = CHARGING;
						ucBattDispState = BATT_CHG;		// up arrow (charging)

						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
						uiPWMval = START_PWM_VAL;
					}
					else
					{
						dMaH_ctr += ((int)((498 << 5) - uiADC_History[0]) * 0.0002306 / 3.6);
					}
				}
				ucChgSecCtr++;							// increment seconds for charge timout counter
				if (ucChgSecCtr > 59)
				{
					ucChgSecCtr = 0;
					uiChgMinCtr++;
				}
			}
			break;

		case 3:
			if (uiBattCtr == 200)						// process RESET or CHARGED states every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if ((ucBattChgState == RESET) || (ucBattChgState == CHARGED))
				{
					if (uiADC_History[1] < BATT6V0)		// correlates to 6.0V: 0.000471V/ADC LSB
					{
						ucBattDispState = BATT_0BAR; 	// battery with 0 bars
						ucBattChgState = BATT_WARNING;
						ucChgSecCtr = 52;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
					}
					else if (uiADC_History[1] < BATT6V4) // correlates to 6.4V: 6-6.39V
					{
						ucBattDispState = BATT_1BAR;	// battery with 1 bar
					}
					else if (uiADC_History[1] < BATT6V8) // correlates to 6.8V: 6.4-6.79V
					{
						ucBattDispState = BATT_2BAR;	// battery with 2 bars
					}
					else if (uiADC_History[1] < BATT7V2) // correlates to 7.2V: 6.8-7.19V
						ucBattDispState = BATT_3BAR;	// battery with 3 bars
					else if (uiADC_History[1] < BATT7V6) // correlates to 7.6V: 7.2-7.59V
						ucBattDispState = BATT_4BAR; 	// battery with 4 bars
					else if (uiADC_History[1] < BATT8V0) // correlates to 8.0V: 7.6-7.99V
						ucBattDispState = BATT_5BAR; 	// battery with 5 bars
					else								// 8V+
						ucBattDispState = BATT_6BAR;	// battery with 6 bars
				}
			}
			break;

		case 4:
			if (uiBattCtr == 200)						// process CHARGING state every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if (ucBattChgState == CHARGING) 
				{

					if ( uiADC_History[ 0 ] > FAST_CHG_CURRENT ) // if using a high current charger, check Vbatt more often
						ucChgChkTime = 4;				// check every 4 minutes
					else
						ucChgChkTime = 15;				// check every 15 minutes

					if ((( uiChgMinCtr % ucChgChkTime) == 0 ) && ( ucChgSecCtr == 0 )) // compare old & new battery voltages every 5 min
					{
						lBatt_Difference = lBatt_History - lBatt_PrevHistory;
						if ( lBatt_History > lBatt_PrevHistory ) // see if Vbatt increased by any amount.  If so, continue charging
						{
							lBatt_PrevHistory = lBatt_History; // set 4 or 15 minute battery voltage reference
							Spkr_Freq = 1000;
							uiSpkr_Timer = 2000;		// limit speaker tone to 0.1 second
						}
						else
						{
							lBatt_PrevHistory = BATT6V0LONG; // set to low value for 2nd charge situations
							uiPWMval = MIN_PWM_VAL;		// stop charging, Vbatt didn't increase within a X minute period
							ucBattChgState = CHARGED;
							Spkr_Freq = 3000;
							uiSpkr_Timer = 60000;		// limit speaker tone to 3 seconds
						}
					}
					
					if ( uiADC_History[ 0 ] > MAX_CHG_CURRENT )	// if we exceed chg current then
						uiPWMval--;						// decrease PWM to decrease chg current
					else
					{
						if ( uiADC_History[ 0 ] < RISE_CHG_CURRENT ) // if some distance from 500mA, increase faster
							uiPWMval = uiPWMval + 10;
						else
							uiPWMval++;					// increase PWM to increase chg current
						if (uiPWMval > MAX_PWM_VAL) 	// don't exceed 90% pwm for 500mA charger
							uiPWMval = MAX_PWM_VAL;
					}
					PWMcharger(uiPWMval);				// set PWM value

					if (uiADC_History[ 2 ] < CHG6V0) 	// if charger disconnected, go to RESET state
					{
						ucBattChgState = RESET;
						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
					if ( uiADC_History[1] > BATT10V8) 	// battery disconnected, go to AC powered state
					{
						ucBattChgState = AC_POWERED; 	// battery voltage exceeds possible value, provide AC PWM power
						ucBattDispState = BATT_AC; 		// AC powered
					}

					// if charge done, go to CHARGED STATE
					if ((uiChgMinCtr > MAX_CHG_TIME) || (dMaH_ctr > MAX_BATT_CHG))
					{
						uiPWMval = MIN_PWM_VAL;
						ucBattChgState = CHARGED;
					}
					dMaH_ctr += ((int)( uiADC_History[0] - (498 << 5)) * 0.0002306 / 3.6);
				}
			}
			break;

		case 5:
			if (uiBattCtr == 200)						// process CHARGED state every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if (ucBattChgState == CHARGED)
				{
					if ( uiADC_History[ 0 ] > MIN_CHG_CURRENT ) // if we exceed chg idle current then
						uiPWMval--;						// decrease PWM to decrease idle current
					else
					{
						uiPWMval++;						// increase PWM to increase idle current
						if (uiPWMval > MAX_PWM_VAL)
							uiPWMval = MAX_PWM_VAL;
					}
					PWMcharger(uiPWMval);				// set idle PWM value

					if (uiADC_History[2] < CHG6V0)		// if charger disconnected, go to RESET state
					{
						ucBattChgState = RESET;
						ucChgSecCtr = 0;				// reset charge counter
						uiChgMinCtr = 0;				// reset charge counter
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
					if (uiADC_History[1] < BATT7V6)		// if battery too low, go to RESET state, then charge
					{
						ucBattChgState = RESET;
						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
					else if ( uiADC_History[1] > BATT11V0) // battery disconnected, go to AC powered state
					{
						ucBattChgState = AC_POWERED; 	// battery voltage exceeds possible value, provide AC PWM power
						ucBattDispState = BATT_AC; 		// AC powered
					}
				}
			}
			break;

		case 6:
			if (uiBattCtr == 200)						// process AC_POWERED and BATT_WARNING states every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if (ucBattChgState == AC_POWERED)
				{
					if ( uiADC_History[1] < BATT10V8) 	// battery connected, go to RESET state
					{
						ucBattChgState = RESET; 		// battery voltage exceeds possible value, provide AC PWM power
						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
				}

				// =============================================================================================
				if (ucBattChgState == BATT_WARNING)
				{
					ucBattDispState = BATT_NEEDCHG; 	// display battery needs charging (!!)

					if (uiChgMinCtr == BATT_WARN_TIME) 	// display battery warning
					{
						ucBattChgState = BATT_SHUTDOWN;
					}
					if (uiADC_History[2] > CHG6V0) 		// if charger is connected, go to RESET state to charge
					{
						ucBattChgState = RESET;
						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
					LCD_BL(65 - ucChgSecCtr);			// set LCD backlight brightness
				}
			}
			break;

		case 7:
			if (uiBattCtr == 200)						// process BATT_SHUTDOWN state every 210/210 times/sec or 1 sec
			{
				// =============================================================================================
				if (ucBattChgState == BATT_SHUTDOWN)
				{
					if (uiADC_History[2] > CHG6V0)		// if charger is connected, go to RESET state to charge
					{
						ucBattChgState = RESET;
						ucChgSecCtr = 0;				// Timeout second counter for charge system
						uiChgMinCtr = 0;				// Timeout minute counter for charge system
						dMaH_ctr = 0.1;					// start with something greater than zero
					}
					LCD_clear();						// clear LCD display
					printf("BATTERY LOW!!!!!!\t");
					printf("PLEASE TURN OFF");
					clk_rate(2);						// set /8 clock rate
					_delay_ms(250);						// equivalent to 2 sec at /8 clk rate
					clk_rate(0);						// go to sleep
					funcptr(); 							// Jump to Reset vector 0x0000 in Application Section. 
				}

				// =============================================================================================
				if ((PORTB & 0x07) == 0x07)				// check to see if we are at slave address 7
				{
					LCD_putcharXY(123,3,ucBattDispState); // display battery status
				}
			}
			break;
		default:
		break;
	}

	sei();							    				// Enable Global Interrupts
}


/******************************************************************************
*                                                                             *
*  Function: Set CPU clock divider based on 20MHz XTAL                        *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description: CPU clock prescaler table:                                    *
*                                                                             *
*                                                                             *
*     CLKPS3 CLKPS2 CLKPS1 CLKPS0 Clock Division Factor                       *
*       0      0      0      0              1                                 *
*       0      0      0      1              2                                 *
*       0      0      1      0              4                                 *
*       0      0      1      1              8                                 *
*       0      1      0      0             16                                 *
*       0      1      0      1             32                                 *
*       0      1      1      0             64                                 *
*       0      1      1      1            128                                 *
*       1      0      0      0            256                                 *
*                                                                             *
******************************************************************************/
void clk_rate ( char speed )
{
	int delay1;

	if (speed == 20)									// go to high speed mode
	{
		ATmega324_init();								// Initialize ATmega324 I/O and peripherals

		PORTB |= (1<<PB4);								// set 5V operation
		cli();											// disable global interrupts
		CLKPR = (1<<CLKPCE);							// set up clock divider change (under cli())
		CLKPR = 0x00;									// divide by 1
		sei();											// enable global interrupts
		for (delay1 = 0; delay1 < 40; delay1++)
		{
			__asm__ __volatile__("nop");				// allow time for other 3.3V/5V devices to power up
		}
// disable all pin change interrupts
		PCICR = 0;										// disable interrupts for pin group PCINT31..0
		PCMSK0 = 0;										// disable the PCINT7..0 group
	}

	if (speed == 2)							// go to 2MHz (20MHz/8) mode
	{
		cli();								// disable global interrupts
		CLKPR = (1<<CLKPCE);				// set up clock divider change (under cli())
		CLKPR = (1<<CLKPS1) | (1<<CLKPS0);				// divide by 8
		sei();											// enable global interrupts
	}

	if (speed == 1)										// go to low speed mode, 2.7V operation
	{
		cli();											// disable global interrupts
		CLKPR = (1<<CLKPCE);							// set up clock divider change (under cli())
		CLKPR = (1<<CLKPS3);							// divide by 256
		sei();											// enable global interrupts
	}

	if (speed == 0)							// go to sleep
	{
		for (delay1 = 0; delay1 < 28000; delay1++)
		{
			__asm__ __volatile__("nop");	
		}
		PORTB &= ~(1<<PB4);					// set 2.7V operation
		for (delay1 = 0; delay1 < 20000; delay1++)
		{
			__asm__ __volatile__("nop");	
		}								
		ADCSRA = 0; 						// disable ADC & set ADC0..2 to digital inputs
		PRR = 0xFF;							//power reduction all peripherals 
// enable pin change interrupt for PA2; used to detect when charger is plugged in
		PCICR = (1 << PCIE0);				// enable interrupt for pin group PCINT7..0: this includes PA2 (PCINT2)
		PCMSK0 = (1 << PCINT2);				// within the PCINT7..0 group, only enable PCINT2 for pin PA2

		sei();								// enable global interrupts
       	SMCR = (1 << SE);
		__asm__( "sleep" );
	}
}


/******************************************************************************
*                                                                             *
*  Function: Send RC servo data to ATtiny microcontroller.                    *
*                                                                             *
*  Input: (Five RC servo positions, RC servo 0-5 with values 0-255.)          *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description: 0=full clockwise, 255=full counter-clockwise                  *
*                                                                             *
******************************************************************************/
void RC_Servo(unsigned char RCservo0, unsigned char RCservo1, unsigned char RCservo2,
              unsigned char RCservo3, unsigned char RCservo4)
{
	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
	SPI_Transmit( 2, 0x0a );			// send attention byte to ATtiny (addr 2)
	SPI_Transmit( 2, 0xb0 );			// send servo command to ATtiny (addr 2)
	SPI_Transmit( 2, RCservo0 );		// send servo data PWM0 to ATtiny (addr 2)
	SPI_Transmit( 2, RCservo1 );		// send servo data PWM1 to ATtiny (addr 2)
	SPI_Transmit( 2, RCservo2 );		// send servo data PWM2 to ATtiny (addr 2)
	SPI_Transmit( 2, RCservo3 );		// send servo data PWM3 to ATtiny (addr 2)
	SPI_Transmit( 2, RCservo4 );		// send servo data PWM4 to ATtiny (addr 2)
	_delay_us(35);						// need some delay before deactivating slave select
	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
}


/******************************************************************************
*                                                                             *
*  Function: Send LCD backlight value to ATtiny microcontroller.              *
*                                                                             *
*  Input: (LCD PWM brightness 0-31, 0=off, 31=full on, 32=flash for charge    *
*          indication)                                                        *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:                                                               *
*                                                                             *
******************************************************************************/
void LCD_BL(unsigned char ucLCD_PWM)		// set LCD backlight intensity
{
	Set_Slave_Addr( 7 );					// set to something other than addr 2 to segregate a unique msg
	SPI_Transmit( 2, 0x0a );				// send attention byte to ATtiny (addr 2)
	SPI_Transmit( 2, 0xb1 );				// send LCD backlight brightness cmd to ATtiny (addr 2)
	SPI_Transmit( 2, ucLCD_PWM );			// send LCD backlight brightness cmd to ATtiny (addr 2)
	_delay_us(35);							// need some delay before deactivating slave select
	Set_Slave_Addr( 7 );					// set to something other than addr 2 to segregate a unique msg
}


/******************************************************************************
*                                                                             *
*  Function: Read Sensor data                                                 *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (Sensor data byte)                                                 *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description: Read sensor data from ATtiny                                  *
*                                                                             *
******************************************************************************/
unsigned char Read_Sensors( void )			
{
	unsigned char rx_read, SPI_XOR_val;

	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
	SPI_Transmit( 2, 0x0a );			// send attention byte to ATtiny (addr 2)
	SPI_Transmit( 2, 0xb2 );			// send switch/IR read command to ATtiny (addr 2)
	_delay_us(35);						// need some delay before requesting ATtiny data
	rx_read = SPI_Receive( 2, 0 ); 		// receive switch/IR data from ATtiny (addr 2, dummy data 0)
	_delay_us(35);						// need some delay before requesting ATtiny data

	SPI_XOR_val = SPI_Receive( 2, 0 );	// read XOR data value from ATtiny (addr 2, dummy data 0)
	ucTest = SPI_XOR_val;
	if (( rx_read ^ SPI_XOR_val ) != 0xff ) // if XOR data does not match, use previous good value
		rx_read = 0x00;
	_delay_us(35);						// need some delay before deactivating slave select
	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
	return rx_read;
}


/******************************************************************************
*                                                                             *
*  Function: Read ATtiny firmware revision level                              *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (Revision Level)                                                   *
*                                                                             *
*  Globals Read: (None)	                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description: Read sensor data from ATtiny                                  *
*                                                                             *
******************************************************************************/
unsigned int Read_Tiny_Rev( void )			
{
	unsigned int rx_read;

	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
	SPI_Transmit( 2, 0x0a );			// send attention byte to ATtiny (addr 2)
	SPI_Transmit( 2, 0xb3 );			// send firmware revision command to ATtiny (addr 2)
	_delay_us(55);						// need some delay before requesting ATtiny data
	rx_read = ( SPI_Receive( 2, 0 ) << 8); // receive first byte from ATtiny (addr 2, dummy data 0)
	_delay_us(55);						// need some delay before requesting ATtiny data
	rx_read |= SPI_Receive( 2, 0 );		// receive second byte from ATtiny (addr 2, dummy data 0)
	_delay_us(155);						// need some delay before deactivating slave select
	Set_Slave_Addr( 7 );				// set to something other than addr 2 to segregate a unique msg
	return rx_read;
}


/******************************************************************************
*                                                                             *
*  Function: Set analog mode for PSX controller                               *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: (None)                                                       *
*                                                                             *
*  Globals Write: response[]:                                                 *
*                                                                             *
*  Description: Force PSX into anaog mode.                                    *
*                                                                             *
******************************************************************************/
void Set_PSX_Analog( void )
{
	Set_Slave_Addr( 7 );
	_delay_us( 2 );
	for( uint8_t i = 0 ; i < 5 ; ++i )		// Put the playstation controller into Configuration Mode
	{
		_delay_us( 1 );
		response[ i ] = SPI_Receive( 1, poll3[ i ] ); // address, data (see header for detials)
	} // end for()

	Set_Slave_Addr( 7 );
	_delay_us(2);

	for( uint8_t i = 0 ; i < 9 ; ++i )		// Force Analog Mode on
	{
		_delay_us( 1 );
		response[ i ] = SPI_Receive( 1, poll2[ i ] ); // address, data (see header for details)
	} // end for()

	Set_Slave_Addr( 7 );
	_delay_us(2);

/*	for( uint8_t i = 0 ; i < 9 ; ++i )	// Force Vibration Motors on
	{
		_delay_us( 1 );
		response[ i ] = SPI_Receive( 1, poll5[ i ] ); // address, data (see header for details)
	} // end for()

	Set_Slave_Addr( 7 );
	_delay_us(2);
*/
	for( uint8_t i = 0 ; i < 9 ; ++i )	// Exit Configuration Mode
	{
		_delay_us( 1 );
		response[ i ] = SPI_Receive( 1, poll4[ i ] ); // address, data (see header for details)
	} // end for()

	Set_Slave_Addr( 7 );
	_delay_us(2);

}


/******************************************************************************
*                                                                             *
*  Function: Read PSX data                                                    *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: response[] from PSX controller                               *
*                                                                             *
*  Globals Write: response[]                                                  *
*                                                                             *
*  Description: Read PSX controller data                                      *
*                                                                             *
******************************************************************************/
void Process_PSX( void )
{
	Read_PSX();								// Read PSX controller data 
	

///////////////////////////////////////////////////////////////////////////////////////
////////////////////START BUMP MODE////////////////////////////////////////////////////

	if ( ucOpMode == BUMP_MODE )
	{
		PORTD &= ~(1<<PD5);						// turn off red LED

		ucSensorStatus = Read_Sensors();		// switch and IR bump sensor data
		Process_Sensor_Data();					// determine what to do with pressed switches

		if( ( ( ucSensorStatus & _BV(1) ) != 0 ) && ( ( ucSensorStatus & _BV(0) ) != 0 ) ) // If both sensor are pushed
		{
			Back_Bot();                         // Back the bot up for turning
			Turn_Bot(rand() >> 7);              // Turn the bot a random direction
		}
		else if( ( ucSensorStatus & _BV(0) ) != 0 ) // The Right Sensor is activated
		{
			Back_Bot();                         // Back the bot up for turning
			Turn_Bot('1');                      // Turn the bot left
		}
		else if( ( ucSensorStatus & _BV(1) ) != 0 ) // The Left Sensor is activated
		{
			Back_Bot();                         // Back the bot up for turning
			Turn_Bot('0');                      // Turn the bot right
		}
		else                                    // ELSE No sensor is activated
		{
			ucMotor_L_Dir = 0;
			ucMotor_R_Dir = 0;                    // Set the motors to forward
			ucMotor_L_Speed = ucBotDigitalSpeed;
			ucMotor_R_Speed = ucBotDigitalSpeed;  // Go forward
			ucBumpMode = BUMP_FORWARD;
		}
	} // end else if (mode == BUMP_MODE)

	Display_data();


////////////////////END BUMP MODE//////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////
/*//////////////////MENU SELECT////////////////////////////////////////////////////////

The menu will prompt to tell the user to select
the menu item he/she wants and press X once they are on the selected item. The user must
press X to pass by this screen. 
The menu then uses the PSX controller to move around and select different options. The D-pad
and left analog stick are used to control scrolling through the menu screen. To select
the item displayed on the screen, one must press the X button. This is the only way to
select items. The bot waits until the X button has been release before performing any
actions in the selected mode. This is to ensure that when going into a driving mode, the
bot does not start backing up after selecting the menu item.


        Left Analog Stick Layout                 Right Analog Stick Layout
        
        
                          nothing                                    nothing
                                                                            
                             ^                                          ^
                             |                                          |  
                           \ | /                                      \ | /
              scroll left   )O(   scroll right              nothing    )O(    nothing
                           / | \                                      / | \
                             |                                          |  
                             v                                          v
                                                                            
                          nothing                                    nothing




            D-Pad:                                       Buttons:

                         nothing                                     nothing

                                                                       /_\  
                            ^                                                                          
            scroll left   < O >   scroll right          nothing    []       O    nothing
                            v                                   
                                                                        X

                         nothing                              Select Current Menu Item  */


///////////////////////////////////////////////////////////////////////////////////////
/*////////////////TANK MODE////////////////////////////////////////////////////////////

In analog mode, Tank Mode uses the two analog sticks to control each of the motors. As
shown below, the left analog stick controls the left motor, while the right analog stick
controls the right motor. The speed has a maximum of 128. The acceleration is based on 
the scaled of x^2 - 0x7F to place the value between 0x00 and 0x80.        

       
        Left Analog Stick Layout                 Right Analog Stick Layout
        
        
                   move left motor forward                      move right motor forward
                                                                            
                             ^                                          ^
                             |                                          |  
                           \ | /                                      \ | /
                 nothing    )O(    nothing                  nothing    )O(    nothing
                           / | \                                      / | \
                             |                                          |  
                             v                                          v
                                                                            
                   move left motor backward                     move right motor backward
        


In digital mode, Tank Mode uses the D-pad and the Buttons to control the left and right
motors, repectively. The maximum speed for digital mode is 117.

            D-Pad:                                       Buttons:

                   move left motor forward                      move right motor forward

                                                                       /_\  
                            ^                                                                          
                nothing   < O >   nothing                nothing   []       O    nothing
                            v                                   
                                                                        X

                   move left motor backward                     move right motor backward   */


    
    if( uiPSX_TimeoutCtr == 0 )				// process PSX activities since PSX is present
    {
		ucSensorStatus = Read_Sensors();	// switch and IR bump sensor data
		Process_Sensor_Data();				// determine what to do with pressed switches

        if( ucOpMode == TANK_MODE )
        { 
			PORTD |= (1<<PD5);				// turn on red LED

            //RESPONSE[8] is left stick forward and backward
            //RESPONSE[6] is right stick forward and backward                   

            //Look at response[3] bits 7:4 D-pad
            //Look at response[4] bits 7:4 buttons
            //bit 7 = left
            //bit 6 = down
            //bit 5 = right
            //bit 4 = up

			if ( response[ 8 ] < PSX_LOW_THRESHOLD)	// if the left stick is forward 
			{
				ucMotor_L_Dir = 0;			// Set left motor direction forward
				ucMotor_L_Speed = Calc_Analog_Speed( PSX_LOW_THRESHOLD - response[ 8 ] ); // speed between 0 ~ 120
			}
 
			else if ( response[ 8 ] > PSX_HIGH_THRESHOLD ) // ELSE IF the left stick is back,
            {                                        //  move left wheel back
				ucMotor_L_Dir = 1;                     // Set left motor direction backwards
				ucMotor_L_Speed = Calc_Analog_Speed( response[ 8 ] - PSX_HIGH_THRESHOLD ); // speed between 0 ~ 120
			}
			else            // ELSE the left stick is near the center, so do nothing
				ucMotor_L_Speed = 0;                   // Turn left motor off


			if ( response[ 6 ] < PSX_LOW_THRESHOLD)	// if the right stick is forward 
			{
				ucMotor_R_Dir = 0;			// Set right motor direction forward
				ucMotor_R_Speed = Calc_Analog_Speed( PSX_LOW_THRESHOLD - response[ 6 ] ); // speed between 0 ~ 120
			}
 
			else if ( response[ 6 ] > PSX_HIGH_THRESHOLD ) // ELSE IF the right stick is back,
            {                                        //  move left wheel back
				ucMotor_R_Dir = 1;                     // Set right motor direction backwards
				ucMotor_R_Speed = Calc_Analog_Speed( response[ 6 ] - PSX_HIGH_THRESHOLD ); // speed between 0 ~ 120
			}
			else            // ELSE the right stick is near the center, so do nothing
				ucMotor_R_Speed = 0;                   // Turn right motor off
		}//end else if(mode == TANK_MODE)


//////////////////////END TANK MODE////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
   
	}//end if(uiPSX_TimeoutCtr == 0)    

	uiPSX_TimeoutCtr++;
	if ( uiPSX_TimeoutCtr > 100 )
	{
		ucOpMode = BUMP_MODE;				// default mode when PSX is not present
		uiPSX_TimeoutCtr = 101;				//   don't allow to roll over to 0 (valid) by counting alone
	}
}//end function


/******************************************************************************
*                                                                             *
*  Function: Calculate Analog Motor Speed based on PSX response and analog    *
*            max speed ucBotAnalogSpeed                                       *
*                                                                             *
*  Input: speed_input (0-0xff)                                                *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: ucBotAnalogSpeed                                             *
*                                                                             *
*  Globals Write:                                                             *
*                                                                             *
*  Description:                                                               *
*                                                                             *
******************************************************************************/
unsigned int Calc_Analog_Speed( unsigned char speed_input )
{
	unsigned int uiCalcSpeed = (( speed_input * ucBotAnalogSpeed ) / PSX_LOW_THRESHOLD);

	return uiCalcSpeed;
}


/******************************************************************************
*                                                                             *
*  Function: Process ATtiny switch presses.                                   *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: ucSensorStatus (ATtiny sensor data)                          *
*                                                                             *
*  Globals Write:                                                             *
*                                                                             *
*  Description: Determine what to do with pressed switches.                   *
*                                                                             *
******************************************************************************/
void Process_Sensor_Data( void )
{
	if ( ucSensorStatus & 0b10000000 )		// check for S3 press
	{
//		printf("\e");						// clear display
		if (ucDisplayMode == 0)				// process button press as <L1>
			ucDisplayMode = DISPLAY_MAX;
		else
			ucDisplayMode--;
	}

	if ( ucSensorStatus & 0b01000000 )		// check for S4 press
	{
//		printf("\e");						// clear display
		if (ucOpMode == BUMP_MODE)			// process button press
			ucOpMode = TANK_MODE;
		else
			ucOpMode = BUMP_MODE;
	}

	if ( ucSensorStatus & 0b00100000 )		// check for S5 press
	{
//		printf("\e");						// clear display
		ucDisplayMode++;
		if (ucDisplayMode > DISPLAY_MAX) 	// process button press as <R1>
			ucDisplayMode = 0;
	}
}


/******************************************************************************
*                                                                             *
*  Function: Read PSX data                                                    *
*                                                                             *
*  Input: (None)                                                              *
*                                                                             *
*  Output: (None)                                                             *
*                                                                             *
*  Globals Read: poll[] PSX request data                                      *
*                                                                             *
*  Globals Write: response[]:                                                 *
*                                                                             *
*  response[3]: bit 7: L digital left        bit6: L digital down             *
*               bit 5: L digital right       bit4: L digital up               *
*               bit 3: Start                 bit2: L analog push              *
*               bit 1: R analog push         bit0: Select                     *
*                                                                             *
*  response[4]: bit 7: R digital left        bit6: R digital down             *
*               bit 5: R digital right       bit4: R digital up               *
*               bit 3: button R1             bit2: button L1                  *
*               bit 1: button R2             bit0: button L2                  *
*                                                                             *
*                                                                             *
*  response[5]: right analog stick left=0, right=0xff                         *
*                                                                             *
*  response[6]: right analog stick up=0, down=0xff                            *
*                                                                             *
*  response[7]: left analog stick left=0, right=0xff                          *
*                                                                             *
*  response[8]: left analog stick up=0, down=0xff                             *
*                                                                             *
*  ucMotor_L_Dir: foward=0, reverse=1                                         *
*                                                                             *
*  ucMotor_R_Dir: foward=0, reverse=1                                         *
*                                                                             *
*  ucMotor_L_Speed: 0-127 speed based on left analog stick                    *
*                                                                             *
*  ucMotor_R_Speed: 0-127 speed based on left analog stick                    *
*                                                                             *
*  Description: Read PSX controller data                                      *
*                                                                             *
******************************************************************************/
void Read_PSX( void )
{
//	unsigned char temp;

	for( uint8_t i = 0 ; i < 21 ; ++i )		// Poll the PSX controller for data.
	{
		_delay_us( 1 );
		response[ i ] = SPI_Receive( 1, poll[ i ] ); // address, data

		// Determine whether the incoming data stream corresponds to
		// digital-mode data, or analog-mode data.  Start doing this 'check'
		// only after we have read the second byte.
		if ( i >= 1 )
		{
			if ( ( response[ 1 ] == DIGITAL_MODE_BYTE ) && ( i == 4 ) )
				Set_PSX_Analog();
		} // end if()
	} // end for()
	Set_Slave_Addr( 7 );					// set to something other than addr 1 to segregate a unique msg

	if ( response[ 2 ] == 0x5A )
	{
		response[ 2 ] = 0;					// reset response in case PSX is unplugged
		uiPSX_TimeoutCtr = 0;

		ucPSX3Change = ucPSX3History ^ response[3]; // PSX byte 3 status: if any bit is a "1", this bit has changed
		ucPSX4Change = ucPSX4History ^ response[4]; // PSX byte 4 status: if any bit is a "1", this bit has changed
		ucPSX3History = response[3];				// retain history for next round
		ucPSX4History = response[4];				// retain history for next round

//		printf("\ePSX4History: %x\n", ucPSX4History);
//		printf("PSX4Change: %x\n", ucPSX4Change);
//		printf("PSX_TimeoutCtr: %x\n", uiPSX_TimeoutCtr);

///////////////////////////////////////////////////////////////////////////////////////
//                 process PSX <X> button to change operation modes                  //
///////////////////////////////////////////////////////////////////////////////////////

		if( (ucPSX4Change & _BV(6) ) && ( ( response[4] & _BV(6) ) == 0 ) ) // if <X> is pushed,
		{                               	//    toggle modes
			if (ucOpMode == BUMP_MODE)		// process button press
				ucOpMode = TANK_MODE;
			else
				ucOpMode = BUMP_MODE;
		}

///////////////////////////////////////////////////////////////////////////////////////
//          process PSX <L1> and <R1> button to change display modes                 //
///////////////////////////////////////////////////////////////////////////////////////

		if( (ucPSX4Change & _BV(2) ) && ( ( response[4] & _BV(2) ) == 0 ) ) // if <L1> is pushed,
		{                               	//    decrement display mode
			if (ucDisplayMode == 0)			// process button press
				ucDisplayMode = DISPLAY_MAX;
			else
				ucDisplayMode--;
		}

		if( (ucPSX4Change & _BV(3) ) && ( ( response[4] & _BV(3) ) == 0 ) ) // if <R1> is pushed,
		{                               	//    increment display mode
			ucDisplayMode++;
			if (ucDisplayMode > DISPLAY_MAX) // process button press
				ucDisplayMode = 0;
		}

///////////////////////////////////////////////////////////////////////////////////////
//          process PSX <L2> and <R2> button to change display modes                 //
///////////////////////////////////////////////////////////////////////////////////////

		if(( response[4] & _BV(0) ) == 0 ) 	// if <L2> is pushed,
		{                               	//    decrease bot speed
			if ( ucOpMode == BUMP_MODE )
			{
				if (ucBotDigitalSpeed > (MIN_DIGITAL_SPEED + 5)) // limit digital speed low end
					ucBotDigitalSpeed = ucBotDigitalSpeed - 5;
				Spkr_Freq = (ucBotDigitalSpeed << 4);
			}
			if ( ucOpMode == TANK_MODE )
			{
				if (ucBotAnalogSpeed > (MIN_ANALOG_SPEED + 5)) // limit analog speed low end
					ucBotAnalogSpeed = ucBotAnalogSpeed - 5;
				Spkr_Freq = (ucBotAnalogSpeed << 4);
			}
			uiSpkr_Timer = 200;			// limit speaker tone to 0.01 second
		}

		if(( response[4] & _BV(1) ) == 0 )	// if <R2> is pushed,
		{                               	//    increase bot speed
			if ( ucOpMode == BUMP_MODE )
			{
				if (ucBotDigitalSpeed < (MAX_DIGITAL_SPEED - 5)) // limit digital speed top end
					ucBotDigitalSpeed = ucBotDigitalSpeed + 5;
				Spkr_Freq = (ucBotDigitalSpeed << 4);
			}
			if ( ucOpMode == TANK_MODE )
			{
				if (ucBotAnalogSpeed < (MAX_ANALOG_SPEED - 5)) // limit analog speed top end
					ucBotAnalogSpeed = ucBotAnalogSpeed + 5;
				Spkr_Freq = (ucBotAnalogSpeed << 4);
			}
			uiSpkr_Timer = 200;			// limit speaker tone to 0.01 second
		}


///////////////////////////////////////////////////////////////////////////////////////
//          process PSX <square> button to turn brakes on (tank mode only)           //
///////////////////////////////////////////////////////////////////////////////////////

		if( (ucPSX4Change & _BV(7) ) && ( ( response[4] & _BV(7) ) == 0 ) ) // if <square> is pushed,
		{
			ucBrakeMode ^= 1;				// toggle brake mode
		}
	}
	else
		uiPSX_TimeoutCtr++;					// flag an invalid PSX read
}


/******************************************************************************
*                                                                             *
*  Function: PCINT0 Interrupt                                                 *
*                                                                             *
*  Input:    (None)                                                           *
*                                                                             *
*  Output:   (None)                                                           *
*                                                                             *
*  HWInput:  (PA0-7 pin change interrupt)                                     *
*                                                                             *
*  HWOutput: (None)                                                           *
*                                                                             *
*  Globals Read:  (None)                                                      *
*                                                                             *
*  Globals Write: (None)                                                      *
*                                                                             *
*  Description:  This interrupt is caused from a change on pin PA2.  This     *
*                can be from a 0->1 or a 1->0 transition on pin PA2.  Pin     *
*                PA2 monitors the charger input.  When PA2=1, the charger is  *
*                present and we can wake up from sleep.                       *
*                                                                             *
*******************************************************************************/
ISR(PCINT0_vect)
{
	if ( PINA & 0x04 )						// check for charger present
	{
		PRR = 0x00;							// turn power on for all peripherals 
		PORTD |= (1<<PD5);					// turn on red LED
		funcptr(); 							// Jump to Reset vector 0x0000 in Application Section. 
	}
} 
