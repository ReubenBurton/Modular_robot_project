//include this .c file's header file
#include "Controller.h"
#include <avr/interrupt.h>

//static function prototypes, functions only called in this file

/*-------------------------------PINOUTS for CONTROLLER----------------------------------

Joysticks
Right
SELECT		- D21 - PD0 - INT[0]
HORIZONTAL  - A1 - PF1 - ADC(1)
VERTICAL	- A0 - PF0 - ADC(0)
VCC			- 5V
GND			- Gnd

Left
SELECT		- D20 - PD1 - INT[1]
HORIZONTAL  - A14 - PK6 - ADC(14)
VERTICAl	- A15 - PK7 - ADC(15)
VCC			- 5V
GND			- Gnd

Button
PIN 19-GND	- D19 - PD2 - INT[2]

XBEE
Din			- D16 - PH1 - TXD2 - Serial 2 TX
Dout		- D17 - PH0 - RXD2 - Serial 2 RX
VCC			- 5V
GND			- Gnd

LCD (mostly taken care of in library)

DB7			- D53 - PB0 - SS - PCINT[0]
DB6			- D52 - PB1 - SCK - PCINT[1]
DB5			- D51 - PB2 - MOSI - PCINT[2]
PB4			- D50 - PB3 - MISO - PCINT[3]
E (enable)  - D47 - PL2 - BLE
R/W			- GND
RS			- D46 - PL3
VS			- middle tab of potentiometer
Vcc			- VCC(5V) arduino
Vss			- GND

------------------------------------------------------------------------------------------
*/



#define DATASPEED 100				//define your transmission speed here!
#define DEADZONE 50					//define deadzone in joysticks here 0-1023

typedef struct {
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
} packet_4;

typedef struct{
	uint8_t com;
	uint16_t data;
} rawdata;

//file scope variables
static char serial_string[200] = {0};
volatile packet_4 dataByte;

volatile bool new_message_received_flag=false;
volatile bool autostate, speedstate, haltstate = 0;			//button variables
volatile uint32_t last_debounce_time = 0;
volatile int debug = 0;										//DEBUG HERE!

void send18_serial2(uint8_t command, uint16_t variable);
void sendmotorcommand(uint8_t command, uint8_t motor_type, uint16_t motor_1, uint16_t motor_2);
rawdata received18_serial(packet_4 data_in);

uint16_t deadzone(uint16_t input);
int constrain(int value, int min_value, int max_value);
void buttonsetup();

/*-------------------------Button interrupt-----------------------------*/
//middle button 
ISR(INT2_vect)													//halt mode
{
	if((milliseconds - last_debounce_time) < 150)
	{
		return;
	}
	last_debounce_time = milliseconds;
	
	haltstate = !haltstate;
}

//left joystick button
ISR(INT1_vect)													//auto mode
{
	if((milliseconds - last_debounce_time) < 150)
	{
		return;
	}
	last_debounce_time = milliseconds;
	
	autostate = !autostate;
}

//right joystick button
ISR(INT0_vect)													//speed change
{
	if((milliseconds - last_debounce_time) < 150)
	{
		return;
	}
	last_debounce_time = milliseconds;
	
	speedstate = !speedstate;
}

/*----------------------------------------------------------------------------------------------*/

int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino
	
	uint16_t left_stick_y, left_stick_x, right_stick_y, right_stick_x = 0;
	
	uint16_t left_sensor, front_sensor, right_sensor = 0;		//store sensor values
	
	rawdata receivedpacket;
	
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	char lcd_string[16];
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	adc_init();
	lcd_init();
	buttonsetup();
	volatile uint8_t sentnow = 0;
	//volatile uint8_t bhoriz = 0;
	sei();
	lcd_command(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_ON));
	lcd_clrscr();
	_delay_ms(10); //some delay may be required
	
	while(1)
	{
		current_ms = milliseconds;
		
		right_stick_x = adc_read(1);
		right_stick_y = adc_read(0);
		
		left_stick_y = adc_read(15);
		left_stick_x = adc_read(14);
		
		//<-------------------commands to avoid flopping around like a fish--------------------->
		left_stick_x = deadzone(left_stick_x);
		left_stick_y = deadzone(left_stick_y);
		right_stick_x = deadzone(right_stick_x);
		right_stick_y = deadzone(right_stick_y);
		
		
		
		//<---------------------------------------------sending section---------------------------------------------------->
		
		if(current_ms-last_send_ms >= DATASPEED) //sending rate controlled here one message every 100ms (10Hz)
		{
			last_send_ms = current_ms;
				
			switch(sentnow)
			{
				case 0:
				
					sentnow++;
					break;
				case 1:
					send18_serial2(1, left_stick_y);		//command byte 1 = vertical
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, left_stick_y);
					}
					sentnow++;
					break;
				case 2:
					send18_serial2(2, left_stick_x);		//command byte 2 = horizontal
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, left_stick_x);
					}
					sentnow++;
					break;
				case 3:
					send18_serial2(3, autostate);			// auto mode boolean variable goes here
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, autostate);
					}
					sentnow++;
					break;
				case 4:
					send18_serial2(4, right_stick_y);		//right stick y
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, left_stick_y);
					}
					sentnow++;
					break;
				case 5:
					send18_serial2(5, right_stick_x);		//right stick x
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, right_stick_x);
					}
					sentnow++;
					break;
				case 6:
					send18_serial2(6, speedstate);			//speed adjust toggle
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, speedstate);
					}
					sentnow++;
					break;
				case 7:
					send18_serial2(7, haltstate);			//halt mode
					if(debug == 1)
					{
						sprintf(serial_string, "%d %d /r",sentnow, haltstate);
					}
					sentnow++;
					break;
				default:
					sentnow = 0;
				
				if(sentnow > 7)
				{
					sentnow = 0;
				}
			}
		}
		if(debug == 1)
		{
			serial0_print_string(serial_string);
		}
		//if a new byte has been received
		
		if(new_message_received_flag)
		{
			receivedpacket = received18_serial(dataByte);
			
			switch(receivedpacket.com)							//received data from robot
			{
				case 11:
					front_sensor = receivedpacket.data;			// front sensor - 11
				break;
				case 12:													
					left_sensor = receivedpacket.data;			// left sensor - 12
				break;
				case 13:
					right_sensor = receivedpacket.data;			// right sensor - 13
				break;
				default:
				break;
			}

			new_message_received_flag=false;	// set the flag back to false
		}
		//display section
		
		sprintf(lcd_string, "L:%4d mm R:%4d mm \r F:%d mm", left_sensor, right_sensor, front_sensor); //build LCD string display
		lcd_goto(0x00); //send cursor to top left
		lcd_puts(lcd_string); //write LCD string at cursor location
		
		//<---------------debug code here------------------>
		/*sprintf(lcd_string, "H%4d v%4d %d", horizontal,vertical,sentnow); //build LCD string display
		lcd_goto(0x40); //send cursor to top left
		lcd_puts(lcd_string); //write LCD string at cursor location
		*/
		//<------------------------------------------------->
		
	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		dataByte.byte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		dataByte.byte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for third parameter
		dataByte.byte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for fourth parameter
		dataByte.byte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}

void send18_serial2(uint8_t command, uint16_t variable)
{
	packet_4 output;
	
	output.byte1 = command;
	output.byte2 = (variable >> 12) & 0b00111111;
	output.byte3 = (variable >> 6) & 0b00111111;
	output.byte4 = variable & 0b00111111;
	output.byte2 = (0b01000000 | output.byte2); // Most significant bit is signed with a 1
	serial2_write_byte(0xFF); 		//send start byte = 255
	serial2_write_byte(output.byte1); 	//send first data byte: must be scaled to the range 0-253
	serial2_write_byte(output.byte2); 	//send second parameter: must be scaled to the range 0-253
	serial2_write_byte(output.byte3); 	//send first data byte: must be scaled to the range 0-253
	serial2_write_byte(output.byte4); 	//send second parameter: must be scaled to the range 0-253
	serial2_write_byte(0xFE);				//end byte = 254
	
}

void sendmotorcommand(uint8_t command, uint8_t motor_type, uint16_t motor_1, uint16_t motor_2)
{
	
	//motor type 1 = dc motors, motor type 2 = servos
	packet_4 output;
	
	output.byte1 = command;
	output.byte2 = motor_type;
	output.byte3 = (motor_1 >> 2);
	output.byte4 = (motor_2 >> 2);
	serial2_write_byte(0xFF);
	serial2_write_byte(output.byte1); 	//send first data byte: must be scaled to the range 0-253
	serial2_write_byte(output.byte2); 	//send second parameter: must be scaled to the range 0-253
	serial2_write_byte(output.byte3); 	//send first data byte: must be scaled to the range 0-253
	serial2_write_byte(output.byte4); 	//send second parameter: must be scaled to the range 0-253
	serial2_write_byte(0xFE);				//end byte = 254
}
rawdata received18_serial(packet_4 data_in)
{
	rawdata output;
	if(data_in.byte2 & 0b01000000)
	{
		output.data = data_in.byte4 | ((data_in.byte3 & 0b00111111) << 6) | ((data_in.byte2 & 0b00111111) << 12);
		output.com = data_in.byte1;
	}
	return output;
}

uint16_t deadzone(uint16_t input)
{
	uint16_t output;
	int middle_value = 512;
	int offset_value = DEADZONE;
	if ((input > middle_value - offset_value) && (input < middle_value + offset_value))
	{
		output = middle_value;
	}
	else
		output = input;
	return output;
}

//<----------------generated by chatGPT------------------------>
int constrain(int value, int min_value, int max_value)
{
	if (value < min_value)
	{
		return min_value;
	}
	else if (value > max_value)
	{
		return max_value;
	}
	else
	{
		return value;
	}
}

void buttonsetup()
{
	DDRD &= ~(1<<PD2)|(1<<PD1)|(1<<PD0);// INT0 is also PD0 and we set the DDR to input
	PORTD |= (1<<PD2)|(1<<PD1)|(1<<PD0);// enable pullup resistor on PD0, PD1, PD2
	EIMSK |= (1<<INT2)|(1<<INT1)|(1<<INT0); // enable INT0, INT1, INT2
}
