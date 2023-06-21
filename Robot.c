//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file

/*----------------------------PINOUT OF Robot Cart------------------------------------
DC Motors
	PWM for motors
		PE3------>Pin 5	(OC3A)
		PE4------>Pin 2	(OC3B)
	Enable-Direction control
		PA0------>Pin 22
		PA1------>Pin 23
		PA2------>Pin 24
		PA3------>Pin 25
Servo
	PWM for servos
		PB5------>Pin 11	(OC1A)
		PB6------>Pin 12	(OC1B)
	Other Connections
		+ and - must be wired to 5V or 6V externally
		
Sensors
	Battery Voltage divider
		R1 = 8.2k resistor
		R2 = 10k resistor
		PA6------>Pin 28 (Led indicator light)
		PF4------>ADC(4) (Vsense)
	
	Range sensors (short)
		PF0------>A0 - ADC(0) (Front Sensor)
		PF1------>A1 - ADC(1) (Left Sensor)
		PF2------>A2 - ADC(2) (Right Sensor)
		All other wires are 5V and GND

---------------------------------------------------------------------------------------
*/

#define DATASPEED 100				//define your data transmission speed here!
#define WALLSPACE 50				//distance from robot to front wall
#define LEFTWALL 50				//distance to left wall
#define RIGHTWALL 50				//distance to right wall

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

typedef struct{
	uint16_t motor_1;
	uint16_t motor_2;
	} motors;


//file scope variables
static char serial_string[200] = {0};
volatile packet_4 dataByte;		// data bytes received
volatile bool new_message_received_flag=false;
volatile uint8_t sentnow = 0;

/*-----------------debugging variables---------------------*/
volatile int debug = 0;         //check inputs and outputs
volatile int inversion = 1;		//change to change motor direction


/*------------------3 button variables----------------------*/
volatile int motorspeed = 0;	    //Change this to go fast or slow
volatile bool auto_mode = false;	//autonomous mode toggle
volatile bool halt = false;			//halt mode toggle

void send18_serial2(uint8_t command, uint16_t variable);
rawdata received18_serial(packet_4 data_in);

void servo_init(void);
void battery_check(uint16_t read);

//<-----------------Motor Control Functions Here------------------->

void motor_init(void);

void L_motor_left(void);
void L_motor_right(void);
void R_motor_left(void);
void R_motor_right(void);


void motor_joystick(uint16_t horizontal, uint16_t vertical);
void motor_individual(char direction_left_motor, char direction_right_motor, int percentage_left, int percentage_right);


void servo_write(int pin, uint16_t input);
void speed_change(uint16_t change);

void stop_and_look();

//<---------------Sensor Calibration functions---------------------->

uint16_t short_range(uint16_t sensor_output);
uint16_t long_range(uint16_t sensor_output);

//<---------------Autonomous Mode Functions-------------------------->
void james_autonomous(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor);
void reuben_autonomous(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor);




int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// micro controller communication to/from another Arduino
	// or loop back communication to same Arduino
	
	uint16_t front_sensor, left_sensor, right_sensor = 100; //distance sensor
	
	uint16_t horizontal, vertical = 100;	//servo motors
	
	rawdata receivedpacket;
	
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	DDRA |= (1 <<PA6);		//enable led for battery checker
	
	servo_init();
	motor_init();
	
	milliseconds_init();
	adc_init();
	
	sei();
	
	_delay_ms(10); //some delay may be required
	
	while(1)
	{
		current_ms = milliseconds;
		battery_check(adc_read(4));
		
		//<------------------------JOYSTICK MOTION CONTROL HERE------------------------>
		
		front_sensor = short_range(adc_read(0));
		left_sensor = short_range(adc_read(1));
		right_sensor = short_range(adc_read(2));
		
		if(debug == '1')
		{
			sprintf(serial_string, "front = %u, left = %u, right = %u \r", front_sensor, left_sensor, right_sensor);
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
			
		}
		
		//<----------------------------------sending section--------------------------------------------->
		
		if(current_ms-last_send_ms >= DATASPEED) //sending rate controlled here one message every 100ms (10Hz)
		{
			//send data from distance sensors
			switch(sentnow)
			{
				case 0:
					send18_serial2(11, front_sensor);		//command byte 11	send distance front
					sentnow++;
					break;
				case 1:
					send18_serial2(12, left_sensor);		//command byte 12 = send distance left
					sentnow++;
					break;
				case 2:
					send18_serial2(13, right_sensor);		//command byte 13 = send distance right
					sentnow++;
					break;
				default:
					sentnow = 0;
				if(sentnow > 2)
				{
					sentnow = 0;
				}
			}
			last_send_ms = current_ms;
			
		}

		//if a new byte has been received
		if(new_message_received_flag)
		{
			receivedpacket = received18_serial(dataByte);
			
			switch(receivedpacket.com) //command byte is used here!
			{
				case 0:
				//do nothing
					break;
				case 1:													// left stick y
					vertical = receivedpacket.data;
					//sprintf(serial_string, "horizontal = %u %d \n\r", horizontal, dataByte1);
					//serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
					break;
				case 2:													// left stick x
					horizontal = receivedpacket.data;
				
					//sprintf(serial_string, "vertical = %u %d \r", vertical, dataByte1);
					//serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
					break;
				case 3:
					auto_mode = receivedpacket.data & 0b00000001;		//checks for auto mode
					break;
				case 4:
					servo_write(11, receivedpacket.data);				// right stick y
					break;
				case 5:
					servo_write(12, receivedpacket.data);				// right stick x
					break;
				case 6:
					speed_change(receivedpacket.data);					//change the speed on all DC motors
					break;
				case 7:
					halt = receivedpacket.data & 0b00000001;			//halt mode
					break;
				default:
					break;
			}

			new_message_received_flag=false;	// set the flag back to false
		}
		/*-----------------------------implement logic here------------------------------*/
		
		if(halt == true)
		{
			if(auto_mode == false)
			{
				motor_joystick(horizontal, vertical);
			}
			else
			{
				reuben_autonomous(front_sensor, left_sensor, right_sensor);
				//james_autonomous(front_sensor, left_sensor, right_sensor);
			}
		}
		
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
		case 3: //waiting for second parameter
		dataByte.byte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		dataByte.byte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
						
			new_message_received_flag = true;
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

void servo_init(void)
{
	/*
		PINOUT:
			PWM for servos
				PB5------>Pin 11	(OC1A)
				PB6------>Pin 12	(OC1B)
			Other Connections
				+ and - must be wired to 5V or 6V externally

		Timer 1 (16-bit)
		Mode 8 - PWM, Phase and Frequency Correct
		Prescaler 64
		Frequency ~50Hz (Top Value 2499)
		Duty cycle 1.5-2.5ms (7.5% - 12.5%)
		Output on compare (inverting)
	*/
	
	TCCR1A |= (1 << WGM11)|(1 << COM1A1)|(1 << COM1A0)|(1 << COM1B1)|(1 << COM1B0);
	TCCR1B |= (1 << WGM13)|(1 << CS11)|(1<<CS10);
	
	ICR1 = 2499;
	
	DDRB |= (1 <<PB5)|(1 << PB6);
}

void motor_init(void)
{
	
	/*
		PINOUT:
			PWM for motors
				PE3------>Pin 5	(OC3A)
				PE4------>Pin 2	(OC3B)
			Enable-Direction control
				PA0------>Pin 22
				PA1------>Pin 23
				PA2------>Pin 24
				PA3------>Pin 25

		Timer 3 (16 bit)
		Mode 10 (PWM Phase Correct)
		Prescaler 64
		Frequency 200 Hz (Top Value 625)
		Duty Cycle 0-100%
		Output on compare (inverting)
	*/
	
	//PWM Signal stuff
	TCCR3A |= (1 << WGM31)|(1 << COM3A1)|(1 << COM3A0)|(1 << COM3B1)|(1 << COM3B0);
	TCCR3B |= (1 << WGM33)|(1 << CS31)|(1<<CS30);		//mode 10, prescaler 64
	
	ICR3 = 625;
	
	DDRE |= (1 <<PE4)|(1 << PE3);							//pwm signals
	DDRA |= (1 <<PA0)|(1 <<PA1)|(1 <<PA2)|(1 <<PA3);		//control signals
}


void L_motor_left(void)
{
	if(inversion == 1)
	{
		PORTA&=~(1<<PA0);
		PORTA|=(1<<PA1);
	}
	else
	{
		PORTA|=(1<<PA0);
		PORTA&=~(1<<PA1);
	}
}

void L_motor_right(void)
{
	if(inversion == 1)
	{
		PORTA|=(1<<PA0);
		PORTA&=~(1<<PA1);
	}
	else
	{
		PORTA&=~(1<<PA0);
		PORTA|=(1<<PA1);
	}
}

void R_motor_left(void)
{
	if(inversion == 1)
	{
		PORTA|=(1<<PA2);
		PORTA&=~(1<<PA3);
	}
	else
	{
		PORTA&=~(1<<PA2);
		PORTA|=(1<<PA3);
	}
}

void R_motor_right(void)
{
	if(inversion == 1)
	{
		PORTA&=~(1<<PA2);
		PORTA|=(1<<PA3);
	}
	else
	{
		PORTA|=(1<<PA2);
		PORTA&=~(1<<PA3);
	}
}

void motor_joystick(uint16_t horizontal, uint16_t vertical)
{
	int16_t lm,rm = 0;
	rm = vertical + horizontal - 1023;
	lm = vertical - horizontal;

	OCR3B = -0.611*abs(lm)*2 + 625; //Left speed
	OCR3A = -0.611*abs(rm)*2 + 625; //Right speed
	
	if (lm >= 0) {
		L_motor_left();
	}
	else{
		L_motor_right();
	}
	if (rm >= 0) {
		R_motor_left();
	}
	else{
		R_motor_right();
	}
}

void motor_individual(char direction_left_motor, char direction_right_motor, int percentage_left, int percentage_right)
{
	/*	
		Usage
			direction commands
				"f" = forward
				"r" = reverse
				"s" = stop
			percentage commands
				0 - 100----->0 - 100%
	*/
	
	switch(direction_left_motor)
	{
		case 'f':
			L_motor_left();
			OCR3B = -6.25*percentage_left + 625;
		break;
		case 'r':
			L_motor_right();
			OCR3B = -6.25*percentage_left + 625;
		break;
		case 's':
			OCR3B = 625;		//TOP VALUE IN INVERTING PWM = 0
		break;
	}
	switch(direction_right_motor)
	{
		case 'f':
			R_motor_left();
			OCR3A = -6.25*percentage_right + 625;
		break;
		case 'r':
			R_motor_right();
			OCR3A = -6.25*percentage_right + 625;
		break;
		case 's':
			OCR3A = 625;
		break;
	}
}

void servo_write(int pin, uint16_t input)
{
	/*
		input = 0 - 1023
		11 = horizontal
		12 = vertical
	*/

	switch(pin)
	{
		case 11:
			OCR1A = -0.122*(input) + 2375;
		break;
		case 12:
			OCR1B = -0.122*(input) + 2375;
		break;
		
	}
}

void speed_change(uint16_t change)
{
	if((change & 0b00000001) == 0)
	{
		return;
	}
	else
	{
		motorspeed = motorspeed + 10;
	}
	if(motorspeed >= 100)
	{
		motorspeed = 0;
	}
}

uint16_t short_range(uint16_t sensor_output)
{
	uint16_t distance;
	distance = ((23362/sensor_output) + 7.13); // Calibration for short range sensor
	return distance;
}

uint16_t long_range(uint16_t sensor_output)
{
	uint16_t distance;
	distance = ((66994/sensor_output-53)); //calibration for long range sensor
	return distance;
}

void battery_check(uint16_t read)
{
	if (read < 800)
	{
		PORTA|= (1 <<PA6);	
	}
	else
		PORTA&=~(1 <<PA6);	
}


void james_autonomous(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor)
{
	//logic for forward
	if(front_sensor > WALLSPACE)			//check if almost hitting a wall. if not near a wall
	{
		if((-(LEFTWALL) <= left_sensor-right_sensor) && (left_sensor - right_sensor <= (RIGHTWALL)))		//check if reasonably in the center
		{
			motor_individual('f', 'f', 75*(motorspeed/100), 75*(motorspeed/100));
		}
		else if(left_sensor > right_sensor)								//left_sensor is greater than right_sensor by more than margin. As such need to turn right more
		{
			motor_individual('f','f', 25*(motorspeed/100), 100*(motorspeed/100));
		}
		else if (right_sensor > left_sensor)
		{
			motor_individual('f','f', 100*(motorspeed/100), 25*(motorspeed/100));							 //right_sensor is greater than left_sensor by more than margin. As such need to turn light more
		}
		else
		{
			motor_individual('r','r', 50*(motorspeed/100), 50*(motorspeed/100));							//oh no confused state. backup
		}
	}		//Logic for turning
	else
	{
		if (left_sensor>right_sensor){			//would mean there is a way clear on the left. so turn left
			motor_individual('r', 'f', 75*(motorspeed/100), 75*(motorspeed/100));	//left reverse, right forward
		}
		else if (right_sensor > left_sensor){	//would mean way clear on the right. so turn right
			motor_individual('r', 'f', 75*(motorspeed/100), 75*(motorspeed/100));	//right reverse, left forward
		}
		else{															//oh no case. just back up a bit
			motor_individual('f','f', 50*(motorspeed/100), 50*(motorspeed/100));	//left reverse, right forward
		}
		// so when a corner is reached, turn into it slightly before front sensor is activated. After which, it'll turn left until it can go forward.
	}
}

void reuben_autonomous(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor)
{
	if(left_sensor <= LEFTWALL)														//too close to left wall
	{
		motor_individual('f','r', 75*(motorspeed/100), 75*(motorspeed/100));		//turn right
	}
	else if(left_sensor >= LEFTWALL + 100)											//too far away from right wall
	{
		motor_individual('r','f', 75*(motorspeed/100), 75*(motorspeed/100));		//turn left
	}
	else if(front_sensor <= WALLSPACE)												//if obstacle in front
	{
		motor_individual('r','f', 75*(motorspeed/100), 75*(motorspeed/100));		//turn left
	}
	else if(right_sensor <= RIGHTWALL)												//too close to right wall
	{
		motor_individual('r','f', 75*(motorspeed/100), 75*(motorspeed/100));		//turn left
	}
	else
	{
		motor_individual('f','f', 75*(motorspeed/100), 75*(motorspeed/100));		//Go forward if everything good
	}
}

void stop_and_look()
{
	motor_individual('s','s',motorspeed, motorspeed);								//brings the motors to a halt
}
