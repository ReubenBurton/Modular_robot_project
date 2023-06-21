#include "Robot.h"

// Constants
#define DISTANCE 40			// Distance to Wall


void autorobot(bool motor, bool motordirection, uint16_t speedpercent);
uint16_t short_range(uint16_t sensor_output);

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0; // data bytes received
volatile bool new_message_received_flag=false; // for receiving a packet

int main(void)
{
	//init
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino

	//defining variable
	uint16_t horizontal, vertical, sidebutton, sidebutton2, horizontal_right = 0; //packet receiving
	
	uint32_t fs,rs,ls = 0; //Front, Right, Left Sensor logic
	int16_t lm,rm = 0; //left motor and right motor mapping logic

	uint16_t receivedpacket = 0; //for receiving packets
	uint8_t commandbyte = 0; //for receiving packets
	
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;	//data bytes sent
	
	uint32_t current_ms=0, last_send_ms=0;	//used for timing the serial send

	int sendnow = 0;
	//Setting up registers
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

		//PWN Signal stuff for servo
	uint16_t duty = 0;
	TCCR1A |= (1<<WGM11)|(1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1)|(1<<COM1B0); //mode 10, prescalar 64
	ICR1 = 2499; // comparison value
	DDRB|= (1 << PB5)|(1 << PB6);

		//PWM Signal stuff for left and right motor
	TCCR3A |= (1 << WGM31)|(1 << COM3A1)|(1 << COM3A0)|(1 << COM3B1)|(1 << COM3B0);
	TCCR3B |= (1 << WGM33)|(1 << CS31)|(1<<CS30);		//mode 10, prescaler 64
	ICR3 = 625; //TOP value for motor (comparison value)
	
	DDRE |= (1 <<PE4)|(1 << PE3);	//pwm signals

	DDRA |= (1 <<PA0)|(1 <<PA1)|(1 <<PA2)|(1 <<PA3); //control signals for H bridge

	DDRA |= (1 <<PA6); //INIT POWER LED OUT

	PORTA = 0;
	
	//general setup
	milliseconds_init();
	adc_init();
	sei();
	
	_delay_ms(10); //some delay may be required

	while(1)
	{
		//Define the current time. Used throughout code
		current_ms = milliseconds;
		
		//check voltage of battery. If below 7v, turn LED ON
		if(adc_read(4) < 800) 
		{
			PORTA|=(1<<PA6);
		}
		else
		{
			PORTA&=~(1<<PA6);
		}
		
		sidebutton = 0;
		
		//Loop for control of robot. If sidebutton = 0, remote control/manual. If sidebutton = 1, autonomous mode
		if(sidebutton == 0){
			
			// L/R motor direction mapping
			rm = vertical + horizontal - 1023;
			lm = vertical - horizontal;

			// L/R motor speed control
			OCR3B = -0.611*abs(lm)*2+625; //Left speed
			OCR3A = -0.611*abs(rm)*2+625; //Right speed

			// Servo motor control
			OCR1A = -0.122*horizontal_right+2375;
			
			// Code controlling direction of motor spin
			if (lm >= 0) {
				
				//turn motor left
				PORTA|=(1<<PA0);
				PORTA&=~(1<<PA1);
				//PA0 = 1;
				//PA1 = 0;
			}
			else{
				//turn motor right
				PORTA&=~(1<<PA0);
				PORTA|=(1<<PA1);
				//PA0 = 0;
				//PA1 = 1;
			}
			
			//turn other motor left
			if (rm >= 0) {
				PORTA&=~(1<<PA2);
				PORTA|=(1<<PA3);
				//PA2 = 0;
				//PA3 = 1;
			}
			else{
				//turn other motor right
				PORTA|=(1<<PA2);
				PORTA&=~(1<<PA3);
				//PA2 = 1;
				//PA3 = 0;
			}

		}
		
		//code for autonomous
		else if(sidebutton == 1){

			//Reading sensor data
			fs = short_range(adc_read(0)); //(F)ront (S)ensor
			ls = short_range(adc_read(1)); //(L)eft (S)ensor
			rs = short_range(adc_read(2)); //(R)ight (S)ensor

			//logic for going forward
			if (fs>60){ //check if almost hitting a wall. if not near a wall
				if ((-60 <= ls - rs) && (ls-rs <= 60)){ //check if reasonble in the center
					autorobot(0,1,25);		//left motor forward 25%
					autorobot(1,1,25);		//right motor forward 25%
				}
				else if(ls>rs){  //ls is greater than rs by more than margin. As such need to turn right more
					autorobot(0,1,25); //left forward 25%
					autorobot(1,1,100); //right forward 100%
				}
				else if(rs>ls){  //rs is greater than ls by more than margin. As such need to turn light more
					autorobot(0,1,100); //left forward 100%
					autorobot(1,1,25); //right forward 25%
				}
				else{ //oh no confused state. backup
					autorobot(0,0,100); //reverse left
					autorobot(1,0,100); //reverse right
				}
			}
			//logic for turning
			else { //this case would be fs<value, meaning near end of a path
				if (ls>rs){ //would mean there is a way clear on the left. so turn left
					autorobot(0,1,100); //left motor reverse
					autorobot(1,0,100); //right motor forward
				}
				else if (rs>ls){ //would mean way clear on the right. so turn right
					autorobot(1,0,100); //right motor reverse
					autorobot(0,1,100); //left motor forward
				}
				else{ //oh no case. just back up a bit
					autorobot(0,0,100); //left motor reverse
					autorobot(1,0,100); //right motor reverse
				}
				// so when a corner is reached, turn into it slightly before front sensor is activated. After which, itll turn left until it can go forward.
			}

			//code for if button is pressed
			if (sidebutton2 == 1){
				sidebutton2 = 0; //change the button state back to 0

				autorobot(0,0,100); //left motor reverse
				autorobot(1,0,100); //right motor reverse

				_delay_ms(100);

				OCR1A = -0.122*1024+2375; //servo left

				_delay_ms(500);

				OCR1A = -0.122*0+2375; //servo right

				_delay_ms(500);

				OCR1A = -0.122*512+2375; //reset servo to start position

			}
			
		}
		
		
		//receiving data
		if(new_message_received_flag)
		{ //if new data received
			if(dataByte2 & 0b01000000)
			{
				receivedpacket = dataByte4 | ((dataByte3 & 0b00111111) << 6) | ((dataByte2 & 0b00111111) << 12);
			}
			
			switch(dataByte1) //command byte is used here!
			{
				case 0: //do nothing
				break;
				case 1:
				vertical = receivedpacket;
				break;
				case 2:
				horizontal = receivedpacket;
				case 3:
				sidebutton = receivedpacket;
				case 4:
				sidebutton2 = receivedpacket;
				case 5:
				horizontal_right = receivedpacket;
				break;
			}
			
			sprintf(serial_string, "vertical = %u, horizontal = %u\r", OCR1A, OCR1B);
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received

			new_message_received_flag=false;	// set the flag back to false
		}

		//sending data
		switch (sendnow){
			case 0:
				sendnow++;
				break;

			case 1: //send left sensor
				sendnow++;

				sendDataByte1 = 1;
				sendDataByte2 = short_range(adc_read(0))/10; //(F)ront (S)ensor
				serial2_write_byte(0xFF); 		//send start byte = 255
				serial2_write_byte(sendDataByte1);
				serial2_write_byte(sendDataByte2);
				serial2_write_byte(0xFE);

				break;

			case 2: //send right sensor
				sendnow++;

				sendDataByte1 = 2;
				sendDataByte2 = short_range(adc_read(1))/10; //(L)eft (S)ensor
				serial2_write_byte(0xFF); 		//send start byte = 255
				serial2_write_byte(sendDataByte1);
				serial2_write_byte(sendDataByte2);
				serial2_write_byte(0xFE);

				break;

			case 3: //send front sensor
				sendnow++;

				sendDataByte1 = 3;
				sendDataByte2 = short_range(adc_read(2))/10; //(R)ight (S)ensor
				serial2_write_byte(0xFF); 		//send start byte = 255
				serial2_write_byte(sendDataByte1);
				serial2_write_byte(sendDataByte2); 	//send first data byte: must be scaled to the range 0-253
				serial2_write_byte(0xFE);

				break;
			
		}
	}
	return(1);
}



ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			
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


// function used for motor control in autonomous mode
void autorobot(bool motor, bool motordirection, uint16_t speedpercent){ //motor 0 = left motor, motor 1 = right motor. motor direction 0 = forward (left), motor direction 1 = bakwards (right)
	if (motor == 0){
		OCR3B = -0.611*speedpercent/100*1023+625;
		if (motordirection == 0){
			PORTA|=(1<<PA0);
			PORTA&=~(1<<PA1);
		}
		else if (motordirection == 1){
			PORTA&=~(1<<PA0);
			PORTA|=(1<<PA1);
		}
		else{
			return;
		}

	}
	else if (motor == 1){
		OCR3A = -0.611*speedpercent/100*1023+625;
		if (motordirection == 0){
			PORTA&=~(1<<PA2);
			PORTA|=(1<<PA3);
		}
		else if (motordirection == 1){
			PORTA|=(1<<PA2);
			PORTA&=~(1<<PA3);
		}
		else{
			return;
		}
	}
	else{
		return;
	}
}

//function for mapping IR sensor data to distance in mm
uint16_t short_range(uint16_t sensor_output)
{
	uint16_t distance;
	distance = ((23362/sensor_output) + 7.13);
	return distance;
}

