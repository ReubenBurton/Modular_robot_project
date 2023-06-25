# ARDUINO MEGA - Atmel 2560

## Analog Pins
|   Silkscreen Pin	|Microcontroller Pin   	| Port  	|   	|   	|
|---	|---	|---	|---	|---	|
|A0   	|PF0   	|PORTF   	|   	|   	|
|A1   	|PF1   	|PORTF   	|   	|   	|
|A2    	|PF2   	|PORTF   	|   	|   	|
|A3     |PF3   	|PORTF   	|   	|   	|
|A4     |PF4   	|PORTF   	|   	|   	|
|A5     |PF5   	|PORTF   	|   	|   	|
|A6     |PF6   	|PORTF   	|   	|   	|
|A7     |PF7   	|PORTF   	|   	|   	|
|A8     |PK0   	|PORTF   	|   	|   	|
|A9     |PK1   	|PORTF   	|   	|   	|
|A10    |PK2   	|PORTF   	|   	|   	|
|A11    |PK3   	|PORTF   	|   	|   	|
|A12    |PK4   	|PORTF   	|   	|   	|
|A13    |PK5   	|PORTF   	|   	|   	|
|A14    |PK6   	|PORTF   	|   	|   	|
|A15    |PK7   	|PORTF   	|   	|   	|

You could probably encapsulate this in a function:
### Usage Example (Analog Pin 0):
Author: Robert

Initialization
uint16_t output;
ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADC prescaler = 128
ADMUX = (1<<REFS0); //use internal Vcc AREF, make sure capacitor is attached to AREF pin
ADCSRB = 0; //ADC0 source, single conversion mode
ADCSRA |= (1<<ADEN); //enable ADC

Read 
ADCSRB &= ~(1<<MUX5); //clear mux5
ADMUX = (ADMUX&(0xE0))|(0&0x07); //set remaining mux bits
while(ADCSRA&(1<<ADSC)){} //wait for conversion to finish
output = ADC;



