# ARDUINO MEGA - Atmel 2560

## Analog Pins
|   Silkscreen Pin	|Microcontroller Pin   	|Port  	|Digital Pin   	|Interrupt Register   	|
|---	|---	|---	|---	|---	|
|A0   	|PF0   	|PORTF   	|D54   	|   	|
|A1   	|PF1   	|PORTF   	|D55   	|   	|
|A2    	|PF2   	|PORTF   	|D56   	|   	|
|A3     |PF3   	|PORTF   	|D57   	|   	|
|A4     |PF4   	|PORTF   	|D58   	|   	|
|A5     |PF5   	|PORTF   	|D59   	|   	|
|A6     |PF6   	|PORTF   	|D60   	|   	|
|A7     |PF7   	|PORTF   	|D61   	|   	|
|A8     |PK0   	|PORTK   	|D62   	|PCINT[16]  	|
|A9     |PK1   	|PORTK   	|D63   	|PCINT[17]   	|
|A10    |PK2   	|PORTK   	|D64   	|PCINT[18]   	|
|A11    |PK3   	|PORTK   	|D65   	|PCINT[19]   	|
|A12    |PK4   	|PORTK   	|D66   	|PCINT[20]   	|
|A13    |PK5   	|PORTK   	|D67   	|PCINT[21]   	|
|A14    |PK6   	|PORTK   	|D68   	|PCINT[22]   	|
|A15    |PK7   	|PORTK   	|D69   	|PCINT[23]   	|

You could probably encapsulate this in a function:
### Usage Example (Analog Pin 0):
Author: Robert

**Initialization:** 

uint16_t output;  
ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     // ADC prescaler = 128  
ADMUX = (1<<REFS0);                            //use internal Vcc AREF, make sure capacitor is attached to AREF pin  
ADCSRB = 0;                                    //ADC0 source, single conversion mode  
ADCSRA |= (1<<ADEN);                           //enable ADC  

**Read:** 

ADCSRB &= ~(1<<MUX5);                           //clear mux5  
ADMUX = (ADMUX&(0xE0))|(0&0x07);                //set remaining mux bits  
while(ADCSRA&(1<<ADSC)){}                       //wait for conversion to finish  
output = ADC;  


