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
Author: Robert H

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


## Digital Pins

|Silkscreen Pin   |Microcontroller pin   |Port   |Digital Pin   |Comms Pin   |Timer Pin| Interrupt Pin|
|---|---|---|---|---|---|---|
|0   |PE0   |PORTE   |D0/RX0   |RXD0   |   |PCINT[8]   |
|1   |PE1   |PORTE   |D1/TX0   |TXD0   |   |   |
|~2  |PE4   |PORTE   |~D2   |   |OC3B   |INT[4]   |
|~3  |PE5   |PORTE   |~D3   |   |OC3C   |INT[6]   |
|~4  |PG5   |PORTG   |~D4   |   |OC0B   |   |
|~5  |PE3   |PORTE   |~D5   |   |OC3A   |   |
|~6  |PH3   |PORTH   |~D6   |   |OC4A   |   |
|~7  |PH4   |PORTH   |~D7   |   |OC4B   |   |
|~8  |PH5   |PORTH   |~D8   |   |OC4C   |   |
|~9  |PH6   |PORTH   |~D9   |   |OC2B   |   |
|~10 |PB4   |PORTB   |~D10  |   |OC2A   |PCINT[4]   |
|~11 |PB5   |PORTB   |~D11  |   |OC1A   |PCINT[5]   |
|~12 |PB6   |PORTB   |~D12  |   |OC1B   |PCINT[6]   |
|~13 |PB7   |PORTB   |~D13  |   |OC0A/OC1C|PCINT[7]   |
|-   |PD1   |PORTD   |D20/SCA   |SDA   |   |INT[1]   |
|-   |PD0   |PORTD   |D21/SCL   |SCL   |   |INT[0]   |
|14  |PJ1   |PORTJ   |D14/TX3   |TXD3   |   |   |
|15  |PJ0   |PORTJ   |D15/RX3   |RXD3   |   |   |
|16  |PH1   |PORTH   |D16/TX2   |TXD2   |   |   |
|17  |PH0   |PORTH   |D17/RX2   |RXD2   |   |   |
|18  |PD3   |PORTD   |D18/TX1   |TXD1   |   |   |
|19  |PD2   |PORTD   |D19/RX1   |RXD1   |   |   |
|20  |PD1   |PORTD   |D20/SDA   |SDA   |   |   |
|21  |PD0   |PORTD   |D21/SCL   |SCL   |   |   |
|22   |PA0   |PORTA   |D22   |AD0   |   |   |
|23   |PA1   |PORTA   |D23   |AD1   |   |   |
|24   |PA2   |PORTA   |D24   |AD2   |   |   |
|25   |PA3   |PORTA   |D25   |AD3   |   |   |
|26   |PA4   |PORTA   |D26   |AD4   |   |   |
|27   |PA5   |PORTA   |D27   |AD5   |   |   |
|28   |PA6   |PORTA   |D28   |AD6   |   |   |
|29   |PA7   |PORTA   |D29   |AF7   |   |   |
|30   |PC7   |PORTC   |D30   |A15   |   |   |
|31   |PC6   |PORTC   |D31   |A14   |   |   |
|32   |PC5   |PORTC   |D32   |A13   |   |   |
|33   |PC4   |PORTC   |D33   |A12   |   |   |
|34   |PC3   |PORTC   |D34   |A11   |   |   |
|35   |PC2   |PORTC   |D35   |A10   |   |   |
|36   |PC1   |PORTC   |D36   |A9    |   |   |
|37   |PC0   |PORTC   |D37   |A8    |   |   |
|38   |PD7   |PORTD   |D38   |      |T0 |   |
|39   |PG2   |PORTG   |D39   |BLE   |   |   |
|40   |PG1   |PORTG   |D40   |RD    |   |   |
|41   |PG0   |PORTG   |D41   |WR    |   |   |
|42   |PL7   |PORTL   |D42   |      |   |   |
|43   |PL6   |PORTL   |D43   |   |   |   |
|~44  |PL5   |PORTL   |D44   |   |   |   |
|~45  |PL4   |PORTL   |D45   |   |   |   |
|~46  |PL3   |PORTL   |D46   |   |   |   |
|47   |PL2   |PORTL   |D47   |   |   |   |
|48   |PL1   |PORTL   |D48   |   |   |   |
|49   |PL0   |PORTL   |D49   |   |   |   |
|50   |PB3   |PORTB   |D50   |MISO   |   |PCINT[3]   |
|51   |PB2   |PORTB   |D51   |MOSI   |    |PCINT[2]   |
|52   |PB1   |PORTB   |D52   |SCK    |    |PCINT[1]   |
|53   |PB0   |PORTB   |D53   |SS     |    |PCINT[0]   |
