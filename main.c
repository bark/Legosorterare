

#include <avr/io.h>
#include <util/delay.h>

//#include <avr/interrupt.h>


/*
ISR( INT0_vect){
	uint32_t i;
	for( i=0; i<1000; i++ ) {
	     asm volatile ("nop");
	}

	PORTC ^= (1 << PORTC5);	
	
}

int main(void){
//	uint32_t i;
	DDRD |= (1 << PORTD5);
//	PORTD |= (1 << PORTD5);
//	PORTD |= (1 << PORTD2);    //aktivera pullup inport 
//	EICRA |= (1 << ISC01) | (1 << ISC00);
//	EIMSK |= (1 << INT0);
//	sei();
	// timmer
	TCCR0A |=  (1 << COM0B1) | (1 << WGM01)| (1 << WGM00 );
	TCCR0B |= (1<< CS00)| (1 << WGM02);
	OCR0A = 96;//
	OCR0B = 0;//puls bredd
	//
	
	for(;;) {
		if(PINC &(1<<PORTC4)){
			PORTC |= (1 << PORTC5);
		}else{
			PORTC &= ~(1 << PORTC5);
		}	
		
		//for( i=0; i<1000000; i++ ) {
		//	asm volatile ("nop");
		//}
	}
	return 0;
}
/*
//-----------------------------------------------------------------------------
//  ATmega168
//  Internal 8 MH RC Osc.
//  (1/8M = 125 ns)
//-----------------------------------------------------------------------------

#include <avr/io.h>

#define BAUD    9600
#define UBRR    F_CPU/16/BAUD-1

//-----------------------------------------------------------------------------
//  Initialize USART0
//-----------------------------------------------------------------------------
voiad init_USART0(unsigned int baud)
{
    UBRR0 = baud;                       // Set Baudrate
    UCSR0C = (3<<UCSZ00);               // Character Size 8 bit
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);  // Receiver and Transmitter Enable
}

//-----------------------------------------------------------------------------
//  Set Receive Interrupt Enable
//-----------------------------------------------------------------------------
void setRXCIE_USART0()
{
    UCSR0B |= _BV(RXCIE0);
}

//-----------------------------------------------------------------------------
//  Receive 1 byte Data
//-----------------------------------------------------------------------------
unsigned char receive_1byte_USART0(void)
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

//-----------------------------------------------------------------------------
//  Transmit 1 byte Data
//-----------------------------------------------------------------------------
void transmit_1byte_USART0(unsigned char data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}

//-----------------------------------------------------------------------------
//  Transmit String Data
//-----------------------------------------------------------------------------
void transmit_str_USART0(char *str)
{
    while (*str != 0) {
        transmit_1byte_USART0(*str);
        *str++;
    }
}

//-----------------------------------------------------------------------------
//  Transmit Four-Digit Integer
//-----------------------------------------------------------------------------
void transmit_4digit_USART0(int num)
{
    unsigned char temp;
    int digit = 1000;

    while (digit != 0) {
        temp = num / digit;
        transmit_1byte_USART0('0'+temp);
        num -= (digit*temp);
        digit /= 10;
    }
}
*/
//-----------------------------------------------------------------------------
//  Main
//-----------------------------------------------------------------------------
#include <avr/io.h> 
//#include <avr/interrupt.h>

#define F_CPU 16000000	// 16 MHz oscillator.
#define BaudRate 9600
#define MYUBRR (F_CPU / 16 / BaudRate ) - 1 

 



void delayLong()
{
	unsigned int delayvar;
	delayvar = 0; 
	while (delayvar <=  65500U)		
	{ 
		asm("nop");  
		delayvar++;
	} 
}


unsigned char serialCheckRxComplete(void)
{
	return( UCSR0A & _BV(RXC0)) ;		// nonzero if serial data is available to read.
}

unsigned char serialCheckTxReady(void)
{
	return( UCSR0A & _BV(UDRE0) ) ;		// nonzero if transmit register is ready to receive new data.
}

unsigned char serialRead(void)
{
	while (serialCheckRxComplete() == 0)		// While data is NOT available to read
	{;;} 
	return UDR0;
}

void serialWrite(unsigned char DataOut)
{
	while (serialCheckTxReady() == 0)		// while NOT ready to transmit 
	{;;} 
	UDR0 = DataOut;
}



void establishContact() {
	while (serialCheckRxComplete() == 0) { 
		serialWrite('A');
	//	serialWrite(65U);
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
	}
}

  

int main (void)
{ 
	
//Interrupts are not needed in this program. You can optionally disable interrupts.	
//asm("cli");		// DISABLE global interrupts.

		
	DDRD = _BV(1);
	DDRB = _BV(0) | _BV(1) | _BV(3) | _BV(5);
	 
	//Serial Initialization
	
 	/*Set baud rate */ 
	UBRR0H = (unsigned char)(MYUBRR>>8); 
	UBRR0L = (unsigned char) MYUBRR; 
	/* Enable receiver and transmitter   */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
	/* Frame format: 8data, No parity, 1stop bit */ 
	UCSR0C = (3<<UCSZ00);  
	
	 
	int firstSensor = 0;    // first analog sensor
	int secondSensor = 0;   // second analog sensor
	int thirdSensor = 0;    // digital sensor
	int inByte = 0;         // incoming serial byte
	
	
	PORTB |= _BV(1); // Turn on LED @ PB1
	

establishContact();  // send a byte to establish contact until Processing responds 

		PORTB &= 253U; // Turn off LED
	

for (;;)  // main loop										
{

	
	if (serialCheckRxComplete()) {
		PORTB |= _BV(1); // Turn on LED @ PB1
		
		inByte = serialRead();
		
		// Simulated data!
		firstSensor++;
		
		secondSensor = firstSensor * firstSensor;
		
		thirdSensor = firstSensor + secondSensor;
		
		serialWrite(firstSensor & 255U);
		serialWrite(secondSensor & 255U);
		serialWrite(thirdSensor & 255U);

		PORTB &= 253U; // Turn off LED
              
	}
	
	
	 	
	

}	//End main loop.
	return 0;
}
/* massor av fungerade kod.


    DDRC |= (1 << PORTC5);//kör syd
   // PORTC |= (1 << PORTC5);  //aktivera utport
    DDRC |= (1 << PORTC4);//stilla
    DDRC |= (1 << PORTC3); // kör nord


    DDRD |= (1 << PORTD2);//upp kran
    DDRD |= (1 << PORTD3);   //ner kran

//PORTC1 är av någon anledning trasig
    PORTD |= (1 << PORTD2);
    PORTD |= (1 << PORTD3);

    PORTC |= (1<<PORTC0);// aktiverar pullup på nordsidan
    PORTC |= (1<<PORTC2);// aktiverar pullup på sydsidan


    //init_USART0(UBRR);  // initialize USART0
PORTC |= (1 << PORTC3);
    while (1) {
	if((!(PINC &(1<<PORTC2))) && PINC &(1<<PORTC5)){// if sats på port c4
			
		PORTC &= ~(1 << PORTC5);

		PORTC |= (1 << PORTC4);
		PORTD |= (1 << PORTD2);
		_delay_ms( 1000 );
		PORTD &= ~(1 << PORTD2);
		_delay_ms( 1000 );		
		PORTD |= (1 << PORTD3);
		_delay_ms( 1000 );
		PORTD &= ~(1 << PORTD3);
		PORTC &= ~(1 << PORTC4);
		PORTC |= (1 << PORTC3);
		
		
	}
		
	if((!(PINC &(1<<PORTC0)) && PINC &(1<<PORTC3)) ){
			
		PORTC &= ~(1 << PORTC3);
		PORTC |= (1 << PORTC4);
		PORTD |= (1 << PORTD2);
		_delay_ms( 1000 );
		PORTD &= ~(1 << PORTD2);
		_delay_ms( 500 );		
		PORTD |= (1 << PORTD3);
		_delay_ms( 1000 );
		PORTD &= ~(1 << PORTD3);
		PORTC &= ~(1 << PORTC4);
				
		PORTC |= (1 << PORTC5);		
		PORTC &= ~(1 << PORTC4);
	}
		
	
		
	//_delay_ms( 500 );   
	//_delay_ms( 500 ); 
    }

    return 0;
}*/
