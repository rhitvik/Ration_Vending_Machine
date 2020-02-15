/*
 * VM2.1atmega8MASTER.cpp
 *
 * Created: 07-Feb-16 6:56:55 PM
 *  Author: RHITVIK
 */ 

#define F_CPU 1000000UL

#define coin_accepted    0xCA
#define coin_rejected    0xAC
#define product_1        0x11
#define product_2        0x22
#define product_3        0x33
#define product_4        0x44    /////////return coin///////////
#define accept_coin      0xCC
#define return_coin      0xDD
#define austin_machines  0xAA
#define TY_for_shopping  0xDE
#define Pl_try_again     0xED

#define P123    0XD1
#define P12     0XD2
#define P23     0XD3
#define P31     0XD4
#define P1      0XD5
#define P2      0XD6
#define P3      0XD7
#define P_ALL   0XD8

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
////////////////
void initialise_GPIO_services(void);
////////////////
void UART_enable(void);
void RecieveUART(void);
void TransmitUART(uint8_t data);
///////////////
void initialize_PWM_mode(void);
///////////////
void initialise_COIN_monitoring_mechanism(void);
void Check_if_something_inserted(void);
void check_METAL(void);
void is_COIN_received(void);
void Check_Authenticity(void);
///////////////
void ACCEPT_COIN(void);
void REJECT_COIN(void);
void _coin_delay(void);
void accept_mode(void);
void reject_mode(void);
void start_PWM_COIN(void);
void stop_PWM_COIN(void);
void disable_driver_COIN(void);
///////////////
void check_stock(void);
///////////////
void dispatch_motor1(void);
void start_PWM_motor1(void);
void stop_PWM_motor1(void);
void disable_driver1(void);
///////////////
void initialize_counter(void);
void stop_counter(void);
///////////////
void receive_product_ID(void);
void receive_final_data(void);
void reset_parameters(void);
///////////////
uint8_t dataRecieved = 0;
uint8_t product = 0;

volatile unsigned int count = 0;
volatile unsigned int count1 = 0;
volatile uint8_t mini_FAIL = 0;
volatile uint8_t FAIL = 0;

uint8_t something_is_inserted = 0;
uint8_t METAL = 0;
uint8_t GOT_COIN = 0;
volatile uint8_t COIN_is_Authentic = 0;

int main(void)
{	
	initialise_GPIO_services();
	UART_enable();
	initialize_PWM_mode();
	//initialize_counter();//to be placed when the dispatch starts!!
	
	sei();
	
	_delay_ms(100);
	
	while(1)
	{
		//TransmitUART(0xC3);		
				
		initialise_COIN_monitoring_mechanism();
		
		receive_product_ID();
		
		receive_final_data();
		
		reset_parameters();	
	}
}

////////////////
void initialise_GPIO_services(void)
{
	DDRB = 0b11111111;
	PORTB= 0b00000000;
	
	DDRC = 0b111000;
	PORTC= 0b000111;
	
	DDRD = 0b11111100;
	PORTD= 0b00000000;
}
////////////////
void UART_enable(void)
{
	int baud = 9600;	
	UCSRA &= ~(1 << U2X);	
	uint16_t UBRRValue = lrint((F_CPU /(16L * baud)) - 1);	
	//Put the upper part of the baud number here (bits 8 to 11)
	UBRRH = (unsigned char) (UBRRValue >> 8);
	//Put the remaining part of the baud number here
	UBRRL = (unsigned char) UBRRValue;
	//Enable the receiver and transmitter
	UCSRB |= (1 << RXEN) | (1 << TXEN);
//	UCSRB &= ~(1 << TXCIE);
//	UCSRB &= ~(1 << RXCIE);
//	UCSRB &= ~(1 << UDRIE);
//	UCSRB |= (1 << TXCIE) | (1 << RXCIE) | (1<<UDRIE);	
	UCSRC |= (1 << USBS); //Sets 2 stop bits
	UCSRC |= (1 << UPM1); //Sets parity to EVEN
	UCSRC |= (3 << UCSZ0); //Alternative code for 8-bit data length
}
void RecieveUART(void)
{
	while (! (UCSRA & (1 << RXC)) );
	dataRecieved = UDR;
}
void TransmitUART(uint8_t data)
{
	while (! (UCSRA & (1 << UDRE)) );
	UDR = data;
}
////////////////
void initialize_PWM_mode(void)
{
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	ICR1 = 19999; //top value
	TCNT1 = 0;
}
////////////////
void initialise_COIN_monitoring_mechanism(void)
{
	PORTD |= (1<<PIND2);
	Check_if_something_inserted();
	if (something_is_inserted == 1)
	{
		check_METAL();
		
		if (METAL == 1)
		{			
			is_COIN_received();
			
			if (GOT_COIN == 1)
			{
				Check_Authenticity();
				
				if (COIN_is_Authentic == 0)
				{
					TransmitUART(coin_rejected);
					_delay_us(5);
					TransmitUART(coin_rejected);
				}				
				else
				if (COIN_is_Authentic == 1)
				{
					TransmitUART(coin_accepted);
					_delay_us(5);
					TransmitUART(coin_accepted);
				}			
			}
		}
	}
}
void Check_if_something_inserted(void)
{
	while(2)
	{
		if (bit_is_clear(PINC,0))
		{
			PORTC |= (1<<PINC3);
			something_is_inserted = 1;
			break;
		}
		else 
		if (bit_is_clear(PINC,2))
		{
			PORTD = 0b11111100;///////
			_delay_ms(2000);///////
			PORTD = 0;///////
			GOT_COIN = 1;
			if ( (something_is_inserted == 0)||(METAL == 0) )
			{
				REJECT_COIN();
				reset_parameters();
				initialise_COIN_monitoring_mechanism(); 
			}
		}
	}
}
void check_METAL(void)
{
	while(3)
	{
		if (bit_is_clear(PINC,1))
		{
			PORTC |= (1<<PINC4);
			METAL = 1;
			break;
		}
		else 
		if (bit_is_clear(PINC,2))
		{
			PORTD = 0b11111100;//////
			_delay_ms(2000);//////
			PORTD = 0;//////
			GOT_COIN = 1;
			if ( (something_is_inserted == 0)||(METAL == 0) )
			{
				REJECT_COIN();
				reset_parameters();
				initialise_COIN_monitoring_mechanism(); 
			}
		}		
	}
}
void is_COIN_received(void)
{
	while(4)
	{
		if (bit_is_clear(PINC,2))
		{
			PORTC |= (1<<PINC5);
			GOT_COIN = 1;			
			break;
		}
	}
}
void Check_Authenticity(void)
{
	if ( (METAL == 1) && (GOT_COIN == 1) && (something_is_inserted == 1) )
	{
		COIN_is_Authentic = 1;		
	}
	else
	if ( (METAL == 0) || (GOT_COIN == 1) || (something_is_inserted == 0) )
	{
		COIN_is_Authentic = 0;
		REJECT_COIN();
		reset_parameters();
		initialise_COIN_monitoring_mechanism();
	}
}
/////////////////
void ACCEPT_COIN(void)
{
	accept_mode();
	start_PWM_COIN();
	_coin_delay();
	stop_PWM_COIN();
	disable_driver_COIN();
}
void REJECT_COIN(void)
{
	reject_mode();
	start_PWM_COIN();
	_coin_delay();
	stop_PWM_COIN();
	disable_driver_COIN();
}
void _coin_delay(void)
{
	_delay_ms(3000);
}
void accept_mode(void)
{
	_delay_us(50);
	PORTB |= (1<<PINB4);
	PORTB &=~ (1<<PINB5);
	_delay_us(50);
}
void reject_mode(void)
{
	_delay_us(50);
	PORTB |= (1<<PINB5);
	PORTB &=~ (1<<PINB4);
	_delay_us(50);
}
void start_PWM_COIN(void)
{
	TCCR1A |= (1<<COM1B1);
	OCR1B = 5000;  //calibrate
}
void stop_PWM_COIN(void)
{
	TCCR1A &= ~(1<<COM1B1);
	OCR1B = 0;
}
void disable_driver_COIN(void)
{
	_delay_us(50);
	PORTB &=~ (1<<PINB2);
	PORTB &=~ (1<<PINB4);
	PORTB &=~ (1<<PINB5);
	_delay_us(50);
}
////////////////
void dispatch_motor1(void)
{
	_delay_us(50);
	PORTB |= (1<<PINB0);
	PORTB &=~ (1<<PINB3);
	_delay_us(50);
}
void start_PWM_motor1(void)
{
	TCCR1A |= (1<<COM1A1);
	OCR1A = 5000; //calibrate
}
void stop_PWM_motor1(void)
{
	TCCR1A &=~ (1<<COM1A1);
	OCR1A = 0;
}
void disable_driver1(void)
{
	PORTB &=~ (1<<PINB1);
	PORTB &=~ (1<<PINB0);
	PORTB &=~ (1<<PINB3);
}
////////////////
void initialize_counter(void)
{
	TCCR0 |= (1<<CS01)|(1<<CS00);
	TIMSK |= (1<<TOIE0);
	TCNT0 = 0;
}
void stop_counter(void)
{
	TCCR0 &=~ (1<<CS00);
	TCCR0 &=~ (1<<CS01);
	TIMSK &=~ (1<<TOIE0);
	_delay_ms(1);
	TCNT0=0;
}
////////////////
void receive_product_ID(void)
{
	while(5)
	{
		RecieveUART();
		
		if (dataRecieved == product_1)
		{
			product = 1;
			
			dispatch_motor1();
			start_PWM_motor1();
			
			break;
		}
		else
		if (dataRecieved == product_2)
		{
			product = 2;
			
			break;
		}
		else
		if (dataRecieved == product_3)
		{
			product = 3;
			
			break;
		}
		else
		if (dataRecieved == product_4)
		{
			product = 4;
			
			break;
		}
	}	
}
void receive_final_data(void)
{	
	while(6)
	{
		RecieveUART();
		
		PORTD = 0;
		
		if (dataRecieved == accept_coin)
		{
			if (product == 1)
			{
				stop_PWM_motor1();
				disable_driver1();
				_delay_ms(50);
				
				TransmitUART(TY_for_shopping);
				
				ACCEPT_COIN();
				
				break;
			}
			else
			if ( (product == 2) || (product == 3) )
			{
				TransmitUART(TY_for_shopping);
				
				ACCEPT_COIN();
				
				break;
			}
			else
			if (product == 4)
			{
				TransmitUART(Pl_try_again);
				
				REJECT_COIN();
				
				break;
			}
		}
		else
		if (dataRecieved == return_coin)
		{
			if (product == 1)
			{
				stop_PWM_motor1();
				disable_driver1();
				_delay_ms(50);
				
				TransmitUART(Pl_try_again);
				
				REJECT_COIN();
				
				break;
			}
			else
			if ( (product == 2) || (product == 3) )
			{
				TransmitUART(Pl_try_again);
				
				REJECT_COIN();
				
				break;
			}
			else
			if (product == 4)
			{
				TransmitUART(Pl_try_again);
				
				REJECT_COIN();
				
				break;
			}
		}
	}
}
void reset_parameters(void)
{
	uint8_t dataRecieved = 0;
	uint8_t product = 0;

	volatile unsigned int count = 0;
	volatile unsigned int count1 = 0;
	volatile uint8_t mini_FAIL = 0;
	volatile uint8_t FAIL = 0;

	uint8_t something_is_inserted = 0;
	uint8_t METAL = 0;
	uint8_t GOT_COIN = 0;
	volatile uint8_t COIN_is_Authentic = 0;
	
 	PORTD= 0b00000000;
 	PORTC= 0b000111;
 	PORTB= 0;
 	
	_delay_ms(50);
}
///////////////
ISR(TIMER0_OVF_vect)
{
	count++;
	if (count==61)
	{
		// 1 second has elapsed		
		count=0;
		count1++;
		if (count1==10)
		{
			mini_FAIL=1;
		}
		
		else
		if (count1==20)/////// calibrate it further
		{
			//return coin if not dispatched
			count1=0;
			FAIL=1;
		}
	}
}

/*
if (COIN_is_Authentic == 0)
{
	TransmitUART(austin_machines);
}
else
if (COIN_is_Authentic == 1)
{
	TransmitUART(coin_accepted);
	//COIN_is_Authentic = 2;
}
else

if (COIN_is_Authentic == 0)
{
	TransmitUART(austin_machines);
}
else
if (COIN_is_Authentic == 1)
{
	TransmitUART(coin_accepted);
	//COIN_is_Authentic = 2;
}

ISR(USART_UDRE_vect)
{
	UDR = 0xAA;
	_delay_ms(50);
}

ISR(USART_RXC_vect)
{
	PORTD ^= (1<<PIND2);
	dataR = UDR;
	
	if (dataR == 0xAA)
	{
		PORTD ^= (1<<PIND3);	
	}	
	else
	if (dataR == 0xBB)
	{
		PORTD ^= (1<<PIND4);
	}
	else
	if (dataR == 0xCC)
	{
		PORTD ^= (1<<PIND5);
	}
	else
	if (dataR == 0xDD)
	{
		PORTD ^= (1<<PIND6);
	}
}



TransmitUART(0xFE);
_delay_ms(3000);
while (! (UCSRA & (1 << RXC)) );
dataR = UDR;

if (dataR == 0xFE)
{
	PORTD ^= (1<<PIND2);
}

else
if (dataR == 0xFF)
{
	PORTD ^= (1<<PIND4);
}


ISR(USART_UDRE_vect)
{
	UDR = 0xF0;
}


*/
