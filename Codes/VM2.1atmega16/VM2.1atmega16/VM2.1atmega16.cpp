/*
 * VM2.1atmega16.cpp
 *
 * Created: 2/3/2016 12:11:58 PM
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

#define MrLCDsControl		  PORTA
#define DataDir_MrLCDsControl DDRA
#define MrLCDsCrib			  PORTB
#define DataDir_MrLCDsCrib	  DDRB
#define BiPolarMood    7
#define ReadWrite      6
#define LightSwitch    5

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

void initialise_GPIO_services(void);
void debounce(void);

void initialize_counter(void);
void stop_counter(void);

void initialize_PWM_mode(void);

void dispatch_motor2(void);
void start_PWM_motor2(void);
void stop_PWM_motor2(void);
void disable_driver2(void);

void dispatch_motor3(void);
void start_PWM_motor3(void);
void stop_PWM_motor3(void);
void disable_driver3(void);

void select_product(void);
void dispatch_product(uint8_t product_no);
void check_for_dispatch_completion(void);
void stop_dispatch(void);
void dispatch_result(void);
void publish_results(void);
void allow_refund_policy(void);

void stock_checklist(void);

void check_for_timeout(void);

void accept_COIN(void);
void return_COIN(void);

void initialize_counter(void);
void stop_counter(void);

void reset(void);
void initialize(void);

void UART_enable(void);
void RecieveUART(void);
void TransmitUART(uint8_t data1);

void Check_IF_MrLCD_isBusy(void);
void Peek_A_Boo(void);
void Send_A_Command(char command);
void Send_A_Character(char character);
void Send_A_String(char *stringOfCharacters);
void initialise_LCD_interface(void);
void clear_LCD_display(void);
void LCDs_top_line(void);
void LCDs_bottom_line(void);

void hang_on_please_dispatching(void);

volatile unsigned int count = 0;
volatile unsigned int count1 = 0;
volatile uint8_t mini_FAIL = 0;
volatile uint8_t FAIL = 0;

uint8_t pressed = 0;
uint8_t product = 0;
uint8_t dataRecieved = 0;
uint8_t product_list = 0;
uint8_t dispatch_successful = 0;
uint8_t product_1_EMPTY = 0;
uint8_t product_2_EMPTY = 0;
uint8_t product_3_EMPTY = 0;
uint8_t allow_refund = 0;

int main(void)
{		
	initialise_GPIO_services();
	initialise_LCD_interface();
	initialize_PWM_mode();
	UART_enable();
	
	sei();
	
	while(1)
	{
		allow_refund = 0;			
		stock_checklist();
		initialize();
		
		while(2)
		{
			RecieveUART();
			
			RecieveUART();
			
			if (dataRecieved == coin_accepted)
			{
				dataRecieved = 0;
				
				_delay_us(5);
				clear_LCD_display();
				LCDs_top_line();
				Send_A_String("COIN ACCEPTED");
				
				_delay_us(5);
				
				LCDs_bottom_line();
				Send_A_String(" SELECT PRODUCT");
				_delay_ms(50);
				
				select_product();
				check_for_dispatch_completion();
				publish_results();
				
				break;
			}
			reset();			
		}
	}			
		
}
////////////////////
void initialise_GPIO_services(void)
{
	DDRA = 0b11100000;
	DDRC = 0b00010000;
	DDRD = 0b11111100;
	
	PORTA = 0b00011111;
	PORTC = 0b11101111;
	PORTD = 0b00000000;
}
void debounce(void)
{
	unsigned int confidence_level=0;
	confidence_level++;
	if (confidence_level>=500)
	{
		confidence_level=0;
	}
}
////////////////////
void initialize_PWM_mode(void)
{
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	ICR1 = 19999; //top value
	TCNT1 = 0;
}
///////////////////
void dispatch_motor2(void)
{
	_delay_us(50);
	PORTD |= (1<<PIND2);
	PORTD &=~ (1<<PIND3);
	_delay_us(50);
}
void start_PWM_motor2(void)
{
	TCCR1A |= (1<<COM1B1);
	OCR1B = 5000;  //calibrate 
}
void stop_PWM_motor2(void)
{
	TCCR1A &=~ (1<<COM1B1);
	OCR1B = 0;
}
void disable_driver2(void)
{
	_delay_us(50);
	PORTD &=~ (1<<PIND2);
	PORTD &=~ (1<<PIND3);
	PORTD &=~ (1<<PIND4);
	_delay_us(50);
}
////////////////////
void dispatch_motor3(void)
{
	_delay_us(50);
	PORTD |= (1<<PIND7);
	PORTD &=~ (1<<PIND6);
	_delay_us(50);
}
void start_PWM_motor3(void)
{
	TCCR1A |= (1<<COM1A1);
	OCR1A = 5000; //calibrate
}
void stop_PWM_motor3(void)
{
	TCCR1A &=~ (1<<COM1A1);
	OCR1A = 0;
}
void disable_driver3(void)
{
	PORTD &=~ (1<<PIND6);
	PORTD &=~ (1<<PIND7);
	PORTD &=~ (1<<PIND5);
}
///////////////////
void stock_checklist(void)
{
	PORTC &= ~(1<<PINC4);
		
	if ( (bit_is_clear(PINC,5)) && (bit_is_clear(PINC,6)) && (bit_is_clear(PINC,7)) )
	{
		product_1_EMPTY = 1;
		product_2_EMPTY = 1;
		product_3_EMPTY = 1;
	}
	else
	if ( (bit_is_clear(PINC,5)) && (bit_is_clear(PINC,6)) && (bit_is_set(PINC,7)) )
	{
		product_1_EMPTY = 1;
		product_2_EMPTY = 1;
		product_3_EMPTY = 0;
	}
	else
	if ( (bit_is_set(PINC,5)) && (bit_is_clear(PINC,6)) && (bit_is_clear(PINC,7)) )
	{
		product_1_EMPTY = 0;
		product_2_EMPTY = 1;
		product_3_EMPTY = 1;
	}
	else
	if ( (bit_is_clear(PINC,5)) && (bit_is_set(PINC,6)) && (bit_is_clear(PINC,7)) )
	{
		product_1_EMPTY = 1;
		product_2_EMPTY = 0;
		product_3_EMPTY = 1;
	}
	else
	if ( (bit_is_clear(PINC,5)) && (bit_is_set(PINC,6)) && (bit_is_set(PINC,7)) )
	{
		product_1_EMPTY = 1;
		product_2_EMPTY = 0;
		product_3_EMPTY = 0;
	}
	else
	if ( (bit_is_set(PINC,5)) && (bit_is_clear(PINC,6)) && (bit_is_set(PINC,7)) )
	{
		product_1_EMPTY = 0;
		product_2_EMPTY = 1;
		product_3_EMPTY = 0;
	}
	else
	if ( (bit_is_set(PINC,5)) && (bit_is_set(PINC,6)) && (bit_is_clear(PINC,7)) )
	{
		product_1_EMPTY = 0;
		product_2_EMPTY = 0;
		product_3_EMPTY = 1;
	}
	else
	if ( (bit_is_set(PINC,5)) && (bit_is_set(PINC,6)) && (bit_is_set(PINC,7)) )
	{
		product_1_EMPTY = 0;
		product_2_EMPTY = 0;
		product_3_EMPTY = 0;
	}
}
void allow_refund_policy(void)
{
	if (allow_refund == 0)
	{
		allow_refund = 1;
	}
	
	_delay_us(50);
	clear_LCD_display();
	LCDs_top_line();
	Send_A_String(" SORRY BRO !!");
	
	_delay_us(5);
	
	LCDs_bottom_line();
	Send_A_String("OUT OF STOCK");
	_delay_ms(50);
	
	_delay_ms(1200);
	
	_delay_us(50);
	clear_LCD_display();
	LCDs_top_line();
	Send_A_String("push RETURN COIN");
	
	_delay_us(5);
	
	LCDs_bottom_line();
	Send_A_String("OR BUY ANOTHER 1");
	_delay_ms(50);
	
	PORTC |= (1<<PINC4);
	
	select_product();
}
///////////////////
void select_product(void)
{
	while(8)
	{
		if( (bit_is_set(PINC,0)) && (bit_is_set(PINC,1) && (bit_is_set(PINC,2)) && (bit_is_set(PINC,3))) )
		{
			pressed = 0;
			break;
		}
	}
	
	while(3)
	{
		if (bit_is_clear(PINC,0))
		{
			debounce();
			if (pressed == 0)
			{
				pressed = 1;
				
				if (product_1_EMPTY == 0)
				{
					dispatch_product(1);  //dispatch product 1
				}
				else
				if (product_1_EMPTY == 1)
				{
					if (allow_refund == 0)
					{
						allow_refund_policy();
					}
					else
					if (allow_refund == 1)
					{
						dispatch_product(4); //return coin
					}
				}
				
				break;
			}
		}
		else
		if (bit_is_clear(PINC,1))
		{
			debounce();
			if (pressed == 0)
			{
				pressed = 1;
				
				if (product_2_EMPTY == 0)
				{
					dispatch_product(2);
				}
				else
				if (product_2_EMPTY == 1)
				{
					if (allow_refund == 0)
					{
						allow_refund_policy();
					}
					else
					if (allow_refund == 1)
					{
						dispatch_product(4);  //return coin
					}
				}
				
				break;
			}
		}
		else
		if (bit_is_clear(PINC,2))
		{
			debounce();
			if (pressed == 0)
			{
				pressed = 1;
				
				if (product_3_EMPTY == 0)
				{
					dispatch_product(3);
				}
				else
				if (product_3_EMPTY == 1)
				{
					if (allow_refund == 0)
					{
						allow_refund_policy();
					}
					else
					if (allow_refund == 1)
					{
						dispatch_product(4);
					}
				}
				
				break;
			}
		}
		else
		if (bit_is_clear(PINC,3))//RETURN COIN
		{
			debounce();
			if (pressed == 0)
			{
				pressed = 1;
				
				if (allow_refund == 1)
				{
					dispatch_product(4);//return coin
				}
				else
				if (allow_refund == 0)
				{
					select_product();
				}
				
				break;
			}
		}
	}
}
///////////////////
void dispatch_product(uint8_t product_no)
{	
	if (product_no == 1)
	{
		hang_on_please_dispatching();
		
		TransmitUART(product_1);//dispatch product 1
		_delay_us(50);
		TransmitUART(product_1);//dispatch product 1
		product = 1;
	}
	else
	if (product_no == 2)
	{
		hang_on_please_dispatching();
		
		TransmitUART(product_2);//dispatch product 2
		_delay_us(50);
		TransmitUART(product_2);
		
		dispatch_motor2();
		start_PWM_motor2();
		product = 2;
	}
	else
	if (product_no == 3)
	{
		hang_on_please_dispatching();
		
		TransmitUART(product_3);//dispatch product 3
		_delay_us(50);
		TransmitUART(product_3);
		
		dispatch_motor3();
		start_PWM_motor3();
		product = 3;
	}
	else
	if (product_no == 4)
	{
		_delay_us(50);
		clear_LCD_display();
		LCDs_top_line();
		Send_A_String(" NOW THE COIN");
		
		_delay_us(5);
		
		LCDs_bottom_line();
		Send_A_String("WILL BE RETURNED");
		_delay_ms(50);
		
		TransmitUART(product_4);
		_delay_us(50);
		TransmitUART(product_4);
		product = 4;	
	}
}
void check_for_dispatch_completion(void)
{
	if (product == 1)
	{
		while(4)
		{			
			if (bit_is_clear(PINA,0))
			{
				dispatch_successful = 1;
				
				stop_dispatch();
				break;
			}						
		}
	}
	else
	if (product == 2)
	{
		while(5)
		{
			if (bit_is_clear(PINA,1))
			{
				dispatch_successful = 1;
				
				stop_dispatch();
				break;
			}
		}
	}
	else
	if (product == 3)
	{
		while(6)
		{
			if (bit_is_clear(PINA,2))
			{
				dispatch_successful = 1;
				
				stop_dispatch();
				break;
			}
		}
	}
	else
	if (product == 4)
	{
		dispatch_successful = 0;
		
		stop_dispatch();
	}
}
void stop_dispatch(void)
{
	if (product == 1)
	{
		//transmit UART to microcontroller 1 to stop
		dispatch_result();
	}
	else
	if (product == 2)
	{
		stop_PWM_motor2();
		disable_driver2();
		
		dispatch_result();
	}
	else
	if (product == 3)
	{
		stop_PWM_motor3();
		disable_driver3();
		
		dispatch_result();
	}
	else
	if (product == 4)
	{
		dispatch_result();
	}
}
void dispatch_result(void)
{
	if (dispatch_successful == 1)
	{		
		TransmitUART(0x69);
		_delay_us(5);
		TransmitUART(accept_coin);
		
		_delay_us(50);
		clear_LCD_display();
		LCDs_top_line();
		Send_A_String("THANK YOU FOR");
		
		_delay_us(5);
		
		LCDs_bottom_line();
		Send_A_String("YOUR SUPPORT");
		_delay_ms(50);
	}
	else
	if (dispatch_successful == 0)
	{
		TransmitUART(0x69);
		_delay_us(5);
		TransmitUART(return_coin);
		
		_delay_us(50);
		clear_LCD_display();
		LCDs_top_line();
		Send_A_String("ERROR 404...");
		
		_delay_us(5);
		
		LCDs_bottom_line();
		Send_A_String("PLEASE TRY AGAIN");
		_delay_ms(50);		
	}
}
void publish_results(void)
{
	while(8)
	{
		RecieveUART();
		
		if (dataRecieved == TY_for_shopping)
		{
			_delay_us(50);
			clear_LCD_display();
			LCDs_top_line();
			Send_A_String("THANK YOU...HOPE");
			
			_delay_us(5);
			
			LCDs_bottom_line();
			Send_A_String("TO SEE YOU SOON");
			_delay_ms(50);
			
			break;
		}
		else
		if (dataRecieved == Pl_try_again)
		{			
			_delay_us(50);
			clear_LCD_display();
			LCDs_top_line();
			Send_A_String("ERROR-404 PLEASE");
			
			_delay_us(5);
			
			LCDs_bottom_line();
			Send_A_String("   TRY AGAIN   ");
			_delay_ms(50);
			
			break;
		}
	}
}
//////////////////
void check_for_timeout(void)
{
	
}
//////////////////
void accept_COIN(void)
{
	TransmitUART(accept_coin);//transmit UART to accept coin
}
void return_COIN(void)
{
	TransmitUART(return_coin);//transmit UART to return coin
}
//////////////////
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
/////////////////
void reset(void)
{
	allow_refund = 0;
	pressed = 0;
	product = 0;
	dataRecieved = 0;
	product_list = 0;
	dispatch_successful = 0;
	product_1_EMPTY = 0;
	product_2_EMPTY = 0;
	product_3_EMPTY = 0;	
	
	count = 0;
	count1 = 0;
	mini_FAIL = 0;
	FAIL = 0;
	
	PORTA = 0b00011111;
	PORTC = 0b11101111;
	PORTD = 0b00000000;	
}
/////////////////
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
	UCSRB &= ~(1 << TXCIE);
	UCSRB &= ~(1 << RXCIE);
	UCSRB &= ~(1 << UDRIE);
	//UCSRB |= (1 << TXCIE) | (1 << RXCIE) | (1 << UDRIE);	
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
/////////////////
void Check_IF_MrLCD_isBusy(void)
{
	DataDir_MrLCDsCrib=0;
	MrLCDsControl |= 1<<ReadWrite;
	MrLCDsControl &=~ 1<<BiPolarMood;
	while(MrLCDsCrib >= 0x80)
	{
		Peek_A_Boo();
	}
	
	DataDir_MrLCDsCrib=0xFF;
}
void Peek_A_Boo(void)
{
	MrLCDsControl |= 1<<LightSwitch;
	asm volatile ("nop");
	asm volatile ("nop");
	MrLCDsControl &=~ 1<<LightSwitch;
}
void Send_A_Command(char command)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = command;
	MrLCDsControl &=~ (1<<ReadWrite|1<<BiPolarMood);
	Peek_A_Boo();
	MrLCDsCrib = 0;
}
void Send_A_Character(char character)
{
	Check_IF_MrLCD_isBusy();
	MrLCDsCrib = character;
	MrLCDsControl &=~ (1<<ReadWrite);
	MrLCDsControl |= (1<<BiPolarMood);
	Peek_A_Boo();
	MrLCDsCrib = 0;
}
void Send_A_String(char *stringOfCharacters)
{
	while(*stringOfCharacters > 0)
	{
		Send_A_Character(*stringOfCharacters++);
		_delay_ms(20);
	}
}
void initialise_LCD_interface(void)
{
	DataDir_MrLCDsControl |= 1<<LightSwitch | 1<<ReadWrite | 1<<BiPolarMood;
	_delay_ms(15);
	clear_LCD_display();
	Send_A_Command(0x38);
	_delay_us(50);
	Send_A_Command(0b00001100);
	_delay_us(50);
}
void clear_LCD_display(void)
{
	Send_A_Command(0x01);//clrscr();
	_delay_ms(2);
}
void LCDs_top_line(void)
{
	Send_A_Command(0X80);
	_delay_us(50);
}
void LCDs_bottom_line(void)
{	
	Send_A_Command(0XC0);
	_delay_us(50);
}
//////////////////
void hang_on_please_dispatching(void)
{
	_delay_us(5);
	clear_LCD_display();
	LCDs_top_line();
	Send_A_String("hang on please..");
	
	_delay_us(5);
	
	LCDs_bottom_line();
	Send_A_String("dispatching....");
	_delay_ms(50);
}
void initialize(void)
{
	_delay_us(5);
	clear_LCD_display();
	LCDs_top_line();
	Send_A_String(" AUSTIN MACHINES ");
	
	_delay_us(5);
	
	LCDs_bottom_line();
	Send_A_String("TECHNOLOGY AHEAD");
	
	_delay_ms(1200);
	
	clear_LCD_display();
	LCDs_top_line();
	Send_A_String("Insert 10 RUPEES");
	
	_delay_us(5);
	LCDs_bottom_line();
	Send_A_String("coin to purchase");
	_delay_us(5);
}
//////////////////
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









