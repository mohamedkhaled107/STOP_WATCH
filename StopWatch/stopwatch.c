/******************************************************************************
 *  File name: stopwatch.c
 *  Created on: Sep 11, 2023
 *  Author: Mohamed Khaled
 *******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Global variables
unsigned char second=0;
unsigned char minute=0;
unsigned char hour=0;
unsigned char sec_flag=0;

void TIMER1_CTC(unsigned short compare_value){
	TCCR1A = (1<<FOC1A); //Non PWM mode
	TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12); //CTC Mode & prescaler = 1024
	TIMSK=(1<<OCIE1A); //enable CTC module interrupt
	TCNT1=0;      //initial value for counting
	OCR1A=compare_value;
}

// ISR for timer1 compare mode channel A
ISR(TIMER1_COMPA_vect){
	sec_flag=1;

}

void INT0_Init(void){
	DDRD &= ~(1<<PD2); //PD2 ->input pin
	PORTD |=(1<<PD2); // Enable internal pull up resistor
	MCUCR |= (1<<ISC01); // select falling edge trigger
	MCUCR &= ~(1<<ISC00);
	GICR |= (1<<INT0); // Enable external interrupt pin INT0
}

// External INT0 ISR
ISR(INT0_vect){
	second=0;
	minute=0;
	hour=0;
}

void INT1_Init(void){
	DDRD &= ~(1<<PD3); //PD3 ->input pin
	MCUCR |= (1<<ISC10)|(1<<ISC11); // select raising edge trigger
	GICR |=(1<<INT1);	// Enable external interrupt pin INT1
}

// External INT1 ISR
ISR(INT1_vect){
	// Clear the timer clock bits (CS10=0 CS11=0 CS12=0) to stop the timer clock.
	TCCR1B &= 0xF8;
}


void INT2_Init(void){
	DDRB &= ~(1<<PB2); //PB2 ->input pin
	PORTB |=(1<<PB2); // Enable internal pull up resistor
	MCUCSR |=(1<<ISC2); // select falling edge trigger
	GICR |=(1<<INT2); // Enable external interrupt pin INT2
}

// External INT2 ISR
ISR(INT2_vect){
	TCCR1B |= (1<<CS10) | (1<<CS12);
}

int main(void){
	DDRC |=0x0F; // configure first 6 pins in PORTA as output pins
	PORTC &=0xF0; // configure first four pins of PORTC as output pins
	DDRA |=0x3F; // Enable all the 7-Segments 
	PORTA |=0x3F; //initialize all of 7-Segments with zero value
	SREG=(1<<7);	// Enable global interrupts.


	TIMER1_CTC(976);
	INT0_Init();
	INT1_Init();
	INT2_Init();

	while(1){
		if(sec_flag==1){
			second++;
			if(second==60){
				minute++;
				second=0;
			}
			if(minute==60){
				hour++;
				minute=0;
			}
			if(hour==24){
				hour=0;
			}
			//reset the second flag
			sec_flag=0;
		}
		// Display the first digit of seconds
		PORTA = (PORTA & 0xC0) |  (1<<0);
		PORTC = (PORTC & 0xF0) | (second%10);
		//small delay to see the changes in the 7-segment
		_delay_ms(2);

		// Display the second digit of seconds
		PORTA = (PORTA & 0xC0) |  (1<<1);
		PORTC = (PORTC & 0xF0) | (second/10);
		_delay_ms(2);

		// Display the first digit of minutes
		PORTA = (PORTA & 0xC0) |  (1<<2);
		PORTC = (PORTC & 0xF0) | (minute%10);
		_delay_ms(2);

		// Display the second digit of minutes
		PORTA = (PORTA & 0xC0) |  (1<<3);
		PORTC = (PORTC & 0xF0) | (minute/10);
		_delay_ms(2);

		// Display the first digit of hours
		PORTA = (PORTA & 0xC0) |  (1<<4);
		PORTC = (PORTC & 0xF0) | (hour%10);
		_delay_ms(2);

		// Display the second digit of hours
		PORTA = (PORTA & 0xC0) |  (1<<5);
		PORTC = (PORTC & 0xF0) | (hour/10);
		_delay_ms(2);
	}
}
