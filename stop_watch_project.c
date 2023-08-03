/*
 * stop_watch_project.c
 *
 *  Created on: Apr 17, 2023
 *      Author: Mohammed Hossam
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>



#define Seconds_LED		 PB0
#define Minutes_LED		 PB1
#define Hours_LED		 PB3

#define RESUME		 	 PB2


#define RESET		 	 PD2
#define Pause		  	 PD3

#define Time	 PD7

void display();
void Seconds_Selector();
void Minutes_Selector();
void Hours_Selector();



unsigned char sec_units = 0;
unsigned char sec_tenth = 0;
unsigned char min_units = 0;
unsigned char min_tenth = 0;
unsigned char hrs_units = 0;
unsigned char hrs_tenth = 0;


/*	FCPU = 1 MHz	*/

void Timer1_Init(void)
{
	TCNT1 = 0;
	OCR1A = 825;
	TIMSK |= (1<<OCIE1A);

	TCCR1A = (1<<FOC1A);
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);
}

void Ext_INT0_Reset(void)
{
	DDRD &= ~(1<<RESET);				// External Interrupt 0 as input
	PORTD |= (1<<RESET);				// External Interrupt 0 Internal pull-up activated
	MCUCR |= (1<<ISC01);				// Falling Edge activated
	GICR |= (1<<INT0);
}

void Ext_INT1_Pause(void)
{
	DDRD &= ~(1<<Pause);				// External Interrupt 1 as input
	MCUCR |= (1<<ISC11) | (1<<ISC10);	// Rising Edge activated
	GICR |= (1<<INT1);					// External Interrupt Request 1 Enable
}

void Ext_INT2_Resume()
{
	DDRB &= ~(1<<RESUME);				// External Interrupt 2 as input
	PORTB |= (1<<RESUME);				// External Interrupt 2 Internal pull-up activated
	MCUCSR &= ~(1<<ISC2);				// Falling Edge activated
	GICR |= (1<<INT2);					// External Interrupt Request 2 Enable
}

ISR (TIMER1_COMPA_vect)					// Interrupt
{
	if (!(PIND & (1<<Time)))	// initiate seconds count
	{
		++sec_units;
	}

}

ISR (INT0_vect)							// Reset Interrupt resets all numbers to 0
{
	sec_units = sec_tenth = min_units = min_tenth = hrs_units = hrs_tenth = 0;
}

ISR (INT1_vect)							// Pause Interrupt stops the timer
{
	TCCR1B = 0;
}

ISR (INT2_vect)							// Resume Interrupt initializes timer again
{
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);
}










void display ()
{
	PORTA = (1<<0);												// First 7 segment displays
	PORTC = (PORTC & 0xF0) | (sec_units & 0x0F  ) ;				// the seconds units are displayed
	_delay_us(5) ;

	PORTA = (1<<1);
	PORTC = (PORTC & 0xF0) | (sec_tenth & 0x0F  ) ;
	_delay_ms(5) ;

	PORTA = (1<<2);
	PORTC = (PORTC & 0xF0) | (min_units & 0x0F  ) ;
	_delay_ms(5) ;

	PORTA = (1<<3);
	PORTC = (PORTC & 0xF0) | (min_tenth & 0x0F  ) ;
	_delay_ms(5) ;

	PORTA = (1<<4);
	PORTC = (PORTC & 0xF0) | (hrs_units & 0x0F  ) ;
	_delay_ms(5) ;

	PORTA = (1<<5);
	PORTC = (PORTC & 0xF0) | (hrs_tenth & 0x0F  ) ;
	_delay_ms(5) ;



}


void Stop_Watch (void)
{
	while ( !(PIND & (1<<Time)))			//  ( Pin is LOW )
	{
		display();


		if (sec_units == 10)
		{
			++sec_tenth;
			sec_units = 0;
		}
		if (sec_tenth == 6)
		{
			sec_tenth = sec_units = 0;
			++min_units;
		}

		/* Minutes */
		if (min_units == 10)
		{
			min_units=0;
			++min_tenth;
		}
		if (min_tenth == 10)
		{
			min_tenth = min_units = 0;
			++hrs_units;
		}

		/* Hours */
		if (hrs_units == 10)
		{
			hrs_units=0;
			++hrs_tenth;
		}
		if (hrs_tenth == 10)
		{
			hrs_tenth = hrs_units = min_tenth = min_units = sec_tenth = sec_units = 0;
		}
	}
}



int main(void)
{
	DDRC |= 0x0F;					// First four pins of PORTC are output ( for 7447 )
	DDRA |= 0x3F;					// First six pins of PORTA are output ( Refresh all 7 segments )
	PORTC &= 0xF0;					//	All 7-Segments are initialized to 0




	sei();							// Set Global interrupt on
	Timer1_Init();
	Ext_INT0_Reset();				// Reset Button Interrupt
	Ext_INT1_Pause();				// Pause Button Interrupt
	Ext_INT2_Resume();				// Resume Button Interrupt

	for(;;)
	{




			Stop_Watch();




	}
}

