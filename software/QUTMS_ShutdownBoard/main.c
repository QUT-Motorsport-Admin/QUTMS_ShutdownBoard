/*
 * QUT Shutdown Board controller
 *
 *  Created on: 28/7/2018
 *      Author: Jonn
 */

/*
 * The system performs two primary functions:
 * 1) Controls, reads and resets the main shutdown loop
 * 2) Provides power and isolation from HV to the TSAL system
 * 2) Interacts on CAN to provide info on the current state of the shutdown system
 */

#include "main.h"

#define LED1ON     PORTD |= 1<<PIND5
#define LED1OFF    PORTD &= 1<<PIND5
#define LED1TOGGLE PORTD ^= 1<<PIND5
#define LED2ON     PORTB |= 1<<PINB4
#define LEF2OFF    PORTB &= 1<<PINB4
#define LED2TOGGLE PORTB ^= 1<<PINB4

#define CC	 	0b01000000000000000000000000000
#define CCmsk		0b01000000000000000000000000000

void IO_init()
{
	DDRD  = 0b00100000;
	PORTD |= (1<<PIND1)|(1<<PIND0);
	
	DDRB  = 0b00010000;
	PORTB |= (1<<PINB7)|(1<<PINB6)|(1<<PINB5)|(1<<PINB2);
	
	DDRC  = 0b00010000;
	PORTC |= (1<<PINC7)|(1<<PINC6)|(1<<PINC5)|(1<<PINC0);
}

//void timer_init()
//{
	//TCCR0A = 0x00;							//normal mode.
	//TCCR0B = (1<<CS02)|(0<<CS01)|(1<<CS00);	//prescale clock by 1024
	//TIMSK0 = (1<<TOIE0);					//allow interrupts
//}

void ADC_init()
{
	ADMUX=(1<<REFS0)|(1<<AREFEN);                      // For Aref=AVcc with external capacitor;
	ADMUX &= ~(1<<ADLAR);								//make sure adlar is not set.
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescaler div factor = 128, 125kHz --> lowest we can go for best accuracy.
}

uint16_t ADC_read(uint8_t channel)
{
	channel = (ADMUX & 0xe0)|(channel & 0x1F); //ADMUX | 0b11100000 and channel | 0b00011111 --> this keeps all bits of ADMUX the same except for the bits signalling which channel to use.
	ADMUX = channel;
	ADCSRA |= (1<<ADSC);							//ADSC (single conversion bit) is set to 1 to start the conversion process.
	while(!(ADCSRA & (1<<ADIF)));				//run a loop while the conversion is taking place.
	uint16_t result = 0;
	result = ADCL;								//read ADCL first, ADCH after --> order is important! --> also not sure if this code is correct. other ADC examples return 'ADC' instead.
	result |= ((3 & ADCH) << 8);
	ADCSRA|=(1<<ADIF);							//once read and done, clear the 'complete' status by writing 1 to the ADIF bit.
	return result;								//pass the 10 bit ADC number to requesting function.
}

uint16_t SHUTDOWN_getBoardTemp() {
	return ADC_read(SHDN_TEMP_CH); // Get status of the SHDN_TEMP
}

uint16_t SHUTDOWN_get5VDiv() {
	return ADC_read(SHDN_5V_CH); // Get status of the SHDN_TEMP
}

uint16_t SHUTDOWN_get12VDiv() {
	return ADC_read(SHDN_12V_CH); // Get status of the SHDN_TEMP
}

/**
 * Gets the status of the shutdown loop
 */
uint16_t SHUTDOWN_getLoopStatus () {
	uint16_t state = 0;
	if(PINB & (1<<PINB1)) {
		state |= SHDN_LOOP_STATUS_HVD;
	}
	if(PINB & (1<<PINB3)) {
		state |= SHDN_LOOP_STATUS_IMD;
	}
	if(PIND & (1<<PIND7)) {
		state |= SHDN_LOOP_STATUS_BSPD;
	}
	if(PINC & (1<<PINC1)) {
		state |= SHDN_LOOP_STATUS_BMS_1;
	}
	if(PINB & (1<<PINB0)) {
		state |= SHDN_LOOP_STATUS_BMS_2;
	}
	if(PINC & (1<<PINC0)) {
		state |= SHDN_LOOP_STATUS_SW;
	}
	if(PINB & (1<<PINB5)) {
		state |= SHDN_LOOP_STATUS_SEG_1;
	}
	if(PINC & (1<<PINC7)) {
		state |= SHDN_LOOP_STATUS_SEG_2;
	}
	if(PIND & (1<<PIND1)) {
		state |= SHDN_LOOP_STATUS_SEG_3;
	}
	if(PIND & (1<<PIND0)) {
		state |= SHDN_LOOP_STATUS_SEG_10;
	}
	if(PINC & (1<<PINC0)) {
		state |= SHDN_LOOP_STATUS_SEG_11;
	}
	return state;
}

int main(void)
{
	_delay_ms(10);
	IO_init();
	_delay_ms(50);
	//SPI_init();
	CAN_init();	//enable this for AVR CAN
	//timer_init();
	
	CAN_RXInit(1, 8, 0xffffffff,0x8800001);
	
	sei();
	
    // Loop forever
    while(1)
    {

	}
}

ISR(CAN_INT_vect)
{
	LED1TOGGLE;
	
	//CANIDT4 is l
	if(CANSIT2 & (1 << SIT5))	//we received a CAN message on mob 5, which is set up to receive exclusively from the Chassis controller.
	{
		CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
		CANSTMOB &= ~(1 << RXOK);	//unset the RXOK bit to clear the interrupt.
		if((CANIDT1 == ((1<<6)|(1<<2))) && ((CANIDT4>>3)== 0b00001) )	//if the received ID has a heartbeat packet
		{
			// The Chassis controller has sent a heartbeat packet
			// Return it, along with the current shutdown loop information
			
			LED2TOGGLE;
			
			uint16_t loopStatus = SHUTDOWN_getLoopStatus();
			uint16_t tempStatus = SHUTDOWN_getBoardTemp();
			uint16_t V5Status = SHUTDOWN_get5VDiv();
			uint16_t V12Status = SHUTDOWN_get12VDiv();
			
			uint8_t status[8];
			status[0] = (V12Status);
			status[1] = (V12Status >> 8);
			status[2] = (V5Status);
			status[3] = (V5Status >> 8);
			status[4] = (tempStatus);
			status[5] = (tempStatus >> 8);
			status[6] = (loopStatus);
			status[7] = (loopStatus >> 8);

			
			//uint8_t mob = CAN_findFreeTXMOB();
			//CAN_TXMOB(mob, 8, status, 0x03000001, 0);
			
			CAN_RXInit(1, 8, 0xffffffff, 0x8800001);
		}
	}
	CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
	CANSTMOB &= ~(1 << RXOK);	//unset the RXOK bit to clear the interrupt.
	
	
}

ISR(TIMER0_OVF_vect)
{

}


