/*
 * AtmelCAN.c
 *
 * Created: 12/5/2015 5:13:51 PM
 *  Author: julius
 */ 
#include "AtmelCAN.h"

int8_t CAN_sendTest()
{
	uint8_t tData [2] = {111,111};
	uint8_t mob = CAN_findFreeTXMOB();
	if(mob>=0)CAN_TXMOB(mob, 2, tData, 0, 20); //transmit registration and do not wait for finish
	return mob;
}

uint8_t CAN_init()
{
	// Nuke CAN, reset peripheral
	CANGCON = (1 << SWRES);     // Reset the CAN controller hardware and general registers.
	
	// Init all mobs (needed for proper operation
	for(uint8_t mobCount = 0; mobCount <= 5; mobCount++)
	{
		CANPAGE = (mobCount << 4);		//select the message object to modify
		CANCDMOB = 0;					//make sure we do not do any actions on this mob.
		CANSTMOB = 0;					//make sure no interrupts are set.
		// Clear all ID's and Masks in the selected MOB
		CANIDM1 = 0;
		CANIDM2 = 0;
		CANIDM3 = 0;
		CANIDM4 = 0;					//these registers are used to control the mask which filters incoming messages
	}
	
	// Set CANBUS comms speed
	CANBT1 = 0x0E;		//these registers control speed of communication
	CANBT2 = 0x04;		//currently with these values, it is 250kbps
	CANBT3 = 0x13;		//with 8 TQ per bit.

	// Enable interrupts
	CANIE2 = (1 << IEMOB4)|(1 << IEMOB5); //enable interrupts on MOB 4 and 5 for receiving
	CANGIE = (1 << ENRX)|(1 << ENIT); //enable receive interrupt; enable global CAN interrupt (all interrupts)
	//put other initialization functions here.

	// Enable CANBUS
	CANGCON = (1 << ENASTB);
	// Wait for CANBUS peripheral to operate
	if(!(CANGSTA & (1<<ENFG)))return 1;
	return 0;
}

void CAN_RXInit(int8_t mob, uint8_t numBytes, uint32_t IDmsk, uint32_t ID)
{
	CANPAGE = ( mob << 4);		//use the mobth mob for receiving.
	//IDEMSK is sent with the CAN packet, we choose to not require that it be set, and instead focus on ID match
	CANIDM4 = (IDmsk<<03) & 0xF8;	//shifts the value sets RTRMSK to zero and IDEMSK to 0
	CANIDM3 = (IDmsk>>05) & 0xFF;
	CANIDM2 = (IDmsk>>13) & 0xFF;	
	CANIDM1 = (IDmsk>>21) & 0xFF;
	
	CANIDT4 = (ID<<03) & 0xF8;	//shifts the value sets RTRTAG, RB1TAG and RB0TAG to 0
	CANIDT3 = (ID>>05) & 0xFF;
	CANIDT2 = (ID>>13) & 0xFF;
	CANIDT1 = (ID>>21) & 0xFF;
	
	CANCDMOB = (numBytes << DLC0)|(2<<CONMOB0)|(1 << IDE);		//we are expecting only numBytes bytes; also set the mob to receive mode.
}

void CAN_TXMOB(int8_t mob, uint8_t numBytes, uint8_t * data, uint32_t ID, uint8_t ms_loop_until_TXOK)
{
	
	CANPAGE = ( mob << 4); // Set the mob to use for transmission, to the one that is given
	//IDEMSK is sent with the CAN packet, we choose to not set it, and instead the receiver will focus on ID match
	
	CANSTMOB &= ~(1<<TXOK); // 
	
	// Clears the 29/11 bit mask
	CANIDM4 = 0x00;
	CANIDM3 = 0x00;
	CANIDM2 = 0x00;
	CANIDM1 = 0x00;
	
	// Set the 29 bit mask
	CANIDT4 = (ID<<03) & 0xF8;	//shifts the value and sets RTRTAG, RB1TAG and RB0TAG
	CANIDT3 = (ID>>05) & 0xFF;
	CANIDT2 = (ID>>13) & 0xFF;
	CANIDT1 = (ID>>21) & 0xFF;
	
	//CANIDT4 = (0 << RTRTAG)|(0 << RB1TAG)|(0 << RB0TAG);	//shifts the value sets RTRTAG, RB1TAG and RB0TAG
	
	for(uint8_t i = 0; i < numBytes; ++i)
	{
		CANMSG = data[i];
	}
	
	CANCDMOB = (numBytes << DLC0)|(1<<CONMOB0)|(1 << IDE); // Set number of bytes to send in DLC0:3, Set MOb selected to transmit, Set Identifier header packet length to 29bit 

	for(uint32_t i = 0; i < ms_loop_until_TXOK; i++) // Loop until specified wait time is up
	{
		if((CANSTMOB & (1 << TXOK))) break;	// Check for transmission complete
		_delay_ms(1); // Do nothing for 1 ms
	}
}

int8_t CAN_findFreeTXMOB()
{
	for(uint8_t i = 0; i < 4; i++) //tx mobs are 0 to 3 (<4)
	{
		CANPAGE = ( i << 4);		//use the mobth mob
		if((CANSTMOB & ((1<<TXOK)|(1<<BERR)|(1<<SERR)|(1<<FERR)|(1<<CERR)) || !(CANCDMOB & (1<<CONMOB0)))) //if the transmission is complete or the mob has not been setup yet
		{
			return i;			//send back the free mob
		}
	}
	return -1;		//otherwise, none are free
}