/*
 * QUT_AMU_1.c
 *
 *  Created on: 25 Apr 2015
 *      Author: julius
 */

#include "main.h"

uint16_t SHUTDOWN_eeprom_read(uint16_t address)
{
	while(!eeprom_is_ready());
	return eeprom_read_word((const uint16_t *)address);
}

void SHUTDOWN_eeprom_write(uint16_t address, uint16_t value)
{
	while(!eeprom_is_ready());
	eeprom_write_word((uint16_t *)address, value);
}

void Parameters_init()
{
	deviceID  = SHUTDOWN_eeprom_read(EEPROM_DEVICE_ID);
	fw_version = SHUTDOWN_eeprom_read(EEPROM_FW_VERSION);
}

void PCINT_init() //Needed for detecting interrupts from the MCP2515
{
	PCICR = (1<<PCIE0);			//enable from pin 26 to pin 16
	MCP2515_reg_write(MCP2515_CANINTF, 0b00000000);
	PCMSK0 = (1<<PCINT4)|(1<<PCINT3);
	
}

/**
 * Function: init_cmuData
 * Will fill in the blank cmu struct array with zeros
 */
void init_cmuData(CMU * cmu)
{
	for(uint8_t count = 0; count < CMU_COUNT; count++)
	{
		cmu->CMU_num = 0;
		cmu->CMU_ID = 0;
		for(uint8_t count2 = 0; count2 < CMU_CELL_COUNT; count2++)
		{
			cmu->temperatures[count2]=0;
			cmu->voltages[count2]=0;
		}
		cmu++;
	}
}

void IO_init()
{
	DDRB  = 0b11000110;
	DDRD  = 0b10001011;
	DDRC  = 0b10100111;
	PORTB = 0b00000000;

	PORTD |= (1<<PIND1)|(1<<PIND7)|(1<<PIND3);		//the SS pin needs to be configured as an output, otherwise we get issues on removal of the programmer.
	PORTC |= 1<<PINC7;					//set CS high
	PORTC |= (1<<MCP2515_PIN_RESET);	//set reset to high
	PORTB &= ~(1<<PINB7);		//make sure it's set up for input.
	ALARM_PORT |= (1<<ALARM_PIN);
}

void timer_init()
{
	TCCR0A = 0x00;							//normal mode.
	TCCR0B = (1<<CS02)|(0<<CS01)|(1<<CS00);	//prescale clock by 1024
	TIMSK0 = (1<<TOIE0);					//allow interrupts
}

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

uint8_t CMU_send_read_receipt(CMU * cmu)
{
	uint8_t data = 0x00; //data to send,(zeros)
	uint8_t mob = MCP2515_findFreeTxBuffer(); //obtain a free transmit buffer.
    //send a CAN packet from a free buffer, with recipient of our CMU ID, type: audit request (to now get other CMUs to talk), with 1 byte of zeros
	MCP2515_TX(mob, 0, &data, ((uint32_t)1<<27)|((uint32_t)DEVICE_ID<<18)|((uint32_t)cmu->CMU_ID<<8)|READ_RECEIPT);		
	return 1; //return successful
}

uint16_t CoulombCount_init(){
	return SHUTDOWN_eeprom_read(EEPROM_COULOMB_COUNT);
}

uint16_t CoulombCount_instRead(){
	return ADC_read(ADC_HALL_EFFECT);
}

uint16_t CoulombCount_updateAndRead(uint16_t coulombCount){
	
	/* Representative of how much time the last reading was taken. */
	static uint16_t ticks;									
	uint16_t ticks_diff = globalTicks - ticks;

	/* Take a new reading of the instantaneous current, integrate, and add it to the coulomb count total */
	coulombCount += CoulombCount_instRead()*TIME_CONSTANT;

	/* Start the timer again, which is to be read the next time current is updated */
	ticks = globalTicks;
	SHUTDOWN_eeprom_write(EEPROM_COULOMB_COUNT, coulombCount);
	return coulombCount;
}

/* Rough voltage to SoC representative array, voltage is in millivolts */
uint32_t voltage2SoC_Translation[100] = {100800,99744,99216,98688,98160,97920,97680,97440,97200,96960,
										  96672,96384,96096,95808,95520,95376,95232,95088,94944,94800,
										  94560,94320,94080,93840,93600,93432,93264,93096,92928,92760,
										  92592,92424,92256,92088,91920,91824,91728,91632,91536,91440,
										  91296,91152,91008,90864,90720,90672,90624,90576,90528,90480,
										  90384,90288,90192,90096,90000,89976,89952,89928,89904,89880,
										  89832,89784,89736,89688,89640,89568,89496,89424,89352,89280,
										  89232,89184,89136,89088,89040,88848,88656,88464,88272,88080,
										  87936,87792,87648,87504,87360,87168,86976,86784,86592,86400,
										  85680,84960,84240,83520,82800,80640,78480,76320,74160,72000};

uint8_t SoC_voltageTranslation(uint32_t voltage){
	uint8_t i = 0;
	
	/* Return the index once the value has been approximated. Index is analogous to percent DoD */
	while(voltage > voltage2SoC_Translation[i]){
		i++;
	}
	return 100-i;
}

uint8_t SoC_calculation(uint32_t voltage, uint16_t coulombCount, uint16_t coulombsFull){

	if(voltage < 86500 && voltage > 96000){
		uint8_t SoC = 0;
		
		/* Take the average between voltage translation and coulomb counting deterministic methods. */
		SoC = ((coulombsFull - coulombCount)/coulombsFull)*100;
		SoC = (SoC + SoC_voltageTranslation(voltage))/2;
		return SoC;
	}
	else{
		/* Return the coulomb count as the SoC. */
		return ((coulombsFull - coulombCount)/coulombsFull)*100;
	}
}

void CMU_process_inbound_message(uint8_t rx_buf_address,CMU * cmu)
{
	//potential for optimisation here
    //combine 5 LSBs of SIDH(bits 26..21) and 3 MSBs of SIDL(bits 20..18) which is the CMU ID in the packet structures.
	uint16_t CMU_ID = ( ((MCP2515_reg_read(rx_buf_address+1) & 0x3F)<<3)|
                        ((MCP2515_reg_read(rx_buf_address+2) & 0xE0)>>5) );
    //message type is held in the 5 LSBs of EID0	
	uint8_t message_type = (MCP2515_reg_read(rx_buf_address+4) & 0x1F);

	switch(message_type)
	{
        // If we get a audit response message, we need to register this CMU
		case AUDIT_RESPONSE:
			// flash_LED(5,YELLOW_LED,10);
			switch(CMU_Register(cmu, CMU_ID))
			{
				case 0:
					break;	// No issues with registering.
				case 1:
					error_state(ERROR_DUPLICATE_CMU_ID);
					break;
				case 2:
					error_state(ERROR_NUMEROUS_CMU);
					break;
				default:
					break;
			}
			break;
		case TEMP1_ID:
			if(!CMU_Store_Data(cmu, CMU_ID, TEMP1_ID, rx_buf_address))
			    { error_state(ERROR_UNKNOWN_CMU_ID); } // Fails if the CMU ID is not known 
			break;
		case TEMP2_ID:
			if(!CMU_Store_Data(cmu, CMU_ID, message_type, rx_buf_address))
			    { error_state(ERROR_UNKNOWN_CMU_ID); } // Fails if the CMU ID is not known 
			break;
		case VOLT1_ID:
			if(!CMU_Store_Data(cmu, CMU_ID, message_type, rx_buf_address))
			    { error_state(ERROR_UNKNOWN_CMU_ID); } // Fails if the CMU ID is not known 
			break;
		case VOLT2_ID:
			if(!CMU_Store_Data(cmu, CMU_ID, message_type, rx_buf_address))
			    { error_state(ERROR_UNKNOWN_CMU_ID); } // Fails if the CMU ID is not known 
			break;
		default:
			break;
	} // This next section could be better off with the bit modify function.
}

uint8_t CMU_send_audit_request()
{
	//uint8_t data = 0x00;
	uint8_t free_buffer = MCP2515_findFreeTxBuffer(); //obtain a free transmit buffer.

	if(free_buffer)
	{
		//flash_LED(1, YELLOW_LED, 50);
        // Send a CAN packet from a free buffer, with recipient of our CMU ID, type audit request (to now get others to talk), with 1 byte of zeros
		// MCP2515_tx(AMU,free_buffer,DEVICE_ID,0x00,AUDIT_REQUEST,0,&data);		
		// MCP2515_TX(MCP2515_findFreeTxBuffer(), 0, &status, ((uint32_t)1<<27)|((uint32_t)DEVICE_ID<<18)|AUDIT_REQUEST );
		return 1; //return successful
	}
	else
	{
		return 0; //return error, there were no free buffers.
	}
}

void CMU_PollandProcess_RxBuffers(CMU * cmu)
{
	
	//potential for optimisation here, using FILHIT bits.
	uint8_t status = (MCP2515_receive_status() & 3);		//poll to see if we have a message waiting.
	switch(status)
	{
		case 0:					//no message waiting (/VERY/ unlikely, as interrupt has said we have one.)
			break;
		case 1:					//rxb0 only has a message. (also /VERY/ unlikely)
			CMU_process_inbound_message(MCP2515_RXB0, cmu);
			MCP2515_bit_modify(MCP2515_CANINTF,0x00, status);				//e.g if status is 0b00000001, it will only modify the 0th pin.
			break;
		case 2:					//rxb1 only has a message
			CMU_process_inbound_message(MCP2515_RXB1, cmu);
			MCP2515_bit_modify(MCP2515_CANINTF,0x00, status);
			break;
		case 3:					//both rxb0 and rxb1 have a message
			CMU_process_inbound_message(MCP2515_RXB1, cmu);
			CMU_process_inbound_message(MCP2515_RXB0, cmu);
			MCP2515_bit_modify(MCP2515_CANINTF,0x00, status);
			break;
		default:				//shouldn't occur, but treat as no messages waiting
			break;
	}
	
}

uint8_t CMU_Check_Registration_Status(CMU * cmu)
{
	uint8_t count;
	for(count = 0; count < CMU_COUNT; count++)							//look through all recorded CMUs, looking for empty ones.
	{

		if(cmu->CMU_ID == 0)break;										//if we find an empty CMU ID, finish the search.
		cmu++;															//increment the element of the array we are looking at.
	}
	if(count==CMU_COUNT)return 1;										//all required CMUs have completed initial registration.
	else if(CMURegistrationCount > CMU_COUNT)return 255;				//if there have been many device registrations and still not enough cells registered, it means one is overwriting the other, which means duplicate device_ID(s)
	else return 0;														//not all CMUs have registered.
}
void CMU_Wake()
{
	CMURxPacketCount= 0;
	PORTC &= ~(1<<PINC1);

	_delay_ms(1);
	PORTC |= (1<<PINC1);
}
void CMU_Wake_set()
{
	CMURxPacketCount= 0;
	PORTC &= ~(1<<PINC1);
}
void CMU_Wake_reset()
{

	PORTC |= (1<<PINC1);
}


uint8_t CMU_Register(CMU * cmu, uint16_t CMU_ID)
{
	for(uint8_t count = 0; count < CMU_COUNT; count++)			//parsing through our cmu array, looking for a free spot, or if we are already registered.
	{

		if(cmu->CMU_ID == 0)			//if the cmu element of the array has not been allocated, or if the cmu is already registered,
		{
			if(!CMUAllRegistered)CMUAudit++;		//while it is in it's registration phase, increment the registration counter;
			cmu->CMU_ID = CMU_ID;								//assign this element to the CMU that sent the reg request.

			while (!CMU_send_read_receipt(cmu));			//keep trying to send, until a tx buffer is free.
			break;
		}
		else if(cmu->CMU_ID == CMU_ID)		//if this function encounters its own ID, it means there is a duplicate ID. 
		{
			return 1;			//return error.
		}
		cmu++;
		if(count == (CMU_COUNT-1))		//otherwise if all of the elements have already been allocated, we have an extra CMU.
		{
			return 2;		//return error
		}
	}
	return 0;
	
}

uint8_t CMU_Store_Data(CMU * cmu, uint8_t CMU_ID, uint8_t message_type2, uint8_t address)
{
	uint8_t found = 0;
	uint8_t data[8];											//make a bucket for our data.
	CMU * cmu2 = cmu;
	cmu2++;
	
	//this function takes a pointer of a suitably sized array for the first address of the data we will acquire.
	MCP2515_RxBufferRead(data, address+6);						//collect our data, using the rx read buffer instruction.
	//interrupt for given RX buffer will now be cleared, by previous function.

	for(uint8_t counter = 0; counter < CMU_COUNT; counter++)	//move through the cells, looking for a matching one to the ID we have.
	{
		//_delay_ms(2);

		
		if(cmu->CMU_ID == CMU_ID)				//if cmu is the one we are looking for,
		{
			found = 1;
			switch(message_type2)				//depending on the message type
			{
				case TEMP1_ID:							//if it was the first TEMPERATURE packet,
					
					cmu->temperatures[0]=data[0]<<8;	//data 0 will hold the 8 MSBs of temp cell 0
					cmu->temperatures[0]|=data[1];		//data 1 will hold the 8 LSBs of temp cell 0
					cmu->temperatures[1]=data[2]<<8;	//data 2 will hold the 8 MSBs of temp cell 1
					cmu->temperatures[1]|=data[3];		//and so on...
					cmu->temperatures[2]=data[4]<<8;
					cmu->temperatures[2]|=data[5];
					cmu->temperatures[3]=data[6]<<8;
					cmu->temperatures[3]|=data[7];
					CMU_data_count++;
					
					break;
				case VOLT1_ID:							//if it was the first VOLTAGE packet,
					
					cmu->voltages[0]=data[0]<<8;		//data 0 will hold the 8 MSBs of voltages cell 0
					cmu->voltages[0]|=data[1];			//data 1 will hold the 8 LSBs of voltages cell 0
					cmu->voltages[1]=data[2]<<8;		//data 2 will hold the 8 MSBs of voltages cell 1
					cmu->voltages[1]|=data[3];			//and so on...
					cmu->voltages[2]=data[4]<<8;
					cmu->voltages[2]|=data[5];
					cmu->voltages[3]=data[6]<<8;
					cmu->voltages[3]|=data[7];
					
					CMU_data_count++;
					break;
				case TEMP2_ID:							//if it was the second TEMPERATURE packet,
				
					cmu->temperatures[4]=data[0]<<8;	//data 0 will hold the 8 MSBs of temp cell 4
					cmu->temperatures[4]|=data[1];		//data 1 will hold the 8 LSBs of temp cell 4
					cmu->temperatures[5]=data[2]<<8;	//data 2 will hold the 8 MSBs of temp cell 5
					cmu->temperatures[5]|=data[3];		//and so on...
					cmu->temperatures[6]=data[4]<<8;
					cmu->temperatures[6]|=data[5];
					cmu->temperatures[7]=data[6]<<8;
					cmu->temperatures[7]|=data[7];
					
					CMU_data_count++;
					//LED_on(YELLOW_LED);
					break;
				case VOLT2_ID:							//if it was the second VOLTAGE packet,
					cmu->voltages[4]=data[0]<<8;		//data 0 will hold the 8 MSBs of voltages cell 0
					cmu->voltages[4]|=data[1];			//data 1 will hold the 8 LSBs of voltages cell 0
					cmu->voltages[5]=data[2]<<8;		//data 2 will hold the 8 MSBs of voltages cell 1
					cmu->voltages[5]|=data[3];			//and so on...
					cmu->voltages[6]=data[4]<<8;
					cmu->voltages[6]|=data[5];
					cmu->voltages[7]=data[6]<<8;
					cmu->voltages[7]|=data[7];
					
					CMU_data_count++;
					break;
				default:
				
					break;
			}
		
		}
		cmu++; //if this cmu was not a match, check the next one.
	}
	if (!found) return 0; //if we went through the entire cell database and couldn't find the cell ID, we will need to error and do another audit request.
	else return 1; //otherwise all ok.
	return 1;
}


uint16_t TX_cellTemps(CMU _cmu, uint16_t * max_cell, uint16_t * min_cell, uint32_t *avgSum)
{
	int8_t mob;
	uint8_t tempData[8];

	for(uint8_t cellCount = 0; cellCount <= 3; cellCount++)
	{
		*avgSum +=_cmu.temperatures[cellCount];
		*min_cell = _cmu.temperatures[cellCount] < *min_cell ? _cmu.temperatures[cellCount] : *min_cell;		//if the cell temperature is lower than the current minimum temp, update the minimum temp
		*max_cell = _cmu.temperatures[cellCount] > *max_cell ? _cmu.temperatures[cellCount] : *max_cell;		//if the cell temperature is higher than the current maximum temp, update the maximum temp
		tempData[cellCount*2]	= _cmu.temperatures[cellCount] >> 8;
		tempData[cellCount*2+1] = _cmu.temperatures[cellCount];
	}
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 8, tempData, ((uint32_t)1<<25)|((uint32_t)DEVICE_ID<<13)|CC_TEMP1_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	
	for(uint8_t cellCount = 4; cellCount <= 7; cellCount++)
	{
		*avgSum +=_cmu.temperatures[cellCount];
		*min_cell = _cmu.temperatures[cellCount] < *min_cell ? _cmu.temperatures[cellCount] : *min_cell;		//if the cell temperature is lower than the current minimum temp, update the minimum temp
		*max_cell = _cmu.temperatures[cellCount] > *max_cell ? _cmu.temperatures[cellCount] : *max_cell;		//if the cell temperature is higher than the current maximum temp, update the maximum temp
		tempData[(cellCount-4)*2]	= _cmu.temperatures[cellCount] >> 8;
		tempData[(cellCount-4)*2+1] = _cmu.temperatures[cellCount];
	}
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 8, tempData, ((uint32_t)1<<25)|((uint32_t)DEVICE_ID<<13)|CC_TEMP2_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	
	return 0;
	
}

uint8_t TX_cellVoltage(CMU _cmu, uint16_t * max_cell, uint16_t * min_cell, uint32_t * avgSum)
{
	int8_t mob;
	uint8_t tempData[8];
	for(uint8_t cellCount = 0; cellCount <= 3; cellCount++)
	{
		*min_cell = _cmu.voltages[cellCount] < *min_cell ? _cmu.voltages[cellCount] : *min_cell;		//if the cell temperature is lower than the current minimum temp, update the minimum temp
		*max_cell = _cmu.voltages[cellCount] > *max_cell ? _cmu.voltages[cellCount] : *max_cell;		//if the cell temperature is higher than the current maximum temp, update the maximum temp
		*avgSum +=_cmu.voltages[cellCount];
		tempData[cellCount*2]	= _cmu.voltages[cellCount] >> 8;
		tempData[cellCount*2+1] = _cmu.voltages[cellCount];
	}
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 8, tempData, ((uint32_t)1<<25)|((uint32_t)DEVICE_ID<<13)|CC_VOLT1_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	for(uint8_t cellCount = 4; cellCount <= 7; cellCount++)
	{
		*min_cell = _cmu.voltages[cellCount] < *min_cell ? _cmu.voltages[cellCount] : *min_cell;		//if the cell temperature is lower than the current minimum temp, update the minimum temp
		*max_cell = _cmu.voltages[cellCount] > *max_cell ? _cmu.voltages[cellCount] : *max_cell;		//if the cell temperature is higher than the current maximum temp, update the maximum temp
		*avgSum +=_cmu.voltages[cellCount];
		tempData[(cellCount-4)*2]	= _cmu.voltages[cellCount] >> 8;
		tempData[(cellCount-4)*2+1] = _cmu.voltages[cellCount];
	}
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 8, tempData, ((uint32_t)1<<25)|((uint32_t)DEVICE_ID<<13)|CC_VOLT2_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	return 0;
}

uint8_t TX_globalData(uint16_t max_volt, uint16_t min_volt, uint16_t max_temp, uint16_t min_temp, uint32_t avgVSum, uint32_t avgTSum )
{
	int8_t mob;
	uint8_t tempData[8];
	uint16_t avgV = avgVSum/(CMU_COUNT*8), avgT = avgTSum/(CMU_COUNT*8);
	
	tempData[0]	= min_volt >> 8;
	tempData[1] = min_volt;
	tempData[2]	= max_volt >> 8;
	tempData[3] = max_volt;
	tempData[4]	= avgV >> 8;
	tempData[5] = avgV;
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 6, tempData, ((uint32_t)1<<25)|((uint32_t)deviceID<<13)|CC_V_GLOBAL_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	tempData[0]	= min_temp >> 8;
	tempData[1] = min_temp;
	tempData[2]	= max_temp >> 8;
	tempData[3] = max_temp;
	tempData[4]	= avgT >> 8;
	tempData[5] = avgT;
	mob = CAN_findFreeTXMOB();
	CAN_TXMOB(mob, 6, tempData, ((uint32_t)1<<25)|((uint32_t)deviceID<<13)|CC_T_GLOBAL_ID, 0); //transmit first 4 cell data and do not wait for finish
	_delay_ms(1);
	
	return 0;
}
void LED_on( uint8_t selection)
{
	switch(selection)
	{
		case YELLOW_LED:
			PORTD &= ~(1<<PIND7);
			break;
		case RED_LED:
			PORTD &= ~(1<<PIND1);
			break;
		default:
			break;
	}
}
void LED_off( uint8_t selection)
{
	switch(selection)
	{
		case YELLOW_LED:
			PORTD |= (1<<PIND7);
			break;
		case RED_LED:
			PORTD |= (1<<PIND1);
			break;
		default:
			break;
	}
}
void flash_LED(uint8_t number, uint8_t selection, uint8_t duration)
{
	duration = duration/2;
	if(selection == RED_LED)
	{
		for(uint8_t count = 0; count < number ;count++)
		{
			PORTD &= ~(1<<PIND1);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
			PORTD |= (1<<PIND1);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
		}
	}
	else
	{
		for(uint8_t count = 0;count<number;count++)
		{
			PORTD &= ~(1<<PIND7);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
			PORTD |= (1<<PIND7);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
		}
	}

}

void error_state(uint8_t code)
{
	CMU_Wake_reset();
	ALARM_PORT &= ~(ALARM_PIN);
	cli();
	while(1)
	{
		ALARM_PORT ^= (1<<ALARM_PIN);
		CAN_TXMOB(CAN_findFreeTXMOB(), 1, &code, ((uint32_t)1<<25)|((uint32_t)deviceID<<13)|ALARM_MSG,0);
		flash_LED(2,YELLOW_LED,100);
		flash_LED(2,RED_LED,100);
	}
}

int main(void)
{
	SHUTDOWN_eeprom_write(EEPROM_FW_VERSION, 1002);
	SHUTDOWN_eeprom_write(EEPROM_DEVICE_ID, 2);
	
	_delay_ms(10);
	IO_init();
	Parameters_init();
	CMU_Wake_reset();
	_delay_ms(50);
	SPI_init();
	init_cmuData(cmuData); //initialise our struct(s) with valid null data.
	CAN_init();	//enable this for AVR CAN
	MCP2515_init();
	PCINT_init();
	timer_init();
	CAN_RXInit(5, 4, 0, 0);
    //mark the transmit buffer as free. seems hacky, but the first interrupt never actually sets, seeing as the tx buffers were never actually busy.
	MCP2515_reg_write(MCP2515_CANINTF, 0b00000000);		
	sei();
	CMUAudit = 0;

	uint8_t status = 0; 
	MCP2515_FilterInit(0, AUDIT_RESPONSE); //setup the filter to receive audit responses
	MCP2515_RXInit(0, 0); //setup the buffer to match to the packet type bits
	_delay_ms(50);
	CMU_Wake_set();
	LED_on(RED_LED);
	HEARTBEATCOUNTER = 0;
	CMU_WAKE_TIMER = 0;
    //if the number of registered CMU does not match the required number
	while(CMUAudit != CMU_COUNT)
	{
		flash_LED(1,YELLOW_LED,50);
        //the &status is just dummy data so in case the function is written wrong, no harm should come to RAM
		MCP2515_TX(MCP2515_findFreeTxBuffer(), 0, &status, ((uint32_t)1<<27)|((uint32_t)DEVICE_ID<<18)|AUDIT_REQUEST );
		_delay_ms(5);
        // If the time the checking for the CMU responses are over 1 second
		if(CMU_WAKE_TIMER > TIM_1_SEC)error_state(ERROR_CMU_TIMEOUT);
		if(STATUS_REG & MCP2515_DataWaiting)
		{
			status = MCP2515_reg_read(MCP2515_CANINTF);
			if(status & 3)
			{
                //if the program has indicated there is data waiting on the MCP2515, process this data.
				CMU_PollandProcess_RxBuffers(cmuData);
			}
			STATUS_REG &= ~(MCP2515_DataWaiting);
			MCP2515_reg_write(MCP2515_CANINTF, 0b00000000);
		}
	}
	
	LED_off(RED_LED);
	CMU_Wake_reset();
	flash_LED(2,YELLOW_LED,200);
	_delay_ms(500);
	
	//MCP2515_FilterInit(0, 0);
	//MCP2515_FilterInit(1, 0);
	//MCP2515_RXInit(0, 0);	//setup the buffer to match to the packet type bits
	//MCP2515_RXInit(1, 0);	//setup the buffer to match to the packet type bits
	
	while(!(STATUS_REG & MODE_HEARTBEATRECVD));
	CMUTimeToScan = 1;
	
    // Loop forever
    while(1)
    {
		if(STATUS_REG & MODE_BALANCING)
		{
			uint8_t balanceInstruction[5] = { (uint8_t)(3700>>8),(uint8_t)(3700),45,0,0};
			MCP2515_TX(MCP2515_findFreeTxBuffer(),sizeof(balanceInstruction),balanceInstruction,((uint32_t)1<<27)|((uint32_t)DEVICE_ID<<18)|BALANCE_ON);
			LED_on(YELLOW_LED);
		}
		else
		{
			LED_off(YELLOW_LED);
		}
		
		if(CMUTimeToScan) //if the program indicates that we will need new cell information, send wake signals.
		{
			CMU_data_count = 0;
			CMU_Wake_set();
			_delay_ms(5);
			if((STATUS_REG & MODE_HEARTBEATRECVD)==0)
			{
				STATUS_REG &= ~MODE_BALANCING;
				//send alarm code here
				//this is an error state
			}
			if(STATUS_REG & MODE_BALANCING)
			{ CMU_Wake_set(); }
			else
			{ CMU_Wake_reset(); }

			LED_on(RED_LED);
			
			CMU_WAKE_TIMER = 0;
			do
			{
                //give all CMUs a total of 1 second to respond
				if(CMU_WAKE_TIMER > TIM_1_SEC) error_state(ERROR_CMU_TIMEOUT);
				if(STATUS_REG & MCP2515_DataWaiting)
				{
					status = MCP2515_reg_read(MCP2515_CANINTF);
					if(status & 3)
					{
						CMU_PollandProcess_RxBuffers(cmuData);	//if the program has indicated there is data waiting on the MCP2515, process this data.
						STATUS_REG &= ~(MCP2515_DataWaiting);
					}
				}
			}while(CMU_data_count != CMU_COUNT*4);
				
			LED_off(RED_LED);
			CMUTimeToScan = 0;
			
		}
		if(STATUS_REG & MODE_HEARTBEATRECVD)
		{
			uint16_t min_temp = 0xFFFF;		//set it to maximum value
			uint16_t max_temp = 0x00;		//set it to minimum value
			uint16_t min_volt = 0xFFFF;
			uint16_t max_volt = 0x00;
			uint32_t avgVSum = 0;
			uint32_t avgTSum = 0;
			for(uint8_t i = 0; i<CMU_COUNT; i++)
			{
				TX_cellVoltage(cmuData[i], &max_volt, &min_volt, &avgVSum);
				TX_cellTemps(cmuData[i], &max_temp, &min_temp, &avgTSum);
			}
			TX_globalData( max_volt, min_volt, max_temp, min_temp, avgVSum, avgTSum);
			//CoulombCount_readAndUpdate();
			STATUS_REG &= ~MODE_HEARTBEATRECVD;
			if(max_volt > CELL_V_ERR_MAX || min_volt < CELL_V_ERR_MIN || max_temp > CELL_T_ERR_MAX || min_temp < CELL_T_ERR_MIN)
				error_state(ERROR_V_T_OOR);
		}
		CAN_RXInit(5,0,CCmsk,CC);
		_delay_ms(50);
    }
}

//ISR(PCINT3_vect)			//This interrupt from the MCP2515 is not reliable, it doesn't trigger.
//{
//	//upon signal of the RX0 buffer full pin,
//	PCIFR |= (1<<PCIE3);				//should clear on this execution routine.
//}

ISR(PCINT0_vect)
{
	//upon signal of the INT pin.
	LED_off(RED_LED);
	if(PINB & ~(1<<PINB3))		//if the pin is low
	{
		STATUS_REG |= MCP2515_DataWaiting;
	}
	PCIFR |= (1<<PCIE0);								//clear the interrupt.
}

ISR(CAN_INT_vect)
{
	//CANIDT4 is l
	if(CANSIT2 & (1 << SIT5))	//we received a CAN message on mob 5, which is set up to receive exclusively from the Chassis controller.
	{
		CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
		CANSTMOB &= ~(1 << RXOK);	//unset the RXOK bit to clear the interrupt.
		if((CANIDT1 == ((1<<6)|(1<<4))) && (CANIDT2==deviceID) && ((CANIDT4>>3)==CC_HEARTBEAT) )	//if the received ID has a heartbeat packet
		{
			STATUS_REG |= MODE_HEARTBEATRECVD;
			HEARTBEATCOUNTER = 0;
			uint32_t heartbeat =	(uint32_t)CANMSG<<24;	//byte 0
			heartbeat |= (uint32_t)CANMSG<<16;				//byte 1
			heartbeat |= (uint32_t)CANMSG<<8;				//byte 2
			heartbeat |= (uint32_t)CANMSG;					//byte 3

			if((heartbeat&1) == 1)
				STATUS_REG |= MODE_BALANCING;
			else
				STATUS_REG &= ~MODE_BALANCING;
			///considering we got a heartbeat, we will send voltages back here. TBA

		}
		if((CANIDT1 == ((1<<6)|(1<<4))) && (CANIDT2==deviceID) && ((CANIDT4>>3)==CC_MODE_CHANGE) )	//if the received ID has a mode change packet
		{
			uint16_t modeAddress =	CANMSG<<8;	//byte 0
			modeAddress |= CANMSG;				//byte 1
			uint16_t modeValue	=	CANMSG<<8;	//byte 2
			modeValue  |= CANMSG;				//byte 3
			switch(modeAddress)
			{
				case 1:
					if(modeValue)
						STATUS_REG |= MODE_BALANCING;
					else
						STATUS_REG &= ~MODE_BALANCING;
					break;
				case 2:
					break;
					//add functionality for changing scanning frequency
				default:
					break;
			}

		}
		CAN_RXInit(5,0,CCmsk,CC);

	}
	CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
	CANSTMOB &= ~(1 << RXOK);	//unset the RXOK bit to clear the interrupt.
}

ISR(TIMER0_OVF_vect)
{
	timerCounter++;
	HEARTBEATCOUNTER++;
	CMU_WAKE_TIMER++;
	if(HEARTBEATCOUNTER > TIM_4_SEC && !(STATUS_REG & MODE_HEARTBEATRECVD))
	{
		error_state(ERROR_NO_HEARTBEAT);
	}
	if(!CMUAllRegistered && timerCounter >= CMUAuditTimeout*61)
	{
		//using the timer, we will re-wake all the CMU's approximately every CMUAuditTimeout*1 seconds.
		timerCounter = 0;
		CMUTimeToScan = 1; //this actually only triggers a new scan when in auditing mode.
	}
	if (timerCounter >= CMUScanInterval*61) //CMUInterval seconds,
	{
		timerCounter = 0; //reset our counter
		CMUTimeToScan = 1; //set the flag that will trigger a scan routine on next flag.
	}
}


