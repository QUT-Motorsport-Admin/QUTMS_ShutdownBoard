/*
 * QUTMS_AMU.h
 *
 *  Created on: 25 Apr 2015
 *      Author: julius
 */
/*********** LEGEND ***********/
//*** 	will need to change upon PCB redesign
//** 	might change regularly


#ifndef QUTMS_AMU_H_
#define QUTMS_AMU_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "AtmelCAN.h"
#include "MCP2515.h"



#define ALARM_PORT	PORTC
#define ALARM_PIN	PINC0
#define RAIL_V 5000						//***
#define YELLOW_LED 0					//***
#define RED_LED 1						//***

#define TEMP_MULTIPLEX_PORT PORTD		//***if ever the PCB design changes and the adc multiplexer changes pins, these two need to be changed
#define TEMP_MULTIPLEX_CH 3				//***

#define VOLT1_ID 		0b00001			//**
#define VOLT2_ID 		0b00010			//**
#define TEMP1_ID 		0b00100			//**
#define TEMP2_ID 		0b00101			//**
#define CC_V_GLOBAL_ID	0x1
#define CC_T_GLOBAL_ID	0x2
#define ALARM_MSG		0x3
#define CC_VOLT1_ID		0xA
#define CC_VOLT2_ID		0xB
#define CC_TEMP1_ID		0x64
#define CC_TEMP2_ID		0x65
#define STATA_ID		0b01001			//**
#define AUDIT_REQUEST 	0b10001		//**
#define AUDIT_RESPONSE 	0b10011		//**
#define READ_RECEIPT	0b10010
#define BALANCE_ON		0b10100
#define BALANCE_OFF		0b10101

#define CMU_COUNT 			3			//** 1 for the moment, will need to be 18 on release.
#define CMU_CELL_COUNT		8			//*** this will rarely need to change. unless the PCB changes

//CMU EEPROM parameters to send
#define EEPROM_DEVICE_ID					0x0000			//**
#define EEPROM_FW_VERSION					0x0002			//**
#define EEPROM_COULOMB_COUNT				0x0004

#define CC	 	0b01000000000000000000000000000
#define CCmsk		0b01000000000000000000000000000
#define MCP2515_BFPCTRL  	0x0C		//***
#define ANY_CMU 			0x00		//**
#define DEVICE_ID  			0x01		//**
#define CHASSIS_ID			0x01		//**
#define ADMIN				0x02		//**
#define AMU					0x01		//**

#define ADC_HALL_EFFECT		0x00

#define CC_HEARTBEAT		0b00001

#define CC_BALANCE			0x0001
#define CC_MODE_CHANGE		0b10000
#define CC_AMU_MODE			0b00100
#define HEARTBEATCOUNTER	GPIOR2
#define STATUS_REG			GPIOR0						//status general purpose register.
#define CMU_WAKE_TIMER		GPIOR1
#define TIM_1_SEC			61
#define TIM_2_SEC			122
#define TIM_4_SEC			244

#define CELL_V_ERR_MAX		4500
#define CELL_V_ERR_MIN		2900
#define CELL_T_ERR_MAX		50
#define CELL_T_ERR_MIN		0

#define MODE_CHARGING		0b00000001
#define MODE_BALANCING		0b00000010
#define MODE_HEARTBEATRECVD	0b00000100
#define MODE_CMUBAD		0b00001000
#define MODE_CHASSISBAD		0b00010000
#define MCP2515_DataWaiting 0b10000000				//data waiting bit.

#define ERROR_V_T_OOR			1						//voltage or temp out of range
#define ERROR_DUPLICATE_CMU_ID	2						//two CMUs with same ID on the network
#define ERROR_NUMEROUS_CMU		3						//more CMUs than expected are trying to register
#define ERROR_NO_HEARTBEAT		4						//no response from Chassis controller
#define ERROR_CMU_TIMEOUT		5						//no response from 1 or more CMUs
#define ERROR_UNKNOWN_CMU_ID	6						//unregistered CMU sending data


typedef struct CMU
{
	uint8_t CMU_num, CMU_ID;
	uint16_t voltages[CMU_CELL_COUNT];	//will change with change of parameter. Unlikely to.
	uint16_t temperatures[CMU_CELL_COUNT];
}CMU;

void 	SPI_init();

void 	init_cmuData(CMU * cmu);

void 	IO_init();

uint8_t spi_send_byte(uint8_t c);
void	PCINT_init();

void error_state(uint8_t flag);
uint8_t CMU_send_read_receipt(CMU * cmu);
void 	CMU_process_inbound_message(uint8_t rx_buf_address,CMU * cmu);
uint8_t CMU_send_audit_request();
void 	CMU_PollandProcess_RxBuffers(CMU * cmu);
uint8_t CMU_Store_Data(CMU * cmu, uint8_t CMU_ID, uint8_t message_type2, uint8_t address);
uint8_t CMU_Check_Registration_Status(CMU * cmu);
void 	CMU_Wake();
void	CMU_Wake_reset();
void	CMU_Wake_set();
uint8_t CMU_Register(CMU * cmu, uint16_t CMU_ID);

void LED_on( uint8_t selection);
void LED_off( uint8_t selection);
void flash_LED(uint8_t number, uint8_t selection, uint8_t duration);

uint16_t deviceID;
uint8_t warningLight;
uint16_t fw_version;
uint8_t CMURxPacketCount = 0;
uint16_t timerCounter = 0;
uint8_t	CMUScanInterval = 10;			//in seconds
uint8_t CMUAuditTimeout = 2;			//in seconds
#define TIME_CONSTANT 2;
uint8_t CMUTimeToScan = 0;
uint8_t CMUAllRegistered = 0;
uint8_t CMURegistrationCount = 0;
volatile uint16_t globalTicks = 0;
volatile uint8_t CMU_data_count = 0;
volatile uint16_t coulombCount = 0;
uint8_t CMUAudit	=	0;		//initial value as we need it to audit.
uint8_t CMUHealthByte = 0;
volatile uint8_t MCP2515_InterruptWaiting = 0;
uint8_t MCP2515_retrievedData[8];
CMU 	cmuData[CMU_COUNT];				//AMU will be taking care of CMU_COUNT cmus
void request_cmuData();
#endif /* QUTMS_AMU_H_ */
