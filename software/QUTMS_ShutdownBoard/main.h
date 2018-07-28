/*
 * QUTMS_AMU.h
 *
 *  Created on: 25 Apr 2015
 *      Author: julius
 */
/*********** LEGEND ***********/
//*** 	will need to change upon PCB redesign
//** 	might change regularly


#ifndef SHDN_H_
#define SHDN_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "AtmelCAN.h"
#include "SPI.h"

#define SHDN_LOOP_STATUS_HVD	0b0000000000000001	/**< Loop Status of HVD (High Voltage Disconnect) */
#define SHDN_LOOP_STATUS_IMD	0b0000000000000010	/**< Loop Status of IMD (Insulation Measurement Device) */
#define SHDN_LOOP_STATUS_BSPD	0b0000000000000100	/**< Loop Status of BSPD (Brake System Plausibility Device) */
#define SHDN_LOOP_STATUS_BMS_1	0b0000000000001000	/**< Loop Status of the first Accumulator */
#define SHDN_LOOP_STATUS_BMS_2	0b0000000000010000	/**< Loop Status of the second Accumulator */
#define SHDN_LOOP_STATUS_SEG_1	0b0000000000100000	
#define SHDN_LOOP_STATUS_SEG_2	0b0000000001000000
#define SHDN_LOOP_STATUS_SEG_3	0b0000000010000000
#define SHDN_LOOP_STATUS_SEG_10 0b0000000100000000
#define SHDN_LOOP_STATUS_SEG_11 0b0000001000000000
#define SHDN_LOOP_STATUS_SW		0b1000000000000000	/**< Status of the switch that controls a testing condition */


#define SHDN_TEMP_CH	(9)		//**< Channel of the ADC unit that the Shutdown temp sensor is on */
#define SHDN_12V_CH		(4)		//**< ADC Channel for 12V Div sense line */
#define SHDN_5V_CH		(10)	//**< ADC Channel for 5V Div sense line */

#endif SHDN_H_