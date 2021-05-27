/*
 * I2CS.c
 *
 * I2C Slave
 * Basic functions
 *
 * Created: 07/05/2019
 * Author: Dieter Reinhardt
 *
 * Tested with Standard Pinout
 *
 * This software is covered by a modified MIT License, see paragraphs 4 and 5
 *
 * Copyright (c) 2019 Dieter Reinhardt
 *
 * 1. Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * 2. The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * 3. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * 4. This software is strictly NOT intended for safety-critical or life-critical applications.
 * If the user intends to use this software or parts thereof for such purpose additional
 * certification is required.
 *
 * 5. Parts of this software are adapted from Microchip I2C slave driver sample code.
 * Additional license restrictions from Microchip may apply.
 */

#define NOP() asm volatile(" nop \r\n")			// useful to set breakpoint for debugging 

#include <avr/io.h>
#include <driver_init.h>
#include <stdbool.h>
#include <util/delay.h>
#include <I2CS.h>

extern volatile uint8_t	i2c_data;				// data byte sent by slave or received from master
												// defined in main program

uint8_t timeout_cnt;							// 1ms timeout tick counter
uint8_t	num_bytes = 0;							// number of bytes sent/received in transaction

// timeout is handled by 1 millisecond RTC PIT interrupt as follows:

/*
extern uint8_t timeout_cnt;

ISR(RTC_PIT_vect) {						// PIT interrupt handling code, 1 ms interrupt
	timeout_cnt++;;						// increment timeout counter
	RTC.PITINTFLAGS = RTC_PI_bm;		// clear interrupt flag
}
*/

// Slave Status Register
//
// DIF APIF CLKHOLD RXACK COLL BUSERR DIR AP
//
// DIF		Data Interrupt Flag		slave byte transmit or receive operation successfully completed
// APIF		Addr./Stop Int. Flag	valid address has been received or stop condition
// CLKHOLD	Clock Hold				slave is currently stretching the TWI clock period
// RXACK	Received Acknowledge	the most recent acknowledge bit from the master. ‘0’ ACK, ‘1’ NACK
// COLL		Collision				slave has not been able to transmit a high data or NACK bit
// BUSERR	Bus Error				illegal bus condition
// DIR		Read/Write Direction	direction bit from last address packet received from master, ‘1’ master read
// AP		Address or Stop			when APIF is set: 0 STOP, 1 ADR

// Initialize I2C interface
// on exit I2C is enabled and interrupts active

void I2CS_init(void)											// initialize slave
{
#ifdef ALTERNATE_PINOUT
	PORTA_set_pin_dir(2, PORT_DIR_OUT);							// SCL = PA2
	PORTA_set_pin_level(2, false);
	PORTA_set_pin_pull_mode(2, PORT_PULL_OFF);
	PORTA_pin_set_inverted(2, false);
	PORTA_pin_set_isc(2, PORT_ISC_INTDISABLE_gc);
	PORTA_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PA1
	PORTA_set_pin_level(1, false);
	PORTA_set_pin_pull_mode(1, PORT_PULL_OFF);
	PORTA_pin_set_inverted(1, false);
	PORTA_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);
	PORTMUX.CTRLB |= PORTMUX_TWI0_bm;							// set alternate pin mux
#else
	PORTB_set_pin_dir(0, PORT_DIR_OUT);							// SCL = PB0
	PORTB_set_pin_level(0, false);
	PORTB_set_pin_pull_mode(0, PORT_PULL_OFF);
	PORTB_pin_set_inverted(0, false);
	PORTB_pin_set_isc(0, PORT_ISC_INTDISABLE_gc);
	PORTB_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PB1
	PORTB_set_pin_level(1, false);
	PORTB_set_pin_pull_mode(1, PORT_PULL_OFF);
	PORTB_pin_set_inverted(1, false);
	PORTB_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);
#endif
		
	TWI0.CTRLA = 0 << TWI_FMPEN_bp								// FM Plus Enable: disabled
	             | TWI_SDAHOLD_50NS_gc							// Typical 50ns hold time
	             | TWI_SDASETUP_8CYC_gc;						// SDA setup time is 8 clock cycles
		// TWI0.DBGCTRL = 0 << TWI_DBGRUN_bp; Debug Run: disabled

	TWI0.SADDR = SLAVE_ADDRESS									// Slave Address (8 bit address, i. e. bit 0 = 0, will be substituted by R/W bit)
	             | 0 << TWI_ADDREN_bp;							// General Call Recognition Enable: disabled
				 
		// TWI0.SADDRMASK = 0 << TWI_ADDREN_bp; Address Mask Enable: disabled
		//	| 0x0 << TWI_ADDRMASK_gp; Address Mask: 0x0

	TWI0.SCTRLA = 1 << TWI_APIEN_bp								// Address/Stop Interrupt Enable: enabled
	              | 1 << TWI_DIEN_bp							// Data Interrupt Enable: enabled
	              | 1 << TWI_ENABLE_bp							// Enable TWI Slave: enabled
	              | 1 << TWI_PIEN_bp							// Stop Interrupt Enable: enabled
	              | 0 << TWI_PMEN_bp							// Promiscuous Mode Enable: disabled
	              | 0 << TWI_SMEN_bp;							// Smart Mode Enable: disabled
}

void I2C_error_handler()										// error handler, should reset I2C slave internal state	
{
	TWI0.SSTATUS |= (TWI_APIF_bm | TWI_DIF_bm);					// clear interrupt flags
#ifdef ALTERNATE_PINOUT
	PORTA_set_pin_dir(2, PORT_DIR_IN);							// SCL = PA2
	PORTA_set_pin_dir(1, PORT_DIR_IN);							// SDA = PA1
	TWI0.SCTRLA = 0;											// disable slave
	_delay_us(10);												// SCL, SDA tristated high
	PORTA_set_pin_dir(2, PORT_DIR_OUT);							// SCL = PA2
	PORTA_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PA1
#else
	PORTB_set_pin_dir(0, PORT_DIR_IN);							// SCL = PB0
	PORTB_set_pin_dir(1, PORT_DIR_IN);							// SDA = PB1
	TWI0.SCTRLA = 0;											// disable slave
	_delay_us(10);												// SCL, SDA tristated high
	PORTB_set_pin_dir(0, PORT_DIR_OUT);							// SCL = PB0
	PORTB_set_pin_dir(1, PORT_DIR_OUT);							// SDA = PB1
#endif
																// re-enable slave
	TWI0.SCTRLA = 1 << TWI_APIEN_bp								// Address/Stop Interrupt Enable: enabled
				| 1 << TWI_DIEN_bp								// Data Interrupt Enable: enabled
				| 1 << TWI_ENABLE_bp							// Enable TWI Slave: enabled
				| 1 << TWI_PIEN_bp								// Stop Interrupt Enable: enabled
				| 0 << TWI_PMEN_bp								// Promiscuous Mode Enable: disabled
				| 0 << TWI_SMEN_bp;								// Smart Mode Enable: disabled
	TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_NOACT_gc;		// set NACK, no action (just in case)
}

// I2C IRQ handler
// WARNING - do NOT clear interrupt flags during transaction, this will lead to false data when master reads

ISR(TWI0_TWIS_vect)
{
	if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_DIF_bm)) { // APIF && DIF, invalid state (should never occur
		I2C_error_handler();									// but happened during debugging
		return;
	}

	if (TWI0.SSTATUS & TWI_COLL_bm) {							// Collision - slave has not been able to transmit a high data or NACK bit
		I2C_error_handler();	
		return;
	}

	if (TWI0.SSTATUS & TWI_BUSERR_bm) {							// Bus Error - illegal bus condition
		I2C_error_handler();				
		return;
	}

	if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_AP_bm)) { // APIF && AP - valid address has been received
		num_bytes = 0;											// reset number of bytes
		TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;	// send ACK 

		if (TWI0.SSTATUS & TWI_DIR_bm) {						// Master wishes to read from slave (else return and wait for data from master)	
//			_delay_us(5);										// TWI0.SDATA write needs minimum 5 us delay at 10 MHz clock	
			timeout_cnt = 0;									// reset timeout counter, will be incremented by ms tick interrupt
			while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {			// wait until Clock Hold flag set
				if (timeout_cnt > 2) return;					// return if timeout error
			}
			NOP();
			TWI0.SDATA = i2c_data;								// Master read operation
//			i2c_data++;											// debugging		
			TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;										
		}
		return;
	}
	if (TWI0.SSTATUS & TWI_DIF_bm) {							// DIF - Data Interrupt Flag - slave byte transmit or receive completed

		if (TWI0.SSTATUS & TWI_DIR_bm) {						// Master wishes to read from slave
			if (!(TWI0.SSTATUS & TWI_RXACK_bm)) {				// Received ACK from master
//				_delay_us(5);									// TWI0.SDATA write needs minimum 5 us delay at 10 MHz clock
				timeout_cnt = 0;								// reset timeout counter, will be incremented by ms tick interrupt
				while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {		// wait until Clock Hold flag set
					if (timeout_cnt > 2) return;				// return if timeout error
				}
				TWI0.SDATA = i2c_data;							// Master read operation
				num_bytes++;									// increment number of bytes
				TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;							
			} else {											// Received NACK from master
				TWI0.SSTATUS |= (TWI_DIF_bm | TWI_APIF_bm);		// Reset module
				TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;				
			}
			
		} else {												// Master wishes to write to slave
			i2c_data = TWI0.SDATA;
//			i2c_data++;											// debugging			
			num_bytes++;										// increment number of bytes
			if (num_bytes > MAX_TRANSACTION) {					// if maximum data transfer length exceeded
				TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_RESPONSE_gc; // send NACK	
			}	
			else {
				TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;	// send ACK	
			}			
		}
		return;
	}
	
	if ((TWI0.SSTATUS & TWI_APIF_bm) && (!(TWI0.SSTATUS & TWI_AP_bm))) { // APIF && !AP - Stop has been received
		TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
		return;
	}
}

