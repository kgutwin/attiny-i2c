/*
 * I2CSD.c
 *
 * I2C Slave
 * Device Driver for Byte Register Array
 *
 * Created: 13/05/2019
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

#define NOP() asm volatile(" nop \r\n")			// useful e.g. to set breakpoint for debugging 

#include <avr/io.h>
#include <driver_init.h>
#include <stdbool.h>
#include <util/delay.h>
#include <I2CSD.h>

volatile uint8_t	i2c_reg_array[SLAVE_REG_SIZE] = {0};
volatile uint8_t	i2c_stop_flag = 0;			// Stop state, 1, 2, 3, 0xff
uint8_t i2c_reg_start = 0;						// first register address of current transaction
uint8_t i2c_bytes = 0;							// number of bytes sent/received during transaction

volatile uint8_t timeout_cnt;					// 1ms timeout tick counter
uint8_t	num_bytes = 0;							// number of bytes sent/received in transaction
uint8_t	i2c_slave_state = 0;					// state machine
uint8_t i2c_reg_addr = 0;						// next register address

// timeout is handled by 1 millisecond RTC PIT interrupt as follows:

/*
extern uint8_t timeout_cnt;

ISR(RTC_PIT_vect) {						// PIT interrupt handling code, 1 ms interrupt
	timeout_cnt++;;						// increment timeout counter
	RTC.PITINTFLAGS = RTC_PI_bm;		// clear interrupt flag
}
*/

// Slave is organized as Byte Array (Register Array) with Read/Write access from Master (similar to EEPROM)
// maximum number of data bytes per transaction is limited to MAX_TRANSACTION.
// Limitation is necessary to ensure error recovery, otherwise the master might clock out
// an unlimited number of bytes when trying to recover from a stuck low SDA line since
// slave sends ACK after each recovery cycle and thus blocks the bus again.

// Master Register Write Cycle
// Start	Device Address, W			Slave ACK
//			Data Register Address		Slave ACK
//			Data Write					Slave ACK
//			...
//			Data Write					Slave ACK
// Stop

// Master Register Read Cycle
// Start	Device Address, W			Slave ACK
//			Data Register Address		Slave ACK
// Restart	Device Address, R			Slave ACK
//			Data Read					Master ACK
//			...
//			Data Read					Master NACK
// Stop

// if Master requests illegal address (address overflow), internal R/W register address will be reset zero
// the register address is incremented and limited to valid range after each data read or write operation
// e.g. valid address range 0 .. 7: read from address 13 will return data from address 0
// e.g. valid address range 0 .. 7: writing 4 consecutive bytes starting with address 6, address sequence will be 6 - 7 - 0 - 1

// slave device states:
// 0: idle, waiting for device address
// 1: slave device address received, waiting for register address
// 2: register address received
// 3: ongoing master data read operation
// 4: ongoing master data write operation

// when a Stop has been received, i2c_stop_flag will be set to signal data read/write transfer, may be reset in main program

// Initialize I2C interface
// on exit I2C is enabled and interrupts active

void I2CSD_init(void)											// initialize slave device
{
	i2c_reg_start = 0;											// clear transaction registers
	i2c_bytes = 0;
	i2c_reg_addr = 0;
	i2c_stop_flag = 0;
	i2c_slave_state = 0;
	
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

// error handler, should reset I2C slave internal state
// additional processing may be required, e.g. set flag in register array to enable master to read status

void I2C_error_handler()	
{
	i2c_reg_start = 0;											// clear transaction registers
	i2c_bytes = 0;
	i2c_reg_addr = 0;
	i2c_stop_flag = 0;
	i2c_slave_state = 0;
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
		i2c_stop_flag = 0;										// nothing to report so far 
		i2c_bytes = 0;											// reset transaction length
		num_bytes = 0;											// reset number of bytes for error recovery
		TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;	// send ACK 

		if (TWI0.SSTATUS & TWI_DIR_bm) {						// Master wishes to read from slave
//			_delay_us(5);										// TWI0.SDATA write needs minimum 5 us delay at 10 MHz clock	
			timeout_cnt = 0;									// reset timeout counter, will be incremented by ms tick interrupt
			while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {			// wait until Clock Hold flag set
				if (timeout_cnt > 2) return;					// return if timeout error
			}
			TWI0.SDATA = i2c_reg_array[i2c_reg_addr];			// Master read operation, return data from current register
			i2c_bytes++;
			i2c_reg_addr++;
			if (i2c_reg_addr > (SLAVE_REG_SIZE-1)) i2c_reg_addr = 0; // limit register address to valid range
			i2c_slave_state = 3;								// signal ongoing read transaction			
			TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
								
		} else {												// Master wishes to write to slave
			if (i2c_slave_state == 0) i2c_slave_state = 1;		// return and wait for register address	
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
				TWI0.SDATA = i2c_reg_array[i2c_reg_addr];		// Master read operation, return data from current register
				i2c_bytes++;
				i2c_reg_addr++;
				if (i2c_reg_addr > (SLAVE_REG_SIZE-1)) i2c_reg_addr = 0; // limit register address to valid range
				i2c_slave_state = 3;							// signal ongoing read transaction
				num_bytes++;									// increment number of bytes
				TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;							
			} else {											// Received NACK from master, last byte read
				TWI0.SSTATUS |= (TWI_DIF_bm | TWI_APIF_bm);		// Reset module
				TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;				
			}
			
		} else {												// Master wishes to write to slave
			if (i2c_slave_state == 1)							// waiting for register address
			{
				i2c_reg_addr = TWI0.SDATA;
				if (i2c_reg_addr > (SLAVE_REG_SIZE-1)) i2c_reg_addr = 0; // set to zero if invalid
				i2c_reg_start = i2c_reg_addr;					// save starting register address
				i2c_slave_state = 2;							// set state: register address received
			}
			else {												// data write transaction
				i2c_reg_array[i2c_reg_addr] = TWI0.SDATA;
				i2c_bytes++;
				i2c_reg_addr++;
				if (i2c_reg_addr > (SLAVE_REG_SIZE-1)) i2c_reg_addr = 0; // limit register address to valid range
				i2c_slave_state = 4;							// signal ongoing write transaction
			}
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
		i2c_stop_flag = 0xff;									// flag: stop received in invalid state
		if (i2c_slave_state == 2) i2c_stop_flag = 1;			// flag: write register address
		if (i2c_slave_state == 3) i2c_stop_flag = 2;			// flag: read
		if (i2c_slave_state == 4) i2c_stop_flag = 3;			// flag: write
		i2c_slave_state = 0;									// wait for Slave Address
		return;
	}
}

