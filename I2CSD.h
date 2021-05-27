/*
 * I2CSD.h
 *
 * I2C Device Slave
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

// I2C address space:
// 0100 XXXr is reserved for video encoders, GPIO expanders, should be applicable fro most cases
// 1010 XXXr memory, real-time clock

#ifndef I2CS_H_
#define I2CS_H_

#define SLAVE_ADDRESS	8		// 8 bit Slave Address, i.e. bit 0 = 0, will be substituted by R/W bit
#define SLAVE_REG_SIZE	8		// byte register array size

#define MAX_TRANSACTION 9		// maximum number of bytes sent/received in single transaction
								// not used for normal transactions, only used for error recovery

extern volatile uint8_t	i2c_reg_array[SLAVE_REG_SIZE];	// register array
extern volatile uint8_t	i2c_stop_flag;				// 1: register address write performed,
													// 2: master read performed, 3: master write performed,
													// 0xff: stop received in invalid state
extern uint8_t i2c_reg_start;						// first register address of current transaction
extern uint8_t i2c_bytes;							// number of bytes sent/received during transaction

//#define ALTERNATE_PINOUT		// use alternate pins
								// standard pinout	alternate pinout
								// SCL = PB0		SCL = PA2
								// SDA = PB1		SDA = PA1

void		I2CSD_init(void);	// initialize I2C slave device

#endif /* I2CS_H_ */