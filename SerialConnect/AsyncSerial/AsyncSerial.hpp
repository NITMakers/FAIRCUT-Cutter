// -*- coding: utf-8 -*-
/**
 @file		AsyncSerial.hpp
 @brief	 	Asynchronous (Non-brocking) Serial Communication library with variable length software ring buffer (FIFO). You can use also RawSerial Library's method. You can set the baudrate of the serial communication when instantiating.
 
 @author	T.Kawamura
 @version	1.3
 @date		2017-03-29	T.Kawamura	Written for C++/mbed.
 @date		2017-03-30	T.Kawamura	Bug Fixed: Cannot use format(), baud().
 @date		2017-06-17	T.Kawamura	Update: FIFO Buffer Fixed.
 @date		2017-06-23	T.Kawamura	Bug Fixed: Cant use wait().
 @see 
 Copyright (C) 2017 T.Kawamura.
 Released under the MIT license.
 http://opensource.org/licenses/mit-license.php
 
*/

#ifndef ASYNCSERIAL_H
#define ASYNCSERIAL_H

#include "mbed.h"
#include "FIFO.hpp"
#include <stdarg.h>
#include <string.h>

using namespace std;

#define PRINTF_STRING_BUFFER_SIZE 256

/**
	@class	AsyncSerial
  @brief	Asynchronous Serial Communication with FIFO
*/
class AsyncSerial{
private:
    //DigitalOut *led;
    	
	// Serial port
	RawSerial *serial;
	
	// FIFO is uint8_t(unsigned char) buffer
	FIFO<uint8_t> *fifo_tx;
	FIFO<uint8_t> *fifo_rx;
	
	bool Is_Serial_Sending;
	
	void ISR_TX(void);
	void ISR_RX(void);

public:

	/**
		 @brief	Create a new AsyncSerial Port.
		 @param	txpin	Tx pin name (Defined in PinName.h)
		 @param	rxpin	Rx pin name (Defined in PinName.h)
		 @param	baudrate	Baudrate (ex: 115200). Default value is 9600.
		 @param	buffer_size	Buffer size. Default value is 256. (byte)
	*/
	AsyncSerial(PinName txpin, PinName rxpin, uint32_t baudrate=9600, uint32_t buffer_size=256);
	/**
		 @brief	Disable the AsyncSerial Port.
		 @param	No parameters.
	*/
	virtual ~AsyncSerial(void);

	/**
		 @brief	Get how many bytes are in the rx buffer.
		 @param	No parameters.
		 @return	Size of readable data. (byte)
	*/
	virtual int readable(void);

	/**
		 @brief	Check writeable or not.
		 @param	No Parameters.
		 @retval	1	Always return 1 because this library provides asynchronous serial.
	*/
	virtual int writeable(void);

	/**
		 @brief	Get 1byte from the AsyncSerial port.
		 @param	No parameters.
		 @retval	All Got Data
		 @retval	0	Error.
	*/
	virtual int getc(void);

	/**
		 @brief	Peek 1byte from the AsyncSerial port.
		 @param	No parameters.
		 @retval	ALL	Got Data
		 @retval	0	Error.
	*/
	virtual int peekc(void);

	/**
		 @brief	Put 1byte to the AsyncSerial port.
		 @param	data	A Data for put
		 @return		Nothing.
	*/
	virtual void putc(int c);

	/**
		 @brief	Write a string with new line. The string must be NULL terminated.
		 @param	*str	A String for write (Must be NULL terminated).
		 @return		Nothing.
	*/
	virtual void puts(const char *str);

	/**
		 @brief	Write a formatted string to the AsyncSerial port. 
		 @param	*format	A Formatted string for write.
		 @retval	0	Error.
		 @retval	1+	Wrote string size (byte).
	*/
	virtual int printf(const char *format, ...);

	/**
		 @brief	Write byte array to the AsyncSerial port.
		 @param	*s	A pointer to the array for write.
		 @param	length	Write size (byte).
		 @retval	0	Error.
		 @retval	1	Success.
	*/
	virtual int write(const uint8_t *buffer, int length);

	/**
		 @brief	Abort the on-going read transfer.
		 @param	No parameters.
	*/
	virtual void abort_read(void);
	
	/**
		 @brief	Abort the on-going write transfer.
		 @param	No parameters.
	*/
	virtual void abort_write(void);
	
	/**
		 @brief	Wait until finish all sending.
		 @param	No parameters.
	*/
	virtual void wait(void);

	/**
		 @brief	Set bits, parity and stop bits.
		 @param	bits	Bits (5 ~ 8)
		 @param	parity	Parity
		 @param	stop_bits	Stop bits (1 or 2)
	*/
	virtual void format(int bits=8, RawSerial::Parity parity=RawSerial::None, int stop_bits=1);

	/**
		 @brief	Set baud rate.
		 @param	baudrate	baudrate (bps).
	*/
	virtual void baud(int baudrate);
	
};

#endif

