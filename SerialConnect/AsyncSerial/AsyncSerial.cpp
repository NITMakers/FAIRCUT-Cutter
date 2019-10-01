// -*- coding: utf-8 -*-
/**
 @file		AsyncSerial.cpp
 @brief	 	Asynchronous (Non-brocking) Serial Communication library with variable length software ring buffer (FIFO). You can use also RawSerial Library's method. You can set the baudrate of the serial communication when instantiating.
 
 @author	T.Kawamura
 @version	1.3
 @date		2017-03-29	T.Kawamura	Written for C++/mbed.
 @date		2017-03-30	T.Kawamura	Bug Fixed: Cannot use format(), baud().
 @date		2017-06-17	T.Kawamura	Update: FIFO Buffer Fixed.
 
 
 @see 
 Copyright (C) 2017 T.Kawamura.
 Released under the MIT license.
 http://opensource.org/licenses/mit-license.php
 
*/

#include "AsyncSerial.hpp"

AsyncSerial::AsyncSerial(PinName txpin, PinName rxpin, uint32_t baudrate, uint32_t buffer_size){
	// RawSerial port init
	serial = new RawSerial(txpin, rxpin, baudrate);
	
	// FIFO init
	fifo_tx = new FIFO<uint8_t>(buffer_size);
	fifo_rx = new FIFO<uint8_t>(buffer_size);
		
	fifo_tx->clear();
	fifo_rx->clear();
	
	Is_Serial_Sending = false;
	
	//Initialize ISR
	serial->attach(callback(this, &AsyncSerial::ISR_TX), SerialBase::TxIrq);
	serial->attach(callback(this, &AsyncSerial::ISR_RX), SerialBase::RxIrq);
	
	return;
}

AsyncSerial::~AsyncSerial(){
	serial->attach(NULL, serial->TxIrq);
	serial->attach(NULL, serial->RxIrq);
    
	delete serial;
	return;
}

void AsyncSerial::ISR_TX(void){
	int data;
	
	if( fifo_tx->available() > 0 ){
		data = (int)fifo_tx->get();
		serial->putc(data);
	}else{
		Is_Serial_Sending = false;
	}
}

void AsyncSerial::ISR_RX(void){
	uint8_t data;

	data = (uint8_t)serial->getc();
	fifo_rx->put(data);
}

int AsyncSerial::readable(void){
	return (int)fifo_rx->available();
}

int AsyncSerial::writeable(void){
	return 1;
}

int AsyncSerial::getc(void){
	return (int)fifo_rx->get();
}

int AsyncSerial::peekc(void){
	return (int)fifo_rx->peek();
}

void AsyncSerial::putc(int c){
	if( Is_Serial_Sending ){
		fifo_tx->put((uint8_t)c);
	}else{
		Is_Serial_Sending = true;
		serial->putc(c);
	}
	return;
}

void AsyncSerial::puts(const char *str){
	uint8_t temp;

	for(uint32_t i = 0; i < strlen(str); i++){
		temp = (uint8_t)str[i];
		fifo_tx->put(temp);
	}

	if( !Is_Serial_Sending ){
		Is_Serial_Sending = true;
		serial->putc((int)fifo_tx->get());
	}
	
	AsyncSerial::putc('\r');
	AsyncSerial::putc('\n');
	return;
}

int AsyncSerial::printf(const char *format, ...){
	int32_t wrote_length = 0;
	char string_buffer[PRINTF_STRING_BUFFER_SIZE];

	memset(string_buffer, 0, PRINTF_STRING_BUFFER_SIZE);
	
	va_list arg;
	va_start(arg, format);
	wrote_length = vsprintf(string_buffer, format, arg);
	
	if( wrote_length > PRINTF_STRING_BUFFER_SIZE ) {
		error("%s @ %d : String is too large, string buffer overwrite. (Max buffer size: %d Wrote length: %d)\n", __FILE__, __LINE__, PRINTF_STRING_BUFFER_SIZE, wrote_length);
		va_end(arg);
		return 0;
	}

	if( wrote_length < 0 ){
		va_end(arg);
		error("Function vsprintf() was failed.");
		return 0;
	}

	va_end(arg);
	AsyncSerial::write((uint8_t*)string_buffer, wrote_length);
	
	return wrote_length;
}

int AsyncSerial::write(const uint8_t *buffer, int length){
	uint8_t temp;
	
	if ( length < 1 ){
		return 0;
	}

	for(uint32_t i = 0; i < length; i++){
		temp = (uint8_t)buffer[i];
		fifo_tx->put(temp);
	}

	if( !Is_Serial_Sending ){
		Is_Serial_Sending = true;
		serial->putc((int)fifo_tx->get());
	}

	return 1;
}

void AsyncSerial::abort_read(void){
	fifo_rx->clear();
	return;
}

void AsyncSerial::abort_write(void){
	fifo_tx->clear();
	return;
}

void AsyncSerial::wait(void){
	while( fifo_tx->available() > 0 ){}
	return;
}

void AsyncSerial::format(int bits, RawSerial::Parity parity, int stop_bits){
	serial->attach(NULL, serial->TxIrq);
	serial->attach(NULL, serial->RxIrq);

	serial->format(bits, parity, stop_bits);

	serial->attach(callback(this, &AsyncSerial::ISR_TX), serial->TxIrq);
	serial->attach(callback(this, &AsyncSerial::ISR_RX), serial->RxIrq);
}

void AsyncSerial::baud(int baudrate){
	serial->attach(NULL, serial->TxIrq);
	serial->attach(NULL, serial->RxIrq);

	serial->baud(baudrate);

	serial->attach(callback(this, &AsyncSerial::ISR_TX), serial->TxIrq);
	serial->attach(callback(this, &AsyncSerial::ISR_RX), serial->RxIrq);	
}

