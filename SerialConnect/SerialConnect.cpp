// -*- coding:utf-8-unix -*-
/*!
  @file   SerialConnect.cpp
  @author Takuma Kawamura <bitpositive@MacBook-Pro-13.local>
  @date   Wed Jan 23 13:40:32 2019
  
  @brief  Nerve-SerialConnect

  MbedのUARTポートを使って，uint8_t(unsigned char)型の配列を送受信するためのライブラリ．割り込み処理を使うため効率的．チェックサムやバイト間タイムアウトを搭載しているので安全かつ，完全な状態で通信を行うことができる．

  @version    1.0

  @see      Mbed OS 2のリビジョンは，必ずRev.146以下にすること．

  @copyright  T.Kawamura (C) 2019 <contact@bit-plus.work>
  
*/

#include "SerialConnect.hpp"


SerialConnect::SerialConnect( PinName txpin, PinName rxpin, uint32_t payload_size, uint32_t baudrate, uint32_t uart_fifo_buffer_size )
	: payload_size( payload_size ), load_status( STATE_WAIT_HEADER ), payload_data_count( 0 )
{
  serial_port = new AsyncSerial( txpin, rxpin, baudrate, uart_fifo_buffer_size );
	
	time_out_timer = new Timer;
	time_out_timer->stop();
	time_out_timer->reset();
	
	memset( recieved_payload, 0, NSC_MAX_PACKET_SIZE );
}

SerialConnect::~SerialConnect( void )
{	
	delete serial_port;
  delete time_out_timer;
}

void SerialConnect::send( const uint8_t *payload_array )
{
	uint8_t checksum = 0;
	
	send_buffer[0] = NSC_HEADER;
	send_buffer[1] = payload_size;
	for( int i = 0; i < payload_size; i++ ) {
		send_buffer[i + 2] = payload_array[i];
		checksum += payload_array[i];
	}
	send_buffer[payload_size + 2] = checksum;
	send_buffer[payload_size + 3] = NSC_FOOTER;
	
	serial_port->write( send_buffer, payload_size + 4 );
	
	return;
}

update_result_e SerialConnect::update( void )
{
	uint8_t data;
	uint32_t i;
	uint8_t checksum = 0;
	
	if( serial_port->readable() > 0 ) {
		
		if( time_out_timer->read_ms() > NSC_TIMEOUT && load_status != STATE_WAIT_HEADER ){
			load_status = STATE_WAIT_HEADER;
		}
		time_out_timer->stop();
		time_out_timer->reset();
		
		data = serial_port->getc(); // 1byte取得
		
		switch( load_status ) {
		case STATE_WAIT_HEADER:
			if( data == NSC_HEADER ) {	// '@'
				load_status = STATE_WAIT_LENGTH;    
			}
			break;
			
		case STATE_WAIT_LENGTH:
			if( data == payload_size ) {
				load_status = STATE_WAIT_PAYLOAD;
			} else {
				load_status = STATE_WAIT_HEADER;
			}
			break;

		case STATE_WAIT_PAYLOAD:
			if( payload_data_count < payload_size - 1 ) {
				recv_buffer[payload_data_count] = data;
				payload_data_count++;
			} else {
				recv_buffer[payload_data_count] = data;
				payload_data_count = 0;
				load_status = STATE_WAIT_CHECKSUM;
			}
			break;

		case STATE_WAIT_CHECKSUM:
			for( i = 0; i < payload_size; i++ ) {
				checksum += recv_buffer[i];
			}
                
			if( data == checksum ) {
				load_status = STATE_WAIT_FOOTER;
			} else {
				load_status = STATE_WAIT_HEADER;
			}
			break;
			
		case STATE_WAIT_FOOTER:
			if( data == NSC_FOOTER ) {		// LF
				for( i = 0; i < payload_size; i++ ) {
					recieved_payload[i] = recv_buffer[i];
				}
				load_status = STATE_WAIT_HEADER;
				
				return READY;
			} else {
				load_status = STATE_WAIT_HEADER;
			}
			
			break;
		}
	}
	
	time_out_timer->start();
	
	return NOT_READY;
}
