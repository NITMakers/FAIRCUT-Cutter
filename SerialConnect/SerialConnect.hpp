// -*- coding:utf-8-unix -*-
/*!
  @file   SerialConnect.hpp
  @author Takuma Kawamura <bitpositive@MacBook-Pro-13.local>
  @date   Wed Jan 23 13:40:32 2019
  
  @brief  Nerve-SerialConnect

  MbedのUARTポートを使って，uint8_t(unsigned char)型の配列を送受信するためのライブラリ．割り込み処理を使うため効率的．チェックサムやバイト間タイムアウトを搭載しているので安全かつ，完全な状態で通信を行うことができる．

  @version    1.0

  @see      Mbed OS 2のリビジョンは，必ずRev.146以下にすること．

  @see     このソースコードのコメントにはDoxygen方式を使っています．Doxygenを使ってコンパイルすることで，html形式のAPIドキュメントを生成できます．

  @copyright  T.Kawamura (C) 2019 <contact@bit-plus.work>
  
*/

#ifndef NSC_SENDRECV_H
#define NSC_SENDRECV_H

// Includes ////////////////////////////////////////////////////////////////////
#include "mbed.h"   // mbed OS 2.0 Rev 146
#include <cstring>
// mbed のサイトからAsyncSerialをインポートすること
#include "AsyncSerial.hpp"

using namespace std;

// Definitions ////////////////////////////////////////////////////////////////
#define NSC_HEADER 0x40
#define NSC_FOOTER 0x0A
#define NSC_TIMEOUT 1000

#ifndef NSC_MAX_PACKET_SIZE
#define NSC_MAX_PACKET_SIZE 128
#endif

// 受信状態
enum loadstate_e{
  STATE_WAIT_HEADER,
  STATE_WAIT_LENGTH,
  STATE_WAIT_PAYLOAD,
  STATE_WAIT_CHECKSUM,
  STATE_WAIT_FOOTER
};
// 受信結果取得用
enum update_result_e{
  NOT_READY,
  READY
};

// Class //////////////////////////////////////////////////////////////////////
/*!
	@class	SerialConnect
	@brief	NSC形式パケットの送受信
*/
class SerialConnect {
private:
	
	// シリアルポート
	AsyncSerial *serial_port;
  // バイト間タイムアウト用タイマ
	Timer *time_out_timer;
	// 送信パケットのバッファ
	uint8_t send_buffer[NSC_MAX_PACKET_SIZE];
	// ペイロードの大きさ(Byte)
	int payload_size;
	// 状態遷移用
  loadstate_e load_status;
	// パケットの受信処理で使用するバッファ
  uint8_t recv_buffer[NSC_MAX_PACKET_SIZE];
	// ペイロードの受信完了データの大きさ
  uint8_t payload_data_count;
	// ペイロードの格納場所
  uint8_t recieved_payload[NSC_MAX_PACKET_SIZE];


public:
	/*! 
		@brief	コンストラクタ
    片方向通信でTx, Rxのどちらかのピンを使わない場合は，ピン番号を"NC"とすること．
		
		@param	txpin									パケットを送信するピン(PinNames.hで定義されたもの)
    @param	rxpin									パケットを受信するピン(PinNames.hで定義されたもの)
    @param	payload_size				  やりとりするパケットのペイロードの大きさ(Byte)
		@param	baudrate							ボーレート(省略可，デフォルトは115200bps)
		@param	uart_fifo_buffer_size	UARTのリングバッファのサイズ(省略可，デフォルトは256Byte)
	*/
	SerialConnect(PinName txpin, PinName rxpin, uint32_t payload_size, uint32_t baudrate=115200, uint32_t uart_fifo_buffer_size=256);

	/*! 
		@brief	デストラクタ
	*/
	virtual ~SerialConnect( void );

	/*! 
		@brief	パケットの送信
		
		@param	uint8_t* payload_array	送信したい配列のポインタ
	*/
	virtual void send( const uint8_t *payload_array );

	/*! 
		@brief	パケット受信処理を行う
		
		メインループ内で毎回必ずこのメソッドを呼び、返り値を見てOKならペイロードの格納が完了しています。
    
		@retval	update_result_e	READY			ペイロードの正常な受信および格納が完了
		@retval	update_result_e NOT_READY	ペイロードの受信処理が続行中
	*/
  virtual update_result_e update( void );
	
	/*! 
		@brief	ペイロードのGetter
		
		格納済みのペイロードを読み出します。
		配列の要素取得と同じように、インデックスを引数として渡します。
    
		注意： 必ずupdateメソッドでペイロードの受信完了を確認し，その後実行してください
		
		@param	index	要素のインデックス
		
		@return uint8_t	要素
	*/
  virtual uint8_t get_payload_by_1byte( uint8_t index ) { return recieved_payload[index]; }
	
};


#endif  // NSC_SENDRECV_H
