/* Copyright (c) 2014 Yajirushi(Cursor)
 *
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "mbed.h"

#ifndef L6470SDC_H
#define L6470SDC_H

/* ------------------------------------------------------------------
The code below, please be rewritten to match your environment
------------------------------------------------------------------ */

//If not using Serial-print debugging: DEBUG_L6470SDC to comment out
//If Serial-print debugging, must be connected "USBTX"(tx), "USBRX"(rx).
//#define DEBUG_L6470SDC

//SPI signal frequency: Less than 5MHz(5,000,000)
#define L6470_SPI_FREQ 4960000

/* ------------------------------------------------------------------
No need to change the below code
------------------------------------------------------------------ */
#define CMD_NOP 0x0
#define CMD_SETPARAM 0x00
#define CMD_GETPARAM 0x20
#define CMD_RUN_PLUS 0x51
#define CMD_RUN_MINUS 0x50
#define CMD_STEP_PLUS 0x59
#define CMD_STEP_MINUS 0x58
#define CMD_ADDSTEP_PLUS 0x41
#define CMD_ADDSTEP_MINUS 0x40
#define CMD_GOTO 0x60
#define CMD_GOTO_DIR_PLUS 0x69
#define CMD_GOTO_DIR_MINUS 0x68
#define CMD_GO_UNTIL_PLUS 0x83
#define CMD_GO_UNTIL_MINUS 0x82
#define CMD_RELEASE_SW_PLUS 0x93
#define CMD_RELEASE_SW_MINUS 0x92
#define CMD_GO_HOME 0x70
#define CMD_GO_MARK 0x78
#define CMD_RESET_POS 0xD8
#define CMD_RESET_DEVICE 0xC0
#define CMD_SOFT_STOP 0xB0
#define CMD_HARD_STOP 0xB8
#define CMD_SOFT_HIZ 0xA0
#define CMD_HARD_HIZ 0xA8
#define CMD_GET_STATUS 0xD0

#define REG_NOTHING 0x0
#define REG_ABS_POS 0x01
#define REG_EL_POS 0x02
#define REG_MARK 0x03
#define REG_SPEED 0x04
#define REG_ACC 0x05
#define REG_DEC 0x06
#define REG_MAX_SPEED 0x07
#define REG_MIN_SPEED 0x08
#define REG_KVAL_HOLD 0x09
#define REG_KVAL_RUN 0x0A
#define REG_KVAL_ACC 0x0B
#define REG_KVAL_DEC 0x0C
#define REG_INT_SPD 0x0D
#define REG_ST_SLP 0x0E
#define REG_FN_SLP_ACC 0x0F
#define REG_FN_SLP_DEC 0x10
#define REG_K_THERM 0x11
#define REG_ADC_OUT 0x12
#define REG_OCD_TH 0x13
#define REG_STALL_TH 0x14
#define REG_FS_SPD 0x15
#define REG_STEP_MODE 0x16
#define REG_ALARM_EN 0x17
#define REG_CONFIG 0x18
#define REG_STATUS 0x19

//DaisyChain CommandQueue Structure
typedef struct L6470CommandQueue{
    unsigned char addr;
    unsigned long val;
    int bitLen;
}L6470CMDQ;

/**
 * @~japanese
 * STマイクロエレクトロニクス社製のL6470ステッピングモーターコントローラー用ライブラリ
 * 真の意味でデイジーチェーンをサポートしています(もちろん個別結線でも使用可能です)
 *
 * @~english
 * Library for STMicroelectronics "L6470" stepper motor controller.
 * This library supported "SPI Daisy-Chain".
 *
 * Example:
 * @code
 * //case: Using "Daisy-Chained" 2 motors. not use Serial-Debugging.
 * #include "mbed.h"
 * #include "L6470SDC.h"
 *
 * int main(){
 *     //create instance(Not use Serial-Debugging)
 *     L6470SDC l6470(SPI_MOSI, SPI_MISO, SPI_SCK, D10);
 *
 *     wait(1);
 *
 *     //L6470 initialize
 *     l6470.init();
 *
 *     wait(5);
 *
 *     //Set the rotation speed of the motor to 100step/s. (Daisy-chained motor No.1)
 *     l6470.setMaximumSpeed(1, l6470.calcMaxSpd(100));
 *     //Set the rotation speed of the motor to 200step/s. (Daisy-chained motor No.2)
 *     l6470.setMaximumSpeed(2, l6470.calcMaxSpd(200));
 *
 *     //Running motor 5seconds.
 *     //Maximum speed = 300step/s
 *     //!!Start timing deviate, because it does not use the "Queue"
 *     l6470.run(1, l6470.calcSpd(300), true); //clockwise No.1
 *     l6470.run(2, l6470.calcSpd(300), false); //counter-clockwise No.2
 *     wait(5.0);
 *     l6470.stop(1);
 *     l6470.stop(2);
 *
 *     //Waiting... until motor "absolute stop".
 *     while(l6470.isBusy(1) || l6470.isBusy(2));
 *
 *     wait(3.0);
 *
 *     //Enqueue: Motor No.1 -> 7 rev rotation.(200step/rev * microstep-count * rev count), clockwise.
 *     l6470.ENQ_move(1, 200*128*7, true);
 *
 *     //Enqueue: Motor No.2 -> 7 rev rotation.(200step/rev * microstep-count * rev count), clockwise.
 *     l6470.ENQ_move(2, 200*128*7, true);
 *
 *     //Execute Queue
 *     l6470.Qexec();
 *
 *     //Waiting... until motor "absolute stop".
 *     while(l6470.isBusy(1) || l6470.isBusy(2));
 *
 *     //Set the rotation speed of the motor No.2 to motor No.1.
 *     l6470.setMaximumSpeed(1, l6470.getMaximumSpeed(2));
 *
 *     //Back to HOME_POS.("home" is aliase "zero")
 *     l6470.home(1);
 *     l6470.zero(2);
 * }
 * @endcode
 */
class L6470SDC{
public:
    /** Constructor:(Serial, SPI, CS)@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param tx シリアル通信で使用するTXピン(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param rx シリアル通信で使用するRXピン(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param csel SPIのChipSelectピン
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param tx Serial USB_TX PinName (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param rx Serial USB_RX PinName (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param csel SPI ChipSelect PinName
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * //ChipSelect -> select pin as you like.
    * L6470SDC l6470(USBTX, USBRX, p5, p6, p7, p8);
    * @endcode
    */
    L6470SDC(PinName tx, PinName rx, PinName mosi, PinName miso, PinName sclk, PinName csel);

    /** Constructor:(Serial, SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param tx シリアル通信で使用するTXピン(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param rx シリアル通信で使用するRXピン(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param tx Serial USB_TX PinName (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param rx Serial USB_RX PinName (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    *
    * //ChipSelect -> select pin as you like.
    * DigitalOut l6470cs(p8);
    *
    * L6470SDC l6470(USBTX, USBRX, p5, p6, p7, &l6470cs );
    * @endcode
    */
    L6470SDC(PinName tx, PinName rx, PinName mosi, PinName miso, PinName sclk, DigitalOut *csel);

    /** Constructor:(Serial, SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param *serial USBシリアルのアドレス(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param *serial USB Serial Addr (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    *
    * //USB Serial
    * Serial l6470Serial(USBTX, USBRX);
    *
    * L6470SDC l6470( &l6470Serial , p5, p6, p7, p8);
    * @endcode
    */
    L6470SDC(Serial *serial, PinName mosi, PinName miso, PinName sclk, PinName csel);

    /** Constructor:(Serial, SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param *serial USBシリアルのアドレス(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param *serial USB Serial Addr (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    *
    * //USB Serial
    * Serial l6470Serial(USBTX, USBRX);
    *
    * //ChipSelect pin
    * DigitalOut l6470cs(p8);
    *
    * L6470SDC l6470( &l6470Serial , p5, p6, p7, &l6470cs );
    * @endcode
    */
    L6470SDC(Serial *serial, PinName mosi, PinName miso, PinName sclk, DigitalOut *csel);

    /** Constructor:(Serial, SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param *serial USBシリアルのアドレス(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param *spi SPIのアドレス
    * @param csel SPIのChipSelectピン
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param *serial USB Serial Addr (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param *spi SPI Addr
    * @param csel SPI ChipSelect PinName
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * SPI l6470spi(p5, p6, p7);
    *
    * //USB Serial
    * Serial l6470Serial(USBTX, USBRX);
    *
    * L6470SDC l6470( &l6470Serial , &l6470spi , p8);
    * @endcode
    */
    L6470SDC(Serial *serial, SPI *spi, PinName csel);

    /** Constructor:(Serial, SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。シリアル通信デバッグを使用する。
    * @param *serial USBシリアルのアドレス(シリアルデバッグを使用する際は、#define DEBUG_L6470SDCのコメントアウトを解除してください)
    * @param *spi SPIのアドレス
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor. use Serial-Debugging.
    * @param *serial USB Serial Addr (If using Serial-debugging. must be #define DEBUG_L6470SDC)
    * @param *spi SPI Addr
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * SPI l6470spi(p5, p6, p7);
    *
    * //USB Serial
    * Serial l6470Serial(USBTX, USBRX);
    *
    * //ChipSelect pin
    * DigitalOut l6470cs(p8);
    *
    * L6470SDC l6470( &l6470Serial , &l6470spi , &l6470cs );
    * @endcode
    */
    L6470SDC(Serial *serial, SPI *spi, DigitalOut *csel);

    /** Constructor:(SPI, CS) overload@n
    * @~japanese
    * コンストラクタ。
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param csel SPIのChipSelectピン
    *
    * @~english
    * Constructor.
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param csel SPI ChipSelect PinName
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * //ChipSelect -> select pin as you like.
    * L6470SDC l6470(p5, p6, p7, p8);
    * @endcode
    */
    L6470SDC(PinName mosi, PinName miso, PinName sclk, PinName csel);

    /** Constructor:(SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。
    * @param mosi SPIのmosiピン
    * @param miso SPIのmisoピン
    * @param sclk SPIのsclkピン
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor.
    * @param mosi SPI Master-out PinName
    * @param miso SPI Slave-out PinName
    * @param sclk SPI Clock PinName
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    *
    * //ChipSelect -> select pin as you like.
    * DigitalOut l6470cs(p8);
    *
    * L6470SDC l6470(p5, p6, p7, &l6470cs );
    * @endcode
    */
    L6470SDC(PinName mosi, PinName miso, PinName sclk, DigitalOut *csel);

    /** Constructor:(SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。
    * @param *spi SPIのアドレス
    * @param csel SPIのChipSelectピン
    *
    * @~english
    * Constructor.
    * @param *spi SPI Addr
    * @param csel SPI ChipSelect PinName
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * SPI l6470spi(p5, p6, p7);
    *
    * L6470SDC l6470( &l6470spi , p8);
    * @endcode
    */
    L6470SDC(SPI *spi, PinName csel);

    /** Constructor:(SPI, CS) overlaod@n
    * @~japanese
    * コンストラクタ。
    * @param *spi SPIのアドレス
    * @param *csel SPIのChipSelectピンのアドレス
    *
    * @~english
    * Constructor.
    * @param *spi SPI Addr
    * @param *csel SPI ChipSelect Pin Addr
    *
    * Example:
    * @code
    * //create instance(case:NXP mbed LPC1768)
    * //hardware-SPI -> MOSI:p5, MISO:p6, SCK:p7
    * SPI l6470spi(p5, p6, p7);
    *
    * //ChipSelect pin
    * DigitalOut l6470cs(p8);
    *
    * L6470SDC l6470( &l6470Serial , &l6470spi , &l6470cs );
    * @endcode
    */
    L6470SDC(SPI *spi, DigitalOut *csel);
    
    /** Destructor@n
     * @~japanese デストラクター(ユーザーが直接呼ぶ必要はありません)
     *
     * @~english Destructor. may be not called by user.
     */
    ~L6470SDC();

private:
    Serial *pc;
    SPI *spi;
    DigitalOut *cs;

    bool hasSerial;
    int motor_count;
    
    L6470CMDQ *Queue; //ADD

    void setCmd(int motorNumber, unsigned char cmdAddress, unsigned long value, int bitLen);
    void setParam(int motorNumber, unsigned char regAddress, unsigned long value, int bitLen);
    unsigned long getParam(int motorNumber, unsigned char regAddress, int bitLen);
    void sendCMD(unsigned char cmd);

public:
    /** Initialize L6470@n
    * @~japanese
    * L6470を初期化します
    * @param initSPI SPIに関するパラメーター(クロックやモード)を初期化しない場合はfalseに(デフォルトはtrue)
    *
    * @~english
    * @param initSPI If initialize SPI parameters.(e.g. clockspeed and mode) = true(default)
    */
    void init(bool initSPI=true);

    /** Get connected(daisy-chained) motor count.@n
    * @~japanese
    * デイジーチェーン接続されたモーターの数を取得します
    * @returns デイジーチェーン接続されたモーター(ドライバー)の個数
    *
    * @~english
    * @returns Connected(daisy-chained) motor count. If not using daisy-chain, return always 1. maybe...
    */
    int getMotorCount();

    /** Get motor Busy-flag status@n
    * @~japanese
    * モーター動作中フラグBUSYの状態を取得します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @returns 指定したモーター番号のBUSYフラグ(false = Busy-flag 0, true = Busy-flag 1)
    *
    * @~english
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @returns Busy status.(false = Busy-flag 0, true = Busy-flag 1)
    */
    bool isBusy(int motorNumber);

    /** Run motor rotate@n
    * @~japanese
    * モーターを指定した速度で回転させます
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param hex_speed 16進数指定でのモーターの回転速度(step毎秒) 。関数 "calcSpd(stepPerSecond)" を使用して16進数換算の値を求めてください。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * At the specified speed, run the motor rotation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param hex_speed Motor rotate speed(hex_value Step/s). -> please use "calcSpd(stepPerSecond)" method.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    *
    * Example:
    * @code
    *     unsigned long calculatedValue = l6470.calcSpd(200); //Calculated "hex value" from "real value".
    *     l6470.run(1, calculatedValue, true); //200step/s, clockwise
    * @endcode
    */
    void run(int motorNumber, unsigned long hex_speed, bool isClockwise);

    /** Run motor steps@n
    * @~japanese
    * 指定したステップの数だけモーターを回します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param count 回転させるステップ数。マイクロステップモードを使用している場合は、その値を掛けなければなりません。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * At the specified step count, run the motor rotation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param count Steps count. If using Microstep-mode, "count" must be multiplied by Microstep-value.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    *
    * Example:
    * @code
    *     l6470.stop(1);
    *     l6470.setStepMode(1, 0x07); //set microstep-mode 0x07 = 1/128 microstep.
    *     l6470.step(1, 256, false); //(256/128)=2 step, counter cloclwise.
    * @endcode
    */
    void step(int motorNumber, unsigned int count, bool isClockwise);

    /** Run motor move@n
    * @~japanese
    * 指定したステップの数だけモーターを回します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param stepsCount 回転させるステップ数。マイクロステップモードを使用している場合は、その値を掛けなければなりません。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    * ステップ数が5未満 の場合 -> 関数 "step(int motorNumber, unsigned int count, bool isClockwise)" を使用
    * ステップ数が5以上 の場合 -> この関数を使用したほうがいい(ズレが生じるため)
    *
    * @~english
    * At the specified step count, run the motor rotation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param stepsCount Steps count. If using Microstep-mode, "stepsCount" must be multiplied by Microstep-value.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    * If less than 5step -> Please use the "step(int motorNumber, unsigned int count, bool isClockwise)".
    * If greater than 5step -> Please use this method.
    */
    void move(int motorNumber, unsigned long stepsCount, bool isClockwise);

    /** Go to motor ABS position (shortest path)@n
    * @~japanese
    * 最短パスで指定した座標まで回転します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param ABSPos 停止させる絶対座標
    * 注意: 絶対座標への回転方向は不定です。移動距離が短いほうに回転します。
    *
    * @~english
    * Motor rotation to the specified position in the shortest path.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param ABSPos Motor Absolute position.
    * CAUTION: This method a motion to ABS position through the shortest path.　Rotate direction is not constant.
    */
    void goto1(int motorNumber, unsigned long ABSPos);

    /** Go to motor ABS position (Specify the rotate direction)@n
    * @~japanese
    * 指定した座標まで回転します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param ABSPos 停止させる絶対座標
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * Motor rotation to the specified position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param ABSPos Motor Absolute position.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    */
    void goto2(int motorNumber, unsigned long ABSPos, bool isClockwise);

    /** ???Run motor rotate. Until external switch status change???
    * @~japanese
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param hex_speed 16進数指定でのモーターの回転速度(step毎秒) 。関数 "calcSpd(stepPerSecond)" を使用して16進数換算の値を求めてください。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    * @param setMark 外部スイッチのステータス？ ごめん。よくわかんないからデータシート見てね。
    *
    * @~english
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param hex_speed Motor rotate speed(hex_value Step/s). -> please use "calcSpd(stepPerSecond)" method.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    * @param setMark External switch status?...??.. Sorry... I could not understand. Please see L6470 datasheet.
    */
    void goUntil(int motorNumber, unsigned long hex_speed, bool isClockwise, bool setMark);

    /** ???Release external switch???
    * @~japanese
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    * @param setMark 外部スイッチのステータス？ ごめん。よくわかんないからデータシート見てね。
    *
    * @~english
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    * @param setMark External switch status?...??.. Sorry... I could not understand. Please see L6470 datasheet.
    */
    void releaseSwitch(int motorNumber, bool isClockwise, bool setMark);

    /** Go to motor home-position@n
    * @~japanese
    * モーターを原点復帰させます
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Return the motor to the zero-potition.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void home(int motorNumber);

    /** Go to motor home-position [alias: home(int motorNumber)]@n
    * @~japanese
    * モーターを原点復帰させます
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Return the motor to the zero-potition.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void zero(int motorNumber);

    /** Go to motor marked-position@n
    * @~japanese
    * 指定されたマーク位置まで回転します
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * マーク位置: -> 関数 "setMarkPosition(int motorNumber, unsigned long value)" を使用してください
    *
    * @~english
    * Motor rotation to the specified MARK Position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * MARK Position: -> Please use setMarkPosition(int motorNumber, unsigned long value).
    */
    void gotoMark(int motorNumber);

    /** Reset the ABS_POS register to zero (ABS_POS zero = home-position)@n
    * @~japanese
    * 現在位置を原点に設定します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Set the zero-position to current position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void resetHome(int motorNumber);

    /** Reset the ABS_POS register to zero (ABS_POS zero = home-position) [alias: resetHome(int motorNumber)]@n
    * @~japanese
    * キューに追加:現在位置を原点に設定します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Set the zero-position to current position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void resetZero(int motorNumber);

    /** Reset L6470.(software reset)@n
    * @~japanese
    * デバイスをリセットします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Reset L6470 driver & motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void motorReset(int motorNumber);

    /** Stop rotation (soft-stop)@n
    * @~japanese
    * モーターを停止
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Stop motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void stop(int motorNumber);

    /** Stop rotation. Ignore deceleration (hard-stop)@n
    * @~japanese
    * 減速を無視して停止
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Stop immediately motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void stopImmidiate(int motorNumber);

    /** Stop rotation (soft-stop) state sets HighImpedance.@n
    * @~japanese
    * モーターを停止し、実行後ハイインピーダンス状態にします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Stop motor. Set state "High Impedance" after stop.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void stop_HighImpedance(int motorNumber);

    /** Stop rotation. Ignore deceleration (hard-stop) state sets HighImpedance.@n
    * @~japanese
    * 減速を無視して停止し、実行後ハイインピーダンス状態にします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Stop immediately motor. Set state "High Impedance" after stop.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    void stopImmidiate_HighImpedance(int motorNumber);

    //Daisy-chain Queue Method -----------------------------------------------------
    /** Enqueue Command [NOP]@n
    * @~japanese
    * キューに追加:何も実行しない
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:No operation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_nop(int motorNumber);

    /** Enqueue Command [run]@n
    * @~japanese
    * キューに追加:モーターを指定した速度で回転させます
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param hex_speed 16進数指定でのモーターの回転速度(step毎秒) 。関数 "calcSpd(stepPerSecond)" を使用して16進数換算の値を求めてください。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * Add Queue:At the specified speed, run the motor rotation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param hex_speed Motor rotate speed(hex_value Step/s). -> please use "calcSpd(stepPerSecond)" method.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    *
    * Example:
    * @code
    *     unsigned long calculatedValue = l6470.calcSpd(200); //Calculated "hex value" from "real value".
    *     l6470.ENQ_run(1, calculatedValue, true); //200step/s, clockwise
    *     l6470.ENQ_NOP(2);
    *     l6470.Qexec();
    * @endcode
    */
    int ENQ_run(int motorNumber, unsigned long hex_speed, bool isClockwise);

    /** Enqueue Command [move]@n
    * @~japanese
    * キューに追加:指定したステップの数だけモーターを回します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param stepsCount 回転させるステップ数。マイクロステップモードを使用している場合は、その値を掛けなければなりません。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * Add Queue:At the specified step count, run the motor rotation.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param stepsCount Steps count. If using Microstep-mode, "stepsCount" must be multiplied by Microstep-value.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    *
    * Example:
    * @code
    *     l6470.stop(1);
    *     l6470.setStepMode(1, 0x07); //set microstep-mode 0x07 = 1/128 microstep.
    *     l6470.step(1, 256, false); //(256/128)=2 step, counter cloclwise.
    *     l6470.step(2, 1024, true); //(1024/128)=8 step, clockwise.
    *     l6470.Qexec();
    * @endcode
    */
    int ENQ_move(int motorNumber, unsigned long stepsCount, bool isClockwise);

    /** Enqueue Command [Goto] (shortest path)@n
    * @~japanese
    * キューに追加:最短パスで指定した座標まで回転します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param ABSPos 停止させる絶対座標
    * 注意: 絶対座標への回転方向は不定です。移動距離が短いほうに回転します。
    *
    * @~english
    * Add Queue:Motor rotation to the specified position in the shortest path.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param ABSPos Motor Absolute position.
    * CAUTION: This method a motion to ABS position through the shortest path.　Rotate direction is not constant.
    */
    int ENQ_goto1(int motorNumber, unsigned long ABSPos);

    /** Enqueue Command [Goto]@n
    * @~japanese
    * キューに追加:最短パスで指定した座標まで回転します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param ABSPos 停止させる絶対座標
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    *
    * @~english
    * Add Queue:Motor rotation to the specified position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param ABSPos Motor Absolute position.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    */
    int ENQ_goto2(int motorNumber, unsigned long ABSPos, bool isClockwise);

    /** Enqueue Command [???go Until]@n
    * @~japanese
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param hex_speed 16進数指定でのモーターの回転速度(step毎秒) 。関数 "calcSpd(stepPerSecond)" を使用して16進数換算の値を求めてください。
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    * @param setMark 外部スイッチのステータス？ ごめん。よくわかんないからデータシート見てね。
    *
    * @~english
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param hex_speed Motor rotate speed(hex_value Step/s). -> please use "calcSpd(stepPerSecond)" method.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    * @param setMark External switch status?...??.. Sorry... I could not understand. Please see L6470 datasheet.
    */
    int ENQ_goUntil(int motorNumber, unsigned long hex_speed, bool isClockwise, bool setMark);

    /** Enqueue Command [???Release external switch]@n
    * @~japanese
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * @param isClockwise 回転方向。true = 時計まわり. false = 反時計回り。(メーカーによって回転方向が違うかも)
    * @param setMark 外部スイッチのステータス？ ごめん。よくわかんないからデータシート見てね。
    *
    * @~english
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * @param isClockwise Rotate direction. true = clockwise. false = counter-clockwise.
    * @param setMark External switch status?...??.. Sorry... I could not understand. Please see L6470 datasheet.
    */
    int ENQ_releaseSwitch(int motorNumber, bool isClockwise, bool setMark);

    /** Enqueue Command [Goto HOME]@n
    * @~japanese
    * キューに追加:モーターを原点復帰させます
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Return the motor to the zero-potition.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_home(int motorNumber);

    /** Enqueue Command [Goto HOME] [aliase: ENQ_home(int motorNumber)]@n
    * @~japanese
    * キューに追加:モーターを原点復帰させます
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Return the motor to the zero-potition.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_zero(int motorNumber);

    /** Enqueue Command [Goto MARK]@n
    * @~japanese
    * キューに追加:指定されたマーク位置まで回転します
    * @param motorNumber　1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    * マーク位置: -> 関数 "setMarkPosition(int motorNumber, unsigned long value)" を使用してください
    *
    * @~english
    * Add Queue:Motor rotation to the specified MARK Position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    * MARK Position: -> Please use setMarkPosition(int motorNumber, unsigned long value).
    */
    int ENQ_gotoMark(int motorNumber);

    /** Enqueue Command [Reset HOME] (ABS_POS zero = home-position)@n
    * @~japanese
    * キューに追加:現在位置を原点に設定します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Set the zero-position to current position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_resetHome(int motorNumber);

    /** Enqueue Command [Reset HOME] [alias: ENQ_resetHome(int motorNumber)]@n
    * @~japanese
    * キューに追加:現在位置を原点に設定します
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Set the zero-position to current position.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_resetZero(int motorNumber);

    /** Enqueue Command [Reset L6470] (software reset)@n
    * @~japanese
    * キューに追加:デバイスをリセットします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Reset L6470 driver & motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_motorReset(int motorNumber);

    /** Enqueue Command [stop] (soft-stop)@n
    * @~japanese
    * キューに追加:モーターを停止
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Stop motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_stop(int motorNumber);

    /** Enqueue Command [hard stop]@n
    * @~japanese
    * キューに追加:減速を無視して停止
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Stop immediately motor.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_stopImmidiate(int motorNumber);

    /** Enqueue Command [HIZ stop]@n
    * @~japanese
    * キューに追加:モーターを停止し、実行後ハイインピーダンス状態にします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Stop motor. Set state "High Impedance" after stop.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_stop_HighImpedance(int motorNumber);

    /** Enqueue Command [HIZ hard stop]@n
    * @~japanese
    * キューに追加:減速を無視して停止し、実行後ハイインピーダンス状態にします
    * @param motorNumber 1から始まるモーター番号(デイジーチェーン接続していない場合は1を指定)
    *
    * @~english
    * Add Queue:Stop immediately motor. Set state "High Impedance" after stop.
    * @param motorNumber Chained motor-number. If not using daisy-chain, you must be motorNumber = 1.
    */
    int ENQ_stopImmidiate_HighImpedance(int motorNumber);

    /** Clear Queue@n
    * @~japanese コマンドキューを全てクリアします
    * @~english Command Queue all zero clear.
    */
    void Qclear();

    /** Execute Queue@n
    * @~japanese
    * コマンドキューに格納されたコマンドを一度に実行します
    * @param finallyClearQueue 実行後にキューをクリアする(デフォルトはtrue)
    *
    * @~english
    * Execute Command Queue.
    * @param finallyClearQueue Clear Queue after execution.(default=true)
    */
    int Qexec(bool finallyClearQueue=true);

    // calc method -----------------------------------------------------------------
    /** Caluculate HEX value from Speed value@n
    * @~japanese
    * 速度(step/s)をパラメーター用数値に変換します
    * @param stepPerSecond 回転速度(step/s)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from real speed value(step/s).
    * @param stepPerSecond Motor rotate speed(step/s).
    *
    * @returns Converted value.
    */
    unsigned long calcSpd(float stepPerSecond);

    /** Caluculate HEX value from Acceleration speed value@n
    * @~japanese
    * 加速速度(step/s^2)をパラメーター用数値に変換します
    * @param stepPerSecond 加速速度(step/s^2)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from acceleration speed value(step/s^2).
    * @param stepPerSecond rotate acceleration　speed(step/s^2).
    *
    * @returns Converted value.
    */
    unsigned short calcAcc(float stepPerSecond_2);

    /** Caluculate HEX value from Deceleration speed value@n
    * @~japanese
    * 減速速度(step/s^2)をパラメーター用数値に変換します
    * @param stepPerSecond 減速速度(step/s^2)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from deceleration speed value(step/s^2).
    * @param stepPerSecond rotate deceleration　speed(step/s^2).
    *
    * @returns Converted value.
    */
    unsigned short calcDec(float stepPerSecond_2);

    /** Caluculate HEX value from MAX speed value@n
    * @~japanese
    * 最大速度(step/s)をパラメーター用数値に変換します
    * @param stepPerSecond 最大回転速度(step/s)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from maximum speed value(step/s).
    * @param stepPerSecond Motor rotate maximum speed(step/s).
    *
    * @returns Converted value.
    */
    unsigned short calcMaxSpd(float stepPerSecond);

    /** Caluculate HEX value from MIN speed value@n
    * @~japanese
    * 最低速度(step/s)をパラメーター用数値に変換します
    * @param stepPerSecond 最低回転速度(step/s)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from minimum speed value(step/s).
    * @param stepPerSecond Motor rotate minimum speed(step/s).
    *
    * @returns Converted value.
    */
    unsigned short calcMinSpd(float stepPerSecond);

    /** Caluculate HEX value from acceleration and deceleration switched point speed@n
    * @~japanese
    * 加減速曲線が切り替わる速度を指定します
    * @param stepPerSecond 加減速曲線が切り替わる速度(step/s)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from acceleration and deceleration switched point speed(step/s).
    * @param stepPerSecond switched point speed(step/s).
    *
    * @returns Converted value.
    */
    unsigned short calcIntSpd(float stepPerSecond);

    /** Caluculate HEX value from full-step mode switched point speed@n
    * @~japanese
    * フルステップモードに切り替わる速度を指定します
    * @param stepPerSecond フルステップモードに切り替わる速度(step/s)
    *
    * @returns 変換された16進数値
    *
    * @~english
    * Convert to Param(Hex) value from full-step mode switched point speed(step/s).
    * @param stepPerSecond switched point speed(step/s).
    *
    * @returns Converted value.
    */
    unsigned short calcFullStepSpd(float stepPerSecond);

    // set method ------------------------------------------------------------------
    void setAbsPosition(int motorNumber, unsigned long value);                                      //絶対座標設定
    void setElecPosition(int motorNumber, unsigned short value);                                    //マイクロステップ位置設定
    void setMarkPosition(int motorNumber, unsigned long value);                                     //マークポジション設定
    void setAcceleration(int motorNumber, unsigned short value);                                    //加速設定
    void setDeceleration(int motorNumber, unsigned short value);                                    //減速設定
    void setMaximumSpeed(int motorNumber, unsigned short value);                                    //最大回転速度設定
    void setMinimumSpeed(int motorNumber, unsigned short value);                                    //最低回転速度設定(普通はゼロ)
    void setHoldingKVAL(int motorNumber, unsigned char value);                                      //モーター停止中の電圧
    void setRunningKVAL(int motorNumber, unsigned char value);                                      //モーター駆動中の電圧
    void setAccelerationKVAL(int motorNumber, unsigned char value);                                 //モーター加速中の電圧
    void setDecelerationKVAL(int motorNumber, unsigned char value);                                 //モーター減速中の電圧
    void setKVAL(                                                                                   //モーターの電圧(一度に設定)
        int motorNumber,
        unsigned char holdVal, unsigned char runVal,
        unsigned char accVal, unsigned char decVal
    );
    void setInterpolateSpeed(int motorNumber, unsigned short value);                                //加減速補間を開始するスピード
    void setInterpolateSlope(int motorNumber, unsigned char value);                                 //加減速補間の傾き
    void setAccSlopeFinal(int motorNumber, unsigned char value);                                    //加速最終時の補間の傾き
    void setDecSlopeFinal(int motorNumber, unsigned char value);                                    //減速最終時の補間の傾き
    void setThermoCorrect(int motorNumber, unsigned char value);                                    //高温時の補正
    void setOCThreshold(int motorNumber, unsigned char value);                                      //オーバーカレントの電流閾値
    void setStallThreshold(int motorNumber, unsigned char value);                                   //ストールの電流閾値
    void setFSSpeed(int motorNumber, unsigned short value);                                         //フルステップ駆動に切り替えるスピード
    void setStepMode(int motorNumber, unsigned char value);                                         //マイクロステッピングモード指定
    void setAlermEnable(int motorNumber, unsigned char value);                                      //アラームの有効無効
    void setSystemConfig(int motorNumber, unsigned short value);                                    //ドライバシステム設定

    // get method ------------------------------------------------------------------
    unsigned long getSpeed(int motorNumber);                                                        //現在の回転スピード
    unsigned short getADC(int motorNumber);                                                         //ADCの取得
    unsigned short getStatus(int motorNumber);                                                      //ステータス読み取り
    unsigned long getAbsPosition(int motorNumber);                                                  //絶対座標
    unsigned short getElecPosition(int motorNumber);                                                //マイクロステップ位置
    unsigned long getMarkPosition(int motorNumber);                                                 //マークポジション
    unsigned short getAcceleration(int motorNumber);                                                //加速
    unsigned short getDeceleration(int motorNumber);                                                //減速
    unsigned short getMaximumSpeed(int motorNumber);                                                //最大回転速度
    unsigned short getMinimumSpeed(int motorNumber);                                                //最低回転速度(普通はゼロ)
    unsigned char getHoldingKVAL(int motorNumber);                                                  //モーター停止中の電圧
    unsigned char getRunningKVAL(int motorNumber);                                                  //モーター駆動中の電圧
    unsigned char getAccelerationKVAL(int motorNumber);                                             //モーター加速中の電圧
    unsigned char getDecelerationKVAL(int motorNumber);                                             //モーター減速中の電圧
    unsigned short getInterpolateSpeed(int motorNumber);                                            //加減速補間を開始するスピード
    unsigned char getInterpolateSlope(int motorNumber);                                             //加減速補間の傾き
    unsigned char getAccSlopeFinal(int motorNumber);                                                //加速最終時の補間の傾き
    unsigned char getDecSlopeFinal(int motorNumber);                                                //減速最終時の補間の傾き
    unsigned char getThermoCorrect(int motorNumber);                                                //高温時の補正
    unsigned char getOCThreshold(int motorNumber);                                                  //オーバーカレントの電流閾値
    unsigned char getStallThreshold(int motorNumber);                                               //ストールの電流閾値
    unsigned short getFSSpeed(int motorNumber);                                                     //フルステップ駆動に切り替えるスピード
    unsigned char getStepMode(int motorNumber);                                                     //マイクロステッピングモード
    unsigned char getAlermEnable(int motorNumber);                                                  //アラームの有効無効
    unsigned short getSystemConfig(int motorNumber);                                                //ドライバシステム設定
};

#endif
