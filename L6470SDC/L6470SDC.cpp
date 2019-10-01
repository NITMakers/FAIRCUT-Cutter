#include "L6470SDC.h"

#define CS_ACTIVE cs->write(0)
#define CS_INACTIVE cs->write(1)

/* ---------------------------------------------------------------------------
 * Constructor (override +9)
 -------------------------------------------------------------------------- */
L6470SDC::L6470SDC(PinName tx, PinName rx, PinName mosi, PinName miso, PinName sclk, PinName csel){
    pc = new Serial(tx, rx);
    spi = new SPI(mosi, miso, sclk);
    cs = new DigitalOut(csel);
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(PinName tx, PinName rx, PinName mosi, PinName miso, PinName sclk, DigitalOut *csel){
    pc = new Serial(tx, rx);
    spi = new SPI(mosi, miso, sclk);
    cs = csel;
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(Serial *serial, PinName mosi, PinName miso, PinName sclk, PinName csel){
    pc = serial;
    spi = new SPI(mosi, miso, sclk);
    cs = new DigitalOut(csel);
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(Serial *serial, PinName mosi, PinName miso, PinName sclk, DigitalOut *csel){
    pc = serial;
    spi = new SPI(mosi, miso, sclk);
    cs = csel;
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(Serial *serial, SPI *spi, PinName csel){
    pc = serial;
    this->spi = spi;
    cs = new DigitalOut(csel);
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(Serial *serial, SPI *spi, DigitalOut *csel){
    pc = serial;
    this->spi = spi;
    cs = csel;
    hasSerial = !(pc == NULL);
}
L6470SDC::L6470SDC(PinName mosi, PinName miso, PinName sclk, PinName csel){
    hasSerial = false;
    spi = new SPI(mosi, miso, sclk);
    cs = new DigitalOut(csel);
}
L6470SDC::L6470SDC(PinName mosi, PinName miso, PinName sclk, DigitalOut *csel){
    hasSerial = false;
    spi = new SPI(mosi, miso, sclk);
    cs = csel;
}
L6470SDC::L6470SDC(SPI *spi, PinName csel){
    hasSerial = false;
    this->spi = spi;
    cs = new DigitalOut(csel);
}
L6470SDC::L6470SDC(SPI *spi, DigitalOut *csel){
    hasSerial = false;
    this->spi = spi;
    cs = csel;
}

/* ---------------------------------------------------------------------------
 * Destructor
 -------------------------------------------------------------------------- */
L6470SDC::~L6470SDC(){
    delete[] Queue;
    delete pc;
    delete spi;
    delete cs;
}

/* ---------------------------------------------------------------------------
 * initializer
 -------------------------------------------------------------------------- */
void L6470SDC::init(bool initSPI){
    //SPI初期化
    if(initSPI){
        spi->frequency(L6470_SPI_FREQ);
        spi->format(8, 3);
    }

    CS_INACTIVE;

    wait_ms(5);

    //SPIで値を送り出し、値が出てくるまでの個数を特定する(デイジーチェーンされている個数がわかる)
    unsigned char rsv = 0x00;
    motor_count = 0;
    CS_ACTIVE;
    spi->write(0xEE);
    do{
        rsv = spi->write(CMD_NOP);
        motor_count++;
    }while(rsv != 0xEE);
    CS_INACTIVE;
    
    //モーター個数分のキューを作成(要素番号とモーター番号を合わせるため+1している)
    Queue = new L6470CMDQ[motor_count+1];

    //キューをクリア
    Qclear();

    //全てのモーターに空コマンドを送り、リセットをかける
    CS_ACTIVE;
    for(int i = 0; i < (motor_count << 2); i++) spi->write(CMD_NOP);
    CS_INACTIVE;
    for(int i = 1; i <= motor_count; i++) motorReset(i);

    //全てのモーターに初期設定
    //最大速度 1秒間に4回転
    //加速、減速 0x02E0(ほぼ一瞬で加減速)
    //フルステップ移行速度 0xff(常にマイクロステップ駆動)
    for(int i = 1; i <= motor_count; i++){
        setMaximumSpeed(i, calcMaxSpd(800.0));
        setAcceleration(i, 0x02E0);
        setDeceleration(i, 0x02E0);
        setFSSpeed(i, 0xff);
    }
}

/* ---------------------------------------------------------------------------
 * GET motor count
 -------------------------------------------------------------------------- */
int L6470SDC::getMotorCount(){
    return motor_count;
}

/* ---------------------------------------------------------------------------
 * PRIVATE: command send to L6470
 -------------------------------------------------------------------------- */
void L6470SDC::setCmd(int motorNumber, unsigned char cmdAddress, unsigned long value, int bitLen){
#ifdef DEBUG_L6470SDC
    if(hasSerial) pc->printf("[Command] Motor-No.%03d, CMD:0x%04X, VAL:0x%06X\r\n", motorNumber, cmdAddress, value);
#endif
    //コマンド送信：対象のモーター以外にはNOPを送信する
    CS_ACTIVE;
    for(int i = motor_count; i >= 1; i--) spi->write((i == motorNumber)? cmdAddress : CMD_NOP);
    CS_INACTIVE;

    //コマンドの後続の値がなければ終了
    if(bitLen < 1) return;

    //値を8bitに分けて送信していく：対象のモーター以外には同じ数だけNOPを送信する
    //bitLenを8で割り、送信回数を求める
    //送信回数の分だけモーター個数分ループさせ(jとiをひっくり返している)
    //対象のモーター以外には同じ数だけNOPを送信する
    int send8 = (bitLen - 1) >> 3; //int send8 = (bitLen-1) / 8;
    int top8bit = bitLen - (send8 << 3); //int top8bit = bitLen - (send8 * 8);
    ++send8;
    for(int j = 1; j <= send8; j++){
        CS_ACTIVE;
        for(int i = motor_count; i >= 1; i--){
            if(i == motorNumber){
                bitLen -= (j == 1) ? top8bit : 8;
                spi->write((unsigned char)(value >> bitLen));
#ifdef DEBUG_L6470SDC
                if(hasSerial) pc->printf(">>>> value transfer[%d] = 0x%02X\r\n", j, (unsigned char)(value >> bitLen));
#endif
            }else{
                spi->write(CMD_NOP);
            }
        }
        CS_INACTIVE;
    }
}

/* ---------------------------------------------------------------------------
 * PRIVATE: parameter send to L6470
 -------------------------------------------------------------------------- */
void L6470SDC::setParam(int motorNumber, unsigned char regAddress, unsigned long value, int bitLen){
#ifdef DEBUG_L6470SDC
    if(hasSerial) pc->printf("[SetParam] Motor-No.%03d, SetParam: REG:0x%04X, VAL:0x%06X\r\n", motorNumber, regAddress, value);
#endif
    //SETPARAM送信：対象のモーター以外にはNOPを送信する
    CS_ACTIVE;
    for(int i = motor_count; i >= 1; i--) spi->write((i == motorNumber)? (CMD_SETPARAM | regAddress) : CMD_NOP);
    CS_INACTIVE;

    //SETPARAMに続く引数を8bitに分けて送信していく：対象のモーター以外には同じ数だけNOPを送信する
    //bitLenを8で割り、送信回数を求める
    //送信回数の分だけモーター個数分ループさせ(jとiをひっくり返している)
    //対象のモーター以外には同じ数だけNOPを送信する
    int send8 = (bitLen - 1) >> 3; //int send8 = (bitLen-1) / 8;
    int top8bit = bitLen - (send8 << 3); //int top8bit = bitLen - (send8 * 8);
    ++send8;
    for(int j = 1; j <= send8; j++){
        CS_ACTIVE;
        for(int i = motor_count; i >= 1; i--){
            if(i == motorNumber){
                bitLen -= (j == 1) ? top8bit : 8;
                spi->write((unsigned char)(value >> bitLen));
#ifdef DEBUG_L6470SDC
                if(hasSerial) pc->printf(">>>> value transfer[%d] = 0x%02X\r\n", j, (unsigned char)(value >> bitLen));
#endif
            }else{
                spi->write(CMD_NOP);
            }
        }
        CS_INACTIVE;
    }
}

/* ---------------------------------------------------------------------------
 * PRIVATE: parameter get from L6470
 -------------------------------------------------------------------------- */
unsigned long L6470SDC::getParam(int motorNumber, unsigned char regAddress, int bitLen){
    //戻り値
    unsigned long ret = 0;
    unsigned char val = 0;

    //GETPARAM送信：対象のモーター以外にはNOPを送信する
    CS_ACTIVE;
    for(int i = motor_count; i >= 1; i--) spi->write((i == motorNumber)? (CMD_GETPARAM | regAddress) : CMD_NOP);
    CS_INACTIVE;

    //データを8bitに分けて受信してretに格納していく
    //bitLenを8で割り、受信回数を求める
    //受信回数の分だけモーター個数分ループさせ(jとiをひっくり返している)
    //対象のモーター以外の受信は無視する
    int receive8 = (bitLen - 1) >> 3; //int receive8 = (bitLen-1) / 8;
    ++receive8;
    for(int j = 1; j <= receive8; j++){
        CS_ACTIVE;
        for(int i = motor_count; i >= 1; i--){
            if(i == motorNumber){
                ret |= (j == receive8) ? spi->write(CMD_NOP) : spi->write(CMD_NOP) << ((receive8 - j) << 3);
#ifdef DEBUG_L6470SDC
                if(hasSerial) pc->printf("<<<< receive data[%d] = 0x%06X\r\n", j, ret);
#endif
            }else{
                val = spi->write(CMD_NOP);
#ifdef DEBUG_L6470SDC
                if(hasSerial) pc->printf("???? receive other data[%d] = 0x%02X\r\n", j, val);
#endif
            }
        }
        CS_INACTIVE;
    }
#ifdef DEBUG_L6470SDC
    if(hasSerial) pc->printf("[GetParam] Motor-No.%03d, GetParam: REG:0x%04X, VAL:0x%06X\r\n", motorNumber, regAddress, ret);
#endif
    return ret;
}

/* ---------------------------------------------------------------------------
 * COMMAND
 -------------------------------------------------------------------------- */
//@ GET busyflag
bool L6470SDC::isBusy(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return false;
    return !((getStatus(motorNumber) & 0x02) >> 1);
}
//@ motor run
void L6470SDC::run(int motorNumber, unsigned long hex_speed, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, (isClockwise) ? CMD_RUN_MINUS : CMD_RUN_PLUS, hex_speed, 20);
}
//@ motor step
void L6470SDC::step(int motorNumber, unsigned int count, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return;
    for(;count<=0;count--){
        while(isBusy(motorNumber));
        setCmd(motorNumber, (isClockwise) ? CMD_STEP_MINUS : CMD_STEP_PLUS, 0, 0);
    }
}
//@ motor move
void L6470SDC::move(int motorNumber, unsigned long stepsCount, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, (isClockwise) ? CMD_ADDSTEP_MINUS : CMD_ADDSTEP_PLUS, stepsCount, 22);
}
//@ motor goto ABSpos(shortest path)
void L6470SDC::goto1(int motorNumber, unsigned long ABSPos){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_GOTO, ABSPos, 22);
}
//@ motor goto ABSpos
void L6470SDC::goto2(int motorNumber, unsigned long ABSPos, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, (isClockwise) ? CMD_GOTO_DIR_MINUS : CMD_GOTO_DIR_PLUS, ABSPos, 22);
}
//@ motor goto mark
void L6470SDC::goUntil(int motorNumber, unsigned long hex_speed, bool isClockwise, bool setMark){
    if(motorNumber > motor_count || motorNumber < 1) return;
    unsigned short cmdAddr = (isClockwise) ? CMD_GO_UNTIL_MINUS : CMD_GO_UNTIL_PLUS;
    setCmd(motorNumber, (setMark) ? (cmdAddr | 0x08) : cmdAddr, hex_speed, 22);
}
//@ release sw
void L6470SDC::releaseSwitch(int motorNumber, bool isClockwise, bool setMark){
    if(motorNumber > motor_count || motorNumber < 1) return;
    unsigned short cmdAddr = (isClockwise) ? CMD_RELEASE_SW_MINUS : CMD_RELEASE_SW_PLUS;
    setCmd(motorNumber, (setMark) ? (cmdAddr | 0x08) : cmdAddr, 0, 0);
}
//@ motor goto zero-position
void L6470SDC::home(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_GO_HOME, 0, 0);
}
//@ alias:home
void L6470SDC::zero(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_GO_HOME, 0, 0);
}
//@ motor goto MARK-position
void L6470SDC::gotoMark(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_GO_MARK, 0, 0);
}
//@ reset zero-position
void L6470SDC::resetHome(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_RESET_POS, 0, 0);
}
//@ alias:resetHome
void L6470SDC::resetZero(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_RESET_POS, 0, 0);
}
//@ motor reset
void L6470SDC::motorReset(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_RESET_DEVICE, 0, 0);
}
//@ motor soft-stop
void L6470SDC::stop(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_SOFT_STOP, 0, 0);
}
//@ motor hard-stop
void L6470SDC::stopImmidiate(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_HARD_STOP, 0, 0);
}
//@ motor HighImpedance
void L6470SDC::stop_HighImpedance(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_SOFT_HIZ, 0, 0);
}
//@ motor HighImpedance hard-stop
void L6470SDC::stopImmidiate_HighImpedance(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return;
    setCmd(motorNumber, CMD_HARD_HIZ, 0, 0);
}

/* ---------------------------------------------------------------------------
 * COMMAND (Daisy-chain Enqueue)
 -------------------------------------------------------------------------- */
//@ enqueue NOP
int L6470SDC::ENQ_nop(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_NOP;
    Queue[motorNumber].val = CMD_NOP;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue run command
int L6470SDC::ENQ_run(int motorNumber, unsigned long hex_speed, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = (isClockwise) ? CMD_RUN_MINUS : CMD_RUN_PLUS;
    Queue[motorNumber].val = hex_speed;
    Queue[motorNumber].bitLen = 20;
    return motorNumber;
}
//@ enqueue move command
int L6470SDC::ENQ_move(int motorNumber, unsigned long stepsCount, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = (isClockwise) ? CMD_ADDSTEP_MINUS : CMD_ADDSTEP_PLUS;
    Queue[motorNumber].val = stepsCount;
    Queue[motorNumber].bitLen = 22;
    return motorNumber;
}
//@ enqueue goto(shortest) command
int L6470SDC::ENQ_goto1(int motorNumber, unsigned long ABSPos){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_GOTO;
    Queue[motorNumber].val = ABSPos;
    Queue[motorNumber].bitLen = 22;
    return motorNumber;
}
//@ enqueue goto command
int L6470SDC::ENQ_goto2(int motorNumber, unsigned long ABSPos, bool isClockwise){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = (isClockwise) ? CMD_GOTO_DIR_MINUS : CMD_GOTO_DIR_PLUS;
    Queue[motorNumber].val = ABSPos;
    Queue[motorNumber].bitLen = 22;
    return motorNumber;
}
//@ enqueue goUntil command
int L6470SDC::ENQ_goUntil(int motorNumber, unsigned long hex_speed, bool isClockwise, bool setMark){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    unsigned short cmdAddr = (isClockwise) ? CMD_GO_UNTIL_MINUS : CMD_GO_UNTIL_PLUS;
    Queue[motorNumber].addr = (setMark)? (cmdAddr | 0x08) : cmdAddr;
    Queue[motorNumber].val = hex_speed;
    Queue[motorNumber].bitLen = 22;
    return motorNumber;
}
//@ enqueue release switch command
int L6470SDC::ENQ_releaseSwitch(int motorNumber, bool isClockwise, bool setMark){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    unsigned short cmdAddr = (isClockwise) ? CMD_RELEASE_SW_MINUS : CMD_RELEASE_SW_PLUS;
    Queue[motorNumber].addr = (setMark)? (cmdAddr | 0x08) : cmdAddr;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue goHome command
int L6470SDC::ENQ_home(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_GO_HOME;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ alias:ENQ_home
int L6470SDC::ENQ_zero(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_GO_HOME;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue gotoMark command
int L6470SDC::ENQ_gotoMark(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_GO_MARK;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue reset_pos command
int L6470SDC::ENQ_resetHome(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_RESET_POS;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ alias:ENQ_resetHome
int L6470SDC::ENQ_resetZero(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_RESET_POS;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue reset motor command
int L6470SDC::ENQ_motorReset(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_RESET_DEVICE;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue soft-stop command
int L6470SDC::ENQ_stop(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_SOFT_STOP;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue hard-stop command
int L6470SDC::ENQ_stopImmidiate(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_HARD_STOP;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue soft-stop(HIZ) command
int L6470SDC::ENQ_stop_HighImpedance(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_SOFT_HIZ;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ enqueue hard-stop(HIZ) command
int L6470SDC::ENQ_stopImmidiate_HighImpedance(int motorNumber){
    if(motorNumber > motor_count || motorNumber < 1) return -1;
    Queue[motorNumber].addr = CMD_HARD_HIZ;
    Queue[motorNumber].val = 0;
    Queue[motorNumber].bitLen = 0;
    return motorNumber;
}
//@ command queue clear
void L6470SDC::Qclear(){
    for(int i=0; i<=motor_count; i++){
        Queue[i].addr = 0;
        Queue[i].val = 0;
        Queue[i].bitLen = -1;
    }
}
//@ queue execute
int L6470SDC::Qexec(bool finallyClearQueue){
    int bitLens[motor_count];
    memset(bitLens, 0, sizeof(int) * motor_count);
    int maxBitLen = 0;

    //構造体の中のbitLenの最大値を抽出, 計算に使用するので別枠に格納
    for(int i=0; i<motor_count; i++){
        bitLens[i] = Queue[i+1].bitLen;
        if(maxBitLen < bitLens[i]) maxBitLen = bitLens[i];
    }

    //bitLenの最大値が0未満、つまり構造体に値が入っていないのでここで終わる
    if(maxBitLen < 0) return -1;

    //コマンド送信
    CS_ACTIVE;
    for(int i=0; i<motor_count; i++){
        spi->write(Queue[i+1].addr);
#ifdef DEBUG_L6470SDC
        if(hasSerial) pc->printf("[Queue:Command] Motor-No.%03d, CMD:0x%04X\r\n", i+1, Queue[i+1].addr);
#endif
    }
    CS_INACTIVE;

    //bitLenの最大値が0、つまり送る値がないのでここで終わる
    if(maxBitLen < 1) return 0;

    //モーター個数分×送信回数分の箱を作り、その中にあらかじめ値を入れ、それを順次送り出す
    //bitLenが最も長いものから送信回数を求める
    int maxSend8 = (maxBitLen - 1) >> 3;

    //8bitに区切った場合の区切りの場所を算出
    int top8bit[motor_count];
    memset(top8bit, 0, sizeof(int) * motor_count);
    for(int i=0, send8c=0; i<motor_count; i++){
        send8c = (bitLens[i] - 1) >> 3;
        top8bit[i] = bitLens[i] - (send8c << 3);
    }

    //箱の中に値を入れていく
    ++maxSend8;
    unsigned char valBox[motor_count * maxSend8];
    memset(valBox, 0, sizeof(char) * motor_count * maxSend8);
    for(int j=0; j<maxSend8; j++){
        for(int i=0, slide=j*maxSend8; i<motor_count; i++){
            bitLens[i] -= (j==0)? top8bit[i] : 8;
            valBox[i+slide] = (bitLens[i]<0)? CMD_NOP : (unsigned char)(Queue[i+1].val >> bitLens[i]);
        }
    }

    //値の送出
    for(int j=0; j<maxSend8; j++){
        CS_ACTIVE;
        for(int i=0, slide=0; i<motor_count; i++){
            slide = j*maxSend8;
            spi->write(valBox[i+slide]);
#ifdef DEBUG_L6470SDC
            if(hasSerial) pc->printf("[Queue:Values]>>>> Motor-No.%03d, VAL[%d]:0x%02X\r\n", i+1, j, valBox[i+slide]);
#endif
        }
        CS_INACTIVE;
    }

    //キューの値を空に
    if(finallyClearQueue) Qclear();

    return 1;
}

/* ---------------------------------------------------------------------------
 * CALCULATE
 -------------------------------------------------------------------------- */
unsigned long L6470SDC::calcSpd(float stepPerSecond){
    stepPerSecond = (stepPerSecond < 0) ? 0 : stepPerSecond;
    //SPEED = ステップ毎秒 * 250ナノ秒 / 2^-28
    //250ナノ秒 / 2^-28 = 2.5e-7 / 2^-28 = 67.108864
    unsigned long ret = stepPerSecond * 67.108864;
    return (ret > 1048575) ? 1048575 : ret;
}

unsigned short L6470SDC::calcAcc(float stepPerSecond_2){
    stepPerSecond_2 = (stepPerSecond_2 < 0) ? 0 : stepPerSecond_2;
    //ACC = ステップ毎秒毎秒 * 250ナノ秒^2 / 2^-40
    //250ナノ秒^2 / 2^-40 = (2.5e-7)^2 / 2^-40 = 0.0687194
    unsigned short ret = stepPerSecond_2 * 0.0687194;
    return (ret > 4095) ? 4095 : ret;
}

unsigned short L6470SDC::calcDec(float stepPerSecond_2){
    stepPerSecond_2 = (stepPerSecond_2 < 0) ? 0 : stepPerSecond_2;
    //DEC = ステップ毎秒毎秒 * 250ナノ秒^2 / 2^-40
    //250ナノ秒^2 / 2^-40 = (2.5e-7)^2 / 2^-40 = 0.0687194
    unsigned short ret = stepPerSecond_2 * 0.0687194;
    return (ret > 4095) ? 4095 : ret;
}

unsigned short L6470SDC::calcMaxSpd(float stepPerSecond){
    stepPerSecond = (stepPerSecond < 0) ? 0 : stepPerSecond;
    //MAXSPEED = ステップ毎秒 * 250ナノ秒 / 2^-18
    //250ナノ秒 / 2^-18 = 2.5e-7 / 2^-18 = 0.065536
    unsigned long ret = stepPerSecond * 0.065536;
    return (ret > 1023) ? 1023 : ret;
}

unsigned short L6470SDC::calcMinSpd(float stepPerSecond){
    stepPerSecond = (stepPerSecond < 0) ? 0 : stepPerSecond;
    //MINSPEED = ステップ毎秒 * 250ナノ秒 / 2^-24
    //250ナノ秒 / 2^-24 = 2.5e-7 / 2^-24 = 4.194304
    unsigned long ret = stepPerSecond * 4.194304;
    return (ret > 4094) ? 4094 : ret;
}

unsigned short L6470SDC::calcIntSpd(float stepPerSecond){
    stepPerSecond = (stepPerSecond < 0) ? 0 : stepPerSecond;
    //INT_SPEED = ステップ毎秒 * 250ナノ秒 / 2^-24
    //250ナノ秒 / 2^-24 = 2.5e-7 / 2^-24 = 4.194304
    unsigned long ret = stepPerSecond * 4.194304;
    return (ret > 16382) ? 16382 : ret;
}

unsigned short L6470SDC::calcFullStepSpd(float stepPerSecond){
    stepPerSecond = (stepPerSecond < 0) ? 0 : stepPerSecond;
    //FS_SPEED = (ステップ毎秒 * 250ナノ秒 / 2^-18) - 0.5
    //250ナノ秒 / 2^-18 = 2.5e-7 / 2^-18 = 0.065536
    unsigned long ret = stepPerSecond * 0.065536 - 0.5;
    return (ret > 1023) ? 1023 : ret;
}

// SET method ------------------------------------------------------------------------------
void L6470SDC::setAbsPosition(int motorNumber, unsigned long value){
    setParam(motorNumber, REG_ABS_POS, value, 22);
}

void L6470SDC::setElecPosition(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_EL_POS, value, 9);
}

void L6470SDC::setMarkPosition(int motorNumber, unsigned long value){
    setParam(motorNumber, REG_MARK, value, 22);
}

void L6470SDC::setAcceleration(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_ACC, value, 12);
}

void L6470SDC::setDeceleration(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_DEC, value, 12);
}

void L6470SDC::setMaximumSpeed(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_MAX_SPEED, value, 10);
}

void L6470SDC::setMinimumSpeed(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_MIN_SPEED, value, 13);
}

void L6470SDC::setHoldingKVAL(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_KVAL_HOLD, value, 8);
}

void L6470SDC::setRunningKVAL(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_KVAL_RUN, value, 8);
}

void L6470SDC::setAccelerationKVAL(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_KVAL_ACC, value, 8);
}

void L6470SDC::setDecelerationKVAL(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_KVAL_DEC, value, 8);
}

void L6470SDC::setKVAL(int motorNumber, unsigned char holdVal, unsigned char runVal, unsigned char accVal, unsigned char decVal){
    setParam(motorNumber, REG_KVAL_HOLD, holdVal, 8);
    setParam(motorNumber, REG_KVAL_RUN, runVal, 8);
    setParam(motorNumber, REG_KVAL_ACC, accVal, 8);
    setParam(motorNumber, REG_KVAL_DEC, decVal, 8);
}

void L6470SDC::setInterpolateSpeed(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_INT_SPD, value, 14);
}

void L6470SDC::setInterpolateSlope(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_ST_SLP, value, 8);
}

void L6470SDC::setAccSlopeFinal(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_FN_SLP_ACC, value, 8);
}

void L6470SDC::setDecSlopeFinal(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_FN_SLP_DEC, value, 8);
}

void L6470SDC::setThermoCorrect(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_K_THERM, value, 4);
}

void L6470SDC::setOCThreshold(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_OCD_TH, value, 4);
}

void L6470SDC::setStallThreshold(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_STALL_TH, value, 7);
}

void L6470SDC::setFSSpeed(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_FS_SPD, value, 10);
}

void L6470SDC::setStepMode(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_STEP_MODE, value, 8);
}

void L6470SDC::setAlermEnable(int motorNumber, unsigned char value){
    setParam(motorNumber, REG_ALARM_EN, value, 8);
}

void L6470SDC::setSystemConfig(int motorNumber, unsigned short value){
    setParam(motorNumber, REG_CONFIG, value, 16);
}

// GET method ------------------------------------------------------------------------------
unsigned long L6470SDC::getSpeed(int motorNumber){
    return getParam(motorNumber, REG_SPEED, 20);
}

unsigned short L6470SDC::getADC(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_ADC_OUT, 5);
}

unsigned short L6470SDC::getStatus(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_STATUS, 16);
}

unsigned long L6470SDC::getAbsPosition(int motorNumber){
    return getParam(motorNumber, REG_ABS_POS, 22);
}

unsigned short L6470SDC::getElecPosition(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_EL_POS, 9);
}

unsigned long L6470SDC::getMarkPosition(int motorNumber){
    return getParam(motorNumber, REG_MARK, 22);
}

unsigned short L6470SDC::getAcceleration(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_ACC, 12);
}

unsigned short L6470SDC::getDeceleration(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_DEC, 12);
}

unsigned short L6470SDC::getMaximumSpeed(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_MAX_SPEED, 10);
}

unsigned short L6470SDC::getMinimumSpeed(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_MIN_SPEED, 13);
}

unsigned char L6470SDC::getHoldingKVAL(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_KVAL_HOLD, 8);
}

unsigned char L6470SDC::getRunningKVAL(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_KVAL_RUN, 8);
}

unsigned char L6470SDC::getAccelerationKVAL(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_KVAL_ACC, 8);
}

unsigned char L6470SDC::getDecelerationKVAL(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_KVAL_DEC, 8);
}

unsigned short L6470SDC::getInterpolateSpeed(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_INT_SPD, 14);
}

unsigned char L6470SDC::getInterpolateSlope(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_ST_SLP, 8);
}

unsigned char L6470SDC::getAccSlopeFinal(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_FN_SLP_ACC, 8);
}

unsigned char L6470SDC::getDecSlopeFinal(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_FN_SLP_DEC, 8);
}

unsigned char L6470SDC::getThermoCorrect(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_K_THERM, 4);
}

unsigned char L6470SDC::getOCThreshold(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_OCD_TH, 4);
}

unsigned char L6470SDC::getStallThreshold(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_STALL_TH, 7);
}

unsigned short L6470SDC::getFSSpeed(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_FS_SPD, 10);
}

unsigned char L6470SDC::getStepMode(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_STEP_MODE, 8);
}

unsigned char L6470SDC::getAlermEnable(int motorNumber){
    return (unsigned char) getParam(motorNumber, REG_ALARM_EN, 8);
}

unsigned short L6470SDC::getSystemConfig(int motorNumber){
    return (unsigned short) getParam(motorNumber, REG_CONFIG, 16);
}
