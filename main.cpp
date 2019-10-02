/*!
  @file   main.cpp
  @author Takuma Kawamura <bitpositive@MacBook-Pro-13.local>
  @date   Wed Jul 10 09:21:26 2019
  
  @brief  FAIRCUT Cutter Main ( India )

  
  @copyright 2019 (C) BitPositive
*/

// Headers //////////////////////////////////////////////////////////
#include "mbed.h"               // Rev. 146
#include "AsyncSerial.hpp"      // For UART
#include "SerialConnect.hpp"    // For Communication with Jetson nano
#include "L6470SDC.h"           // For Stepper Motor
#include "TextLCD.h"            // For HD44780
#include "FIFO.hpp"             // For data queue

// Definitions ////////////////////////////////////////////////////////////////
#define nop() __NOP()
enum motorIdentity_t { SM_VERTICAL_1 = 1, SM_VERTICAL_2, SM_ROTATE };
enum serverCommands_t { COM_SEND = 1, COM_ACK, COM_ROTATE, COM_UPDOWN, COM_NOTLOADED, COM_LOADED, COM_RECEIVED, COM_STARTEDPREDICTING };
enum autoCuttingStates_t { STATE_IDLE, STATE_ROTATE, STATE_DOWN, STATE_UP, STATE_ENDING };

// Global variables ///////////////////////////////////////////////////////////
DigitalIn UpButton( PC_3 );
DigitalIn DownButton( PC_2 );
DigitalIn RotateRightButton( PA_0 ); // New
DigitalIn RotateLeftButton( PA_1 ); // New
DigitalIn RotateExecButton( PA_4 );
DigitalIn EnableCtrlButton( PB_7 );
DigitalIn InitialMoveButton( PC_13 );
DigitalIn ManNotAutoButton( PC_8 ); // New
DigitalIn CSpeedHighButton( PC_6 );
DigitalIn CSpeedLowButton( PA_12 );
DigitalIn CutterResetButton( PF_1 );
DigitalIn RSpeedHighButton( PB_0 );
DigitalIn RSpeedMidButton( PC_1 );
DigitalIn RSpeedLowButton( PC_0 );

DigitalOut GPLED1( PC_11 );
DigitalOut GPLED2( PD_2 );
DigitalOut RSpeedHighLED( PA_13 );
DigitalOut RSpeedMidLED( PA_14 );
DigitalOut RSpeedLowLED( PA_15 );
DigitalOut CuttingLED( PC_9 );

volatile bool IsUpButtonPressed = false;
volatile bool IsDownButtonPressed = false;
volatile bool IsRotateRightButtonPressed = false;
volatile bool IsRotateLeftButtonPressed = false;
volatile bool IsRotateExecButtonPressed = false;
volatile bool IsEnableCtrlButtonPressed = false;
volatile bool IsInitialMoveButtonPressed = false;
volatile bool IsAutoNotManButtonPressed = false;
volatile bool IsCSpeedHighButtonPressed = false;
volatile bool IsCSpeedLowButtonPressed = false;
volatile bool IsCutterResetButtonPressed = false;
volatile bool IsRSpeedHighButtonPressed = false;
volatile bool IsRSpeedMidButtonPressed = false;
volatile bool IsRSpeedLowButtonPressed = false;

autoCuttingStates_t currentAutoCuttingState = STATE_IDLE;
bool IsDuringAutoCuttingProcess = false;

Ticker buttonTicker;
Ticker debugTicker;
Ticker autoCuttingTicker;
AsyncSerial debugSerial( USBTX, USBRX, 115200 );
SerialConnect jetsonPort( PB_9, PB_8, 11, 115200 );
TextLCD display( PA_5, PA_6, PA_7, PB_6, PC_7, PA_9, TextLCD::LCD16x2 );
DigitalOut ssel( PB_12 );
L6470SDC stepperMotors( PB_15, PB_14, PB_13, &ssel );
FIFO<uint8_t> stepAmountQueue( 16 );

// Function prototypes ////////////////////////////////////////////////////////
void InitializeButtons( void );
void InitializeTicker( void );
void InitializeDisplay( void );
void InitializeStepper( void );
bool ManualControlStepper( void );
void ISR_PeriodicButtonWatcher( void );
void ISR_AutoCutting( void );
void CheckButtons( void );

// Main function //////////////////////////////////////////////////////////////
int main(){
  // Initialize buttons
  InitializeButtons();
  
  // Initialize buttons
  InitializeTicker();
  debugTicker.attach( &CheckButtons, 0.2 );
  autoCuttingTicker.attach( &ISR_AutoCutting, 0.5 );
  
  // Initialize serial
  debugSerial.printf( "Hello, Mac\n" );
  
  // Initialize LCD
  InitializeDisplay();
  
  // Initialize stepper motors
  InitializeStepper();
  
  stepAmountQueue.clear();
  
  
  while( 1 ) {
    uint8_t command;
    uint8_t data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    
    bool wasManualControlled = ManualControlStepper();
    
    if( jetsonPort.update() == READY && !wasManualControlled && IsAutoNotManButtonPressed ){
      // Store payload
      command = jetsonPort.get_payload_by_1byte( 0 );
      for( int8_t i = 0; i < 10; i++ ){
        data[i] = jetsonPort.get_payload_by_1byte( i + 1 );
      }
      
      switch( command ){
      case COM_SEND:
        debugSerial.printf( "Recv: COM_SEND\n" );
        debugSerial.printf( "Data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9] );

        for( int8_t i = 0; i < 10; i++ ){
          if( data[i] == 0 ){
            break;
          }else{
            stepAmountQueue.put( ( uint8_t )( data[i] * 2 ) );
          }
        }
        
        display.cls();
        display.printf( "Firmware:  IndiaRecv:   COM_SEND\n" );
        
        if( !IsDuringAutoCuttingProcess && !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) && !stepperMotors.isBusy( SM_ROTATE ) ){
          IsDuringAutoCuttingProcess = true;
        }
        break;
      case COM_NOTLOADED:
        display.cls();
        display.printf("Firmware:  IndiaNow Loading...  \n");
        break;
      case COM_LOADED:
        display.cls();
        display.printf("Firmware:  IndiaStart Capturing!\n");
        break;
      case COM_RECEIVED:
        display.cls();
        display.printf("Firmware:  IndiaReceived:%2d / %-2d\n", data[0], data[1]);
        break;
      case COM_STARTEDPREDICTING:
        display.cls();
        display.printf("Firmware:  IndiaNow Predicting..\n");
        break;
      default:
        break;
      }
    }
    
  }
}

// Functions //////////////////////////////////////////////////////////////////
void InitializeButtons( void )
{
  UpButton.mode( PullUp );
  DownButton.mode( PullUp );
  RotateRightButton.mode( PullUp );
  RotateLeftButton.mode( PullUp );
  RotateExecButton.mode( PullUp );
  EnableCtrlButton.mode( PullUp );
  InitialMoveButton.mode( PullUp );
  ManNotAutoButton.mode( PullUp );
  CSpeedHighButton.mode( PullUp );
  CSpeedLowButton.mode( PullUp );
  CutterResetButton.mode( PullUp );
  RSpeedHighButton.mode( PullUp );
  RSpeedMidButton.mode( PullUp );
  RSpeedLowButton.mode( PullUp );

  return;
}

void InitializeTicker( void )
{
  buttonTicker.attach_us( &ISR_PeriodicButtonWatcher, 10000 ); // 10ms
  
  return;
}

void InitializeDisplay( void )
{
  display.cls();
  display.printf( "   NITMakers\n    Presents\n" );
  wait(1.0);
  display.cls();
  display.printf( "Firmware:  IndiaNot Initialized " );
  
  return;
}

void InitializeStepper( void )
{
  stepperMotors.init();

  // Set 1st motor (VERTICAL1) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_VERTICAL_1, stepperMotors.calcMaxSpd( 250 ) );
  // Set 2nd motor (VERTICAL2) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_VERTICAL_2, stepperMotors.calcMaxSpd( 250 ) );
  // Set 3rd motor (ROTATE) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_ROTATE, stepperMotors.calcMaxSpd( 50 ) );
  
  stepperMotors.resetHome( SM_VERTICAL_1 );
  stepperMotors.resetHome( SM_VERTICAL_2 );
  stepperMotors.resetHome( SM_ROTATE );
  stepperMotors.home( SM_VERTICAL_1 );
  stepperMotors.home( SM_VERTICAL_2 );
  stepperMotors.home( SM_ROTATE );
  
  return;
}

bool ManualControlStepper( void )
{
  bool isActed = false;
  
  if( IsEnableCtrlButtonPressed && !IsAutoNotManButtonPressed ){
    
    if( IsRotateExecButtonPressed ){
      if( IsRotateRightButtonPressed ){
        stepperMotors.run( SM_ROTATE, stepperMotors.calcSpd( 200 ), true ); // Clockwise
        isActed = true;
      }else if( IsRotateLeftButtonPressed ){
        stepperMotors.run( SM_ROTATE, stepperMotors.calcSpd( 200 ), false ); // Counter-Clockwise
        isActed = true;
      }else{
        stepperMotors.stop( SM_ROTATE );
      }
    }else{
      stepperMotors.stop( SM_ROTATE );
    }
    
    if( IsUpButtonPressed ){       // Up
      if( !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) ){
        // Add a action to queue: Rotate Vertical1 2 times/Counter-Clockwise
        stepperMotors.move( SM_VERTICAL_1, 200*128*4, true );
        // Add a action to queue: Rotate Vertical2 2 times/Counter-Clockwise
        stepperMotors.move( SM_VERTICAL_2, 200*128*4, true );
        isActed = true;
      }
    }else if( IsDownButtonPressed ){ // Down
      if( !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) ){
        // Add a action to queue: Rotate Vertical1 2 times/Clockwise
        stepperMotors.move( SM_VERTICAL_1, 200*128*4, false );
        // Add a action to queue: Rotate Vertical2 2 times/Clockwise
        stepperMotors.move( SM_VERTICAL_2, 200*128*4, false );
      }
      isActed = true;
    }
    
    if( IsInitialMoveButtonPressed ){
      stepperMotors.home( SM_ROTATE );
      stepperMotors.home( SM_VERTICAL_1 );
      stepperMotors.home( SM_VERTICAL_2 );
    }
  }
  
  return isActed;
}

// ISR
void ISR_PeriodicButtonWatcher( void )
{
  volatile static uint8_t UpButtonPressCount = 0;
  volatile static uint8_t DownButtonPressCount = 0;
  volatile static uint8_t RotateRightButtonPressCount = 0;
  volatile static uint8_t RotateLeftButtonPressCount = 0;
  volatile static uint8_t RotateExecButtonPressCount = 0;
  volatile static uint8_t EnableCtrlButtonPressCount = 0;
  volatile static uint8_t InitialMoveButtonPressCount = 0;
  volatile static uint8_t ManNotAutoButtonPressCount = 0;
  volatile static uint8_t CSpeedHighButtonPressCount = 0;
  volatile static uint8_t CSpeedLowButtonPressCount = 0;
  volatile static uint8_t CutterResetButtonPressCount = 0;
  volatile static uint8_t RSpeedHighButtonPressCount = 0;
  volatile static uint8_t RSpeedMidButtonPressCount = 0;
  volatile static uint8_t RSpeedLowButtonPressCount = 0;
  
  volatile static uint8_t UpButtonReleaseCount = 0;
  volatile static uint8_t DownButtonReleaseCount = 0;
  volatile static uint8_t RotateRightButtonReleaseCount = 0;
  volatile static uint8_t RotateLeftButtonReleaseCount = 0;
  volatile static uint8_t RotateExecButtonReleaseCount = 0;
  volatile static uint8_t EnableCtrlButtonReleaseCount = 0;
  volatile static uint8_t InitialMoveButtonReleaseCount = 0;
  volatile static uint8_t ManNotAutoButtonReleaseCount = 0;
  volatile static uint8_t CSpeedHighButtonReleaseCount = 0;
  volatile static uint8_t CSpeedLowButtonReleaseCount = 0;
  volatile static uint8_t CutterResetButtonReleaseCount = 0;
  volatile static uint8_t RSpeedHighButtonReleaseCount = 0;
  volatile static uint8_t RSpeedMidButtonReleaseCount = 0;
  volatile static uint8_t RSpeedLowButtonReleaseCount = 0;
  
  if( UpButton.read() == 0 ){
    if( ++UpButtonPressCount > 10 ){
      IsUpButtonPressed = true;
      UpButtonPressCount = 0;
    }
  } else {
    if( ++UpButtonReleaseCount > 10 ){
      IsUpButtonPressed = false;
      UpButtonReleaseCount = 0;
    }
  }

  if( DownButton.read() == 0 ){
    if( ++DownButtonPressCount > 10 ){
      IsDownButtonPressed = true;
      DownButtonPressCount = 0;
    }
  } else {
    if( ++DownButtonReleaseCount > 10 ){
      IsDownButtonPressed = false;
      DownButtonReleaseCount = 0;
    }
  }

  if( RotateRightButton.read() == 0 ){
    if( ++RotateRightButtonPressCount > 10 ){
      IsRotateRightButtonPressed = true;
      RotateRightButtonPressCount = 0;
    }
  } else {
    if( ++RotateRightButtonReleaseCount > 10 ){
      IsRotateRightButtonPressed = false;
      RotateRightButtonReleaseCount = 0;
    }
  }

  if( RotateLeftButton.read() == 0 ){
    if( ++RotateLeftButtonPressCount > 10 ){
      IsRotateLeftButtonPressed = true;
      RotateLeftButtonPressCount = 0;
    }
  } else {
    if( ++RotateLeftButtonReleaseCount > 10 ){
      IsRotateLeftButtonPressed = false;
      RotateLeftButtonReleaseCount = 0;
    }
  }

  if( RotateExecButton.read() == 0 ){
    if( ++RotateExecButtonPressCount > 10 ){
      IsRotateExecButtonPressed = true;
      RotateExecButtonPressCount = 0;
    }
  } else {
    if( ++RotateExecButtonReleaseCount > 10 ){
      IsRotateExecButtonPressed = false;
      RotateExecButtonReleaseCount = 0;
    }
  }

  if( EnableCtrlButton.read() == 0 ){
    if( ++EnableCtrlButtonPressCount > 10 ){
      IsEnableCtrlButtonPressed = true;
      EnableCtrlButtonPressCount = 0;
    }
  } else {
    if( ++EnableCtrlButtonReleaseCount > 10 ){
      IsEnableCtrlButtonPressed = false;
      EnableCtrlButtonReleaseCount = 0;
    }
  }

  if( InitialMoveButton.read() == 0 ){
    if( ++InitialMoveButtonPressCount > 10 ){
      IsInitialMoveButtonPressed = true;
      InitialMoveButtonPressCount = 0;
    }
  } else {
    if( ++InitialMoveButtonReleaseCount > 10 ){
      IsInitialMoveButtonPressed = false;
      InitialMoveButtonReleaseCount = 0;
    }
  }

  if( ManNotAutoButton.read() == 0 ){
    if( ++ManNotAutoButtonPressCount > 10 ){
      IsAutoNotManButtonPressed = true;
      ManNotAutoButtonPressCount = 0;
    }
  } else {
    if( ++ManNotAutoButtonReleaseCount > 10 ){
      IsAutoNotManButtonPressed = false;
      ManNotAutoButtonReleaseCount = 0;
    }
  }

  if( CSpeedHighButton.read() == 0 ){
    if( ++CSpeedHighButtonPressCount > 10 ){
      IsCSpeedHighButtonPressed = true;
      CSpeedHighButtonPressCount = 0;
    }
  } else {
    if( ++CSpeedHighButtonReleaseCount > 10 ){
      IsCSpeedHighButtonPressed = false;
      CSpeedHighButtonReleaseCount = 0;
    }
  }

  if( CSpeedLowButton.read() == 0 ){
    if( ++CSpeedLowButtonPressCount > 10 ){
      IsCSpeedLowButtonPressed = true;
      CSpeedLowButtonPressCount = 0;
    }
  } else {
    if( ++CSpeedLowButtonReleaseCount > 10 ){
      IsCSpeedLowButtonPressed = false;
      CSpeedLowButtonReleaseCount = 0;
    }
  }

  if( CutterResetButton.read() == 0 ){
    if( ++CutterResetButtonPressCount > 10 ){
      IsCutterResetButtonPressed = true;
      CutterResetButtonPressCount = 0;
    }
  } else {
    if( ++CutterResetButtonReleaseCount > 10 ){
      IsCutterResetButtonPressed = false;
      CutterResetButtonReleaseCount = 0;
    }
  }

  if( RSpeedHighButton.read() == 0 ){
    if( ++RSpeedHighButtonPressCount > 10 ){
      IsRSpeedHighButtonPressed = true;
      RSpeedHighButtonPressCount = 0;
    }
  } else {
    if( ++RSpeedHighButtonReleaseCount > 10 ){
      IsRSpeedHighButtonPressed = false;
      RSpeedHighButtonReleaseCount = 0;
    }
  }

  if( RSpeedMidButton.read() == 0 ){
    if( ++RSpeedMidButtonPressCount > 10 ){
      IsRSpeedMidButtonPressed = true;
      RSpeedMidButtonPressCount = 0;      
    }
  } else {
    if( ++RSpeedMidButtonReleaseCount > 10 ){
      IsRSpeedMidButtonPressed = false;
      RSpeedMidButtonReleaseCount = 0;
    }
  }
  
  if( RSpeedLowButton.read() == 0 ){
    if( ++RSpeedLowButtonPressCount > 10 ){
      IsRSpeedLowButtonPressed = true;
      RSpeedLowButtonPressCount = 0;
    }
  } else {
    if( ++RSpeedLowButtonReleaseCount > 10 ){
      IsRSpeedLowButtonPressed = false;
      RSpeedLowButtonReleaseCount = 0;
    }
  }
  
  return;
}

void ISR_AutoCutting( void )
{
  if( IsDuringAutoCuttingProcess && !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) && !stepperMotors.isBusy( SM_ROTATE ) ){
    switch( currentAutoCuttingState ){
    case STATE_IDLE: {
      currentAutoCuttingState = STATE_ROTATE;

      display.cls();
      display.printf( "Firmware:  IndiaState:    Idling\n" );
      
      break;
    }
    case STATE_ROTATE: {
      uint32_t steps = stepAmountQueue.get();
      stepperMotors.move( SM_ROTATE, steps * 128, true );
      currentAutoCuttingState = STATE_DOWN;
      
      display.cls();
      display.printf( "Firmware:  IndiaState:  Rotating\n" );
      
      break;
    }
    case STATE_DOWN: {
      uint8_t rotation = 4;
      uint32_t steps = ( uint32_t )( rotation * 200 * 128 );
      stepperMotors.move( SM_VERTICAL_1, steps, false );
      stepperMotors.move( SM_VERTICAL_2, steps, false );
      currentAutoCuttingState = STATE_UP;

      display.cls();
      display.printf( "Firmware:  IndiaState:   Cutting\n" );
      
      break;
    }
    case STATE_UP: {
      uint8_t rotation = 4;
      uint32_t steps = ( uint32_t )( rotation * 200 * 128 );
      stepperMotors.move( SM_VERTICAL_1, steps, true );
      stepperMotors.move( SM_VERTICAL_2, steps, true );
      
      if( stepAmountQueue.available() <= 0 ){
        // it's last action
        currentAutoCuttingState = STATE_ENDING;

        display.cls();
        display.printf( "Firmware:  IndiaState:    Ending\n" );
      }else{
        // there is more actions to do
        currentAutoCuttingState = STATE_ROTATE;
        
        display.cls();
        display.printf( "Firmware:  IndiaState:   Raising\n" );
      }
      
      break;
    }
    case STATE_ENDING: {
      uint32_t steps = 50 * 128;
      stepperMotors.move( SM_ROTATE, steps, false );
      stepAmountQueue.clear();
      currentAutoCuttingState = STATE_IDLE;
      IsDuringAutoCuttingProcess = false;
      
      display.cls();
      display.printf( "State: CompletedStart Capturing!\n");

      break;
    }
    default:
      break;
    }
  }
  
  return;
}

void CheckButtons( void )
{
  
  // if( IsUpButtonPressed ){
  //   debugSerial.printf("UpButton\n");
  // }
  // if( IsDownButtonPressed ){
  //   debugSerial.printf("DownButton\n");
  // }
  // if( IsRotateRightButtonPressed ){
  //   debugSerial.printf("RotateRightButton\n");
  // }
  // if( IsRotateLeftButtonPressed ){
  //   debugSerial.printf("RotateLeftButton\n");
  // }
  // if( IsRotateExecButtonPressed ){
  //   debugSerial.printf("RotateExecButton\n");
  // }
  // if( IsEnableCtrlButtonPressed ){
  //   debugSerial.printf("EnableCtrlButton\n");
  // }
  // if( IsInitialMoveButtonPressed ){
  //   debugSerial.printf("InitialMoveButton\n");
  // }
  // if( IsAutoNotManButtonPressed ){
  //   debugSerial.printf("AutoNotManButton\n");
  // }
  // if( IsCSpeedHighButtonPressed ){
  //   debugSerial.printf("CSHighButton\n");
  // }
  // if( IsCSpeedLowButtonPressed ){
  //   debugSerial.printf("CSLowButton\n");
  // }
  // if( IsCutterResetButtonPressed ){
  //   debugSerial.printf("CResetButton\n");
  // }
  // if( IsRSpeedHighButtonPressed ){
  //   debugSerial.printf("RSHighButton\n");
  // }
  // if( IsRSpeedMidButtonPressed ){
  //   debugSerial.printf("RSMidButton\n");
  // }
  // if( IsRSpeedLowButtonPressed ){
  //   debugSerial.printf("RSLowButton\n");
  // }
}
