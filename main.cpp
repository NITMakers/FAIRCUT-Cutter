/*!
  @file   main.cpp
  @author Takuma Kawamura <bitpositive@MacBook-Pro-13.local>
  @date   Wed Jul 10 09:21:26 2019
  
  @brief  FAIRCUT Cutter Main ( Juliett )

  
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
enum serverCommands_t { COM_SEND = 1, COM_ACK, COM_ROTATE, COM_UPDOWN, COM_LOADING, COM_LOADED, COM_RECEIVED, COM_STARTEDPREDICTING, COM_SHUTDOWN };
enum autoCuttingStates_t { STATE_IDLE, STATE_ROTATE, STATE_DOWN, STATE_UP, STATE_ENDING };

// Global variables ///////////////////////////////////////////////////////////
DigitalIn UpButton( PC_3 );
DigitalIn DownButton( PC_2 );
DigitalIn RotateExecButton( PA_1 );
DigitalIn RotateRightButton( PA_4 );
DigitalIn RotateLeftButton( PB_0 );
DigitalIn ManNotAutoButton( PC_1 );
DigitalIn EnableCtrlButton( PC_0 );

volatile bool IsUpButtonPressed = false;
volatile bool IsDownButtonPressed = false;
volatile bool IsRotateRightButtonPressed = false;
volatile bool IsRotateLeftButtonPressed = false;
volatile bool IsRotateExecButtonPressed = false;
volatile bool IsEnableCtrlButtonPressed = false;
volatile bool IsAutoNotManButtonPressed = false;

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
uint32_t totalStepAmountCount = 0;

// Function prototypes ////////////////////////////////////////////////////////
void InitializeButtons( void );
void InitializeTicker( void );
void InitializeDisplay( void );
void InitializeStepper( void );
bool ManualControlStepper( void );
void ISR_AutoCutting( void );
void ISR_PeriodicButtonWatcher( void );
void ISR_CheckButtons( void );

// Main function //////////////////////////////////////////////////////////////
int main(){
  // Initialize buttons
  InitializeButtons();
  
  // Initialize buttons
  InitializeTicker();
  debugTicker.attach( &ISR_CheckButtons, 0.2 );
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

      debugSerial.printf("Command: %d\n", command);
      
      switch( command ){
      case COM_SEND:
        debugSerial.printf( "Recv: COM_SEND\n" );
        debugSerial.printf( "Data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9] );

        totalStepAmountCount = 0;
        
        for( int8_t i = 0; i < 10; i++ ){
          if( data[i] == 0 ){
            break;
          }else{
            stepAmountQueue.put( ( uint8_t )( data[i] * 2 ) );
            totalStepAmountCount++;
          }
        }
        
        // display.cls();
        // display.printf( "Firmware:Juliett\n\n" );
        
        if( !IsDuringAutoCuttingProcess && !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) && !stepperMotors.isBusy( SM_ROTATE ) ){
          IsDuringAutoCuttingProcess = true;
        }
        break;
      case COM_LOADING:
        display.cls();
        display.printf("Loading VGGNet16Please wait..   \n");
        debugSerial.printf("Loading VGGNet16\n");
        break;
      case COM_LOADED:
        display.cls();
        display.printf("Ready to cut a  baumkuchen      \n");
        debugSerial.printf("Ready\n");
        break;
      case COM_RECEIVED:
        display.cls();
        display.printf("Receiving data..Received:%2d / %-2d\n", data[0], data[1]);
        debugSerial.printf("Received:%2d / %-2d\n", data[0], data[1]);
        break;
      case COM_STARTEDPREDICTING:
        display.cls();
        display.printf("Jetson nano is  now predicting..\n");
        debugSerial.printf("Predicting\n");
        break;
      case COM_SHUTDOWN:
        display.cls();
        display.printf("Jetson nano willbe turned off...\n");
        debugSerial.printf("JetsonWillShutdown\n");
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
  ManNotAutoButton.mode( PullUp );
  
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
  display.printf( "Firmware:JuliettUninitialized   " );
  
  return;
}

void InitializeStepper( void )
{
  stepperMotors.init();

  // Set 1st motor (VERTICAL1) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_VERTICAL_1, stepperMotors.calcMaxSpd( 200 ) );
  // Set 2nd motor (VERTICAL2) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_VERTICAL_2, stepperMotors.calcMaxSpd( 200 ) );
  // Set 3rd motor (ROTATE) max speed to 100steps/sec
  stepperMotors.setMaximumSpeed( SM_ROTATE, stepperMotors.calcMaxSpd( 30 ) );
  
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
        stepperMotors.run( SM_ROTATE, stepperMotors.calcSpd( 30 ), true ); // Clockwise
        isActed = true;
      }else if( IsRotateLeftButtonPressed ){
        stepperMotors.run( SM_ROTATE, stepperMotors.calcSpd( 30 ), false ); // Counter-Clockwise
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
        stepperMotors.move( SM_VERTICAL_1, 200*128*6, true );
        // Add a action to queue: Rotate Vertical2 2 times/Counter-Clockwise
        stepperMotors.move( SM_VERTICAL_2, 200*128*6, true );
        isActed = true;
      }
    }else if( IsDownButtonPressed ){ // Down
      if( !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) ){
        // Add a action to queue: Rotate Vertical1 2 times/Clockwise
        stepperMotors.move( SM_VERTICAL_1, 200*128*6, false );
        // Add a action to queue: Rotate Vertical2 2 times/Clockwise
        stepperMotors.move( SM_VERTICAL_2, 200*128*6, false );
      }
      isActed = true;
    }
    
  }
  
  return isActed;
}

// ISR
void ISR_AutoCutting( void )
{
  if( IsDuringAutoCuttingProcess && !stepperMotors.isBusy( SM_VERTICAL_1 ) && !stepperMotors.isBusy( SM_VERTICAL_2 ) && !stepperMotors.isBusy( SM_ROTATE ) ){
    switch( currentAutoCuttingState ){
    case STATE_IDLE: {
      currentAutoCuttingState = STATE_ROTATE;

      display.cls();
      display.printf( "State:    IdlingSteps:   %2d / %2d\n", 1, totalStepAmountCount * 3 + 1 );
      
      break;
    }
    case STATE_ROTATE: {
      uint32_t steps = stepAmountQueue.get();
      stepperMotors.move( SM_ROTATE, steps * 128, true );
      currentAutoCuttingState = STATE_DOWN;

      uint32_t cuttingRecipientNumber = totalStepAmountCount - stepAmountQueue.available();
      display.cls();
      display.printf( "State:  RotatingSteps:   %2d / %2d\n", 1 + (cuttingRecipientNumber - 1) * 3 + 1, totalStepAmountCount * 3 + 1 );
      
      break;
    }
    case STATE_DOWN: {
      uint8_t rotation = 6;
      uint32_t steps = ( uint32_t )( rotation * 200 * 128 );
      stepperMotors.move( SM_VERTICAL_1, steps, false );
      stepperMotors.move( SM_VERTICAL_2, steps, false );
      currentAutoCuttingState = STATE_UP;

      uint32_t cuttingRecipientNumber = totalStepAmountCount - stepAmountQueue.available();
      display.cls();
      display.printf( "State:   CuttingSteps:   %2d / %2d\n", 1 + (cuttingRecipientNumber - 1) * 3 + 2, totalStepAmountCount * 3 + 1 );
      
      break;
    }
    case STATE_UP: {
      uint8_t rotation = 6;
      uint32_t steps = ( uint32_t )( rotation * 200 * 128 );
      stepperMotors.move( SM_VERTICAL_1, steps, true );
      stepperMotors.move( SM_VERTICAL_2, steps, true );

      uint32_t cuttingRecipientNumber = totalStepAmountCount - stepAmountQueue.available();
      
      if( stepAmountQueue.available() <= 0 ){
        // it's the last action
        currentAutoCuttingState = STATE_ENDING;

        display.cls();
        display.printf( "State:    EndingSteps:   %2d / %2d\n", 1 + (cuttingRecipientNumber - 1) * 3 + 3, totalStepAmountCount * 3 + 1 );
      }else{
        // there are more actions to do
        currentAutoCuttingState = STATE_ROTATE;
        
        display.cls();
        display.printf( "State:   RaisingSteps:   %2d / %2d\n", 1 + (cuttingRecipientNumber - 1) * 3 + 3, totalStepAmountCount * 3 + 1 );
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
      display.printf("Ready to cut a  baumkuchen      \n");
      
      break;
    }
    default:
      break;
    }
  }
  
  return;
}

void ISR_PeriodicButtonWatcher( void )
{
  volatile static uint8_t UpButtonPressCount = 0;
  volatile static uint8_t DownButtonPressCount = 0;
  volatile static uint8_t RotateRightButtonPressCount = 0;
  volatile static uint8_t RotateLeftButtonPressCount = 0;
  volatile static uint8_t RotateExecButtonPressCount = 0;
  volatile static uint8_t EnableCtrlButtonPressCount = 0;
  volatile static uint8_t ManNotAutoButtonPressCount = 0;
  
  volatile static uint8_t UpButtonReleaseCount = 0;
  volatile static uint8_t DownButtonReleaseCount = 0;
  volatile static uint8_t RotateRightButtonReleaseCount = 0;
  volatile static uint8_t RotateLeftButtonReleaseCount = 0;
  volatile static uint8_t RotateExecButtonReleaseCount = 0;
  volatile static uint8_t EnableCtrlButtonReleaseCount = 0;
  volatile static uint8_t ManNotAutoButtonReleaseCount = 0;
  
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
  
  return;
}

void ISR_CheckButtons( void )
{
  if( IsUpButtonPressed ){
    debugSerial.printf("UpButton\n");
  }
  if( IsDownButtonPressed ){
    debugSerial.printf("DownButton\n");
  }
  if( IsRotateRightButtonPressed ){
    debugSerial.printf("RotateRightButton\n");
  }
  if( IsRotateLeftButtonPressed ){
    debugSerial.printf("RotateLeftButton\n");
  }
  if( IsRotateExecButtonPressed ){
    debugSerial.printf("RotateExecButton\n");
  }
  if( IsEnableCtrlButtonPressed ){
    debugSerial.printf("EnableCtrlButton\n");
  }
  if( IsAutoNotManButtonPressed ){
    debugSerial.printf("AutoNotManButton\n");
  }
}

