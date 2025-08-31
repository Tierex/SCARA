// File     | src/main.cpp
// Project  | SCARA Robot ESP32
// Date     | 03-08-2025
// Revision | 1.0

//LIB & VARS//////////////////////////////////////////////////////////////////////////////////////
  //Include libraries 
  #include <Arduino.h>
  #include <ESP_FlexyStepper.h>    //https://github.com/pkerspe/ESP-FlexyStepper
  #include <Bounce2.h>             //https://github.com/thomasfredericks/Bounce2
  #include <Adafruit_PCF8574.h>    //https://github.com/adafruit/Adafruit_PCF8574/tree/main
  #include <AS5600.h>              //https://github.com/RobTillaart/AS5600?utm_source=platformio&utm_medium=piohome
  #include <ModbusSerial.h>        //https://github.com/epsilonrt/modbus-serial?tab=readme-ov-file

  //Include header files
  #include "Parameters.h"       //Project parameters
  #include "MOD_VAR.h"          //Modbus Registers/adresses
  #include "PIN_Definitions.h"  //Pin definitions
  #include "STATES.h"           //Robot State definitions


  //Include custom libraries
  #include "Modbus_Controller.h"  //Modbus controller
  #include "Motion_Controller.h"  //Motion controller
  #include "LED_Controller.h"     //LED ring controller
  #include "FAN_Controller.h"     //FAN controller

  //Global Variables
  bool Stop_Initiated = false;

  //Define Objects
  ModbusSerial mb (Serial, SlaveId, -1);
  Modbus_Controller mb_controller(&mb);

  ESP_FlexyStepper  stepper_A;  // A-axis
  ESP_FlexyStepper  stepper_B;  // B-axis
  ESP_FlexyStepper  stepper_C;  // C-axis
  ESP_FlexyStepper  stepper_Z;  // Z-axis

  Motion_Controller Motion(stepper_A, stepper_B, stepper_C, stepper_Z);

  LED_Controller LED_Ring(PIN_LED);
  FAN_Controller FAN(PIN_FAN);
  Adafruit_PCF8574 pcf;
  AS5600 as5600;
  

//FUNCTIONS/////////////////////////////////////////////////////////////////////////////////////
bool Valid_State_Transition(int currentState, int newState) { // Check if transition from currentState to newState is valid/expected
    switch (currentState) {
        case ST_ERROR_STOP:
            return newState == ST_DISABLED || newState == ST_STANDBY;
        case ST_DISABLED:
            return newState == ST_STANDBY;
        case ST_STANDBY:
            return newState == ST_MOVING  || newState == ST_HOMING;
        case ST_MOVING:
            return newState == ST_STOPPING || newState == ST_STANDBY;
        case ST_STOPPING:
            return newState == ST_ERROR_STOP;
        case ST_HOMING:
            return newState == ST_STANDBY;
        default:
            return false;
    }
}


//SETUP/////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //LED Ring
  LED_Ring.begin();
  LED_Ring.startCycle();

  //Modbus
  Serial.begin (Baudrate, SERIAL_8E1);
  mb.config (Baudrate);
  mb.setAdditionalServerData ("SCARA");
  mb_controller.Init_Registers();
  delay(100); //Wait for Modbus to initialize

  //Steppers
  stepper_A.connectToPins(PIN_STEP_A, PIN_DIR_A);   //A-axis
  stepper_B.connectToPins(PIN_STEP_B, PIN_DIR_B);   //B-axis
  stepper_C.connectToPins(PIN_STEP_C, PIN_DIR_C);   //C-axis
  stepper_Z.connectToPins(PIN_STEP_Z, PIN_DIR_Z);   //Z-axis
  
  //Pinmodes
  pinMode(PIN_ENABLE,   OUTPUT);
  pinMode(PIN_FAN,      OUTPUT);
  pinMode(PIN_POWER_S,  INPUT);
  pinMode(PIN_LM_A,     INPUT_PULLUP);

  //Initial Pin States
  digitalWrite(PIN_FAN, LOW); //fan off
  digitalWrite(PIN_ENABLE, HIGH); //disable steppers
  
  //Setup finished, continue to loop
  mb.setHreg(MB_STATE, ST_DISABLED); //Set initial state to disabled
}



//MAIN LOOP////////////////////////////////////////////////////////////////////////////////////
void loop() {
//Update Modbus
  mb.task();

//Main State Machine
  switch (mb.Hreg(MB_STATE)) { 
    case ST_ERROR_STOP:
      digitalWrite(PIN_ENABLE, HIGH);
      Motion.Reset_Stop();
    break;


    case ST_DISABLED:
      digitalWrite(PIN_ENABLE, HIGH);
      Motion.Set_Params();

    break;


    case ST_STANDBY:
      digitalWrite(PIN_ENABLE, LOW);
      Motion.Set_Params();

    break;
    

    case ST_STOPPING:
      digitalWrite(PIN_ENABLE, LOW);
      if (!Stop_Initiated) {
        Motion.Set_Stop();
        Stop_Initiated = true;
      }

      Motion.Process_Movement();

      if (Motion.Standstill()){ //If all steppers have stopped
        delay(200); //short delay to ensure complete stop
        Stop_Initiated = false;
        mb.setHreg(MB_STATE, ST_ERROR_STOP);
      }
    break;

    case ST_MOVING:
      digitalWrite(PIN_ENABLE, LOW);
      Motion.Set_Params();
      Motion.Process_Movement();

      if (Motion.Target_Reached()){ //If all steppers have reached their target
        mb.setHreg(MB_STATE, ST_STANDBY);
      }
    break;


    case ST_HOMING:
      digitalWrite(PIN_ENABLE, LOW);
    break;
 
    default:
      digitalWrite(PIN_ENABLE, HIGH);
      mb.setHreg(MB_STATE, ST_ERROR_STOP);
    break;
  }

  // Update the LED ring 
  LED_Ring.setStatus(mb.Hreg(MB_STATE));
}
