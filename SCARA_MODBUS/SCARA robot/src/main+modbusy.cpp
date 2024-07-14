/*
 * Program:      SCARA + modbus
 * Version:      V0
 * Date:         20-4-2024
 * Authors:      Tiemen van Rijswijk 
 * Project:      SCARA robotarm
 * Function:     Run SCARA basic functionality from previous program, now based on modbus communication.
 */

//Librarys
  #include <Arduino.h>
  #include <FlexyStepper.h>       //https://github.com/Stan-Reifel/FlexyStepper
  #include <Bounce2.h>            //https://github.com/thomasfredericks/Bounce2
  #include <Adafruit_PCF8574.h>   //https://github.com/adafruit/Adafruit_PCF8574/tree/main
  #include <ModbusSerial.h>       //https://github.com/epsilonrt/modbus-serial?tab=readme-ov-file
//

//Modbus
  // Modbus Registers
  const int RUNNING     = 0x1;   //(0001)
  const int T_REACHED   = 0x2;   //(0002)
  const int ERROR       = 0x3;   //(0003)
  const int HOMED       = 0x4;   //(0004)

  const int ENABLE_M    = 0x3E8;  //(1000)
  const int AK_ERROR    = 0x3E9;  //(1001)

  const int MODE        = 0xFA0;  //(4000)

  const int A_TARGET    = 0x1004;  //(4100)
  const int B_TARGET    = 0x1005;  //(4101)
  const int C_TARGET    = 0x1006;  //(4102)
  const int Z_TARGET    = 0x1007;  //(4103)

  const int A_SPEED     = 0x1008;  //(4104)
  const int B_SPEED     = 0x1009;  //(4105)
  const int C_SPEED     = 0x100A;  //(4106)
  const int Z_SPEED     = 0x100B;  //(4107)

  const int A_ACCEL     = 0x100C;  //(4108)
  const int B_ACCEL     = 0x100D;  //(4109)
  const int C_ACCEL     = 0x100E;  //(4110)
  const int Z_ACCEL     = 0x100F;  //(4111)

  const int ERROR_CODE  = 0xBC0;  //(3004)

  const int A_POS       = 0xBB8;  //(3000)
  const int B_POS       = 0xBB9;  //(3001)
  const int C_POS       = 0xBBA;  //(3002)
  const int Z_POS       = 0xBBB;  //(3003)

  const int STEP_A_POS  = 0xBBC;  //(3004)
  const int STEP_B_POS  = 0xBBD;  //(3005)
  const int STEP_C_POS  = 0xBBE;  //(3006)
  const int STEP_Z_POS  = 0xBBF;  //(3007)

  

  //Modbus connection parameters
  #define MySerial Serial // define serial port used, Serial most of the time, or Serial1, Serial2 ... if available
  const unsigned long Baudrate = 19200;
  const byte SlaveId = 24;
  ModbusSerial mb (MySerial, SlaveId, -1);
//

//Initiate components
  #define limitSwitch1 A2  //A-as
  Bounce2::Button A = Bounce2::Button();

  Adafruit_PCF8574 pcf;

  FlexyStepper stepper_A;  // A-as
  FlexyStepper stepper_B;  // B-as
  FlexyStepper stepper_C;  // C-as
  FlexyStepper stepper_Z;  // Z-as
//

//Variables
  //motor variables;
  int16_t         stepper_A_Position, stepper_B_Position, stepper_C_Position, stepper_Z_Position;

  const float A_steps_degree  = 2.4;          // A-as
  const float B_steps_degree  = 1.657;        // B-as 
  const float C_steps_degree  = 2.083;        // C-as
  const float Z_steps_mm      = 3.25;         // Z-as
  const float Z_steps_degree  = 0.4514;       // Z-as graden            
  const float Z_degree_mm     = 0.0139;       // Z-as graden/milimeters

//


//FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
  void homing() {
    bool A_Homed = false;
    bool B_Homed = false;
    bool C_Homed = false;
    bool Z_Homed = false;

    //set homing speeds
      stepper_A.setSpeedInStepsPerSecond(200);
      stepper_B.setSpeedInStepsPerSecond(200);
      

      stepper_A.setAccelerationInStepsPerSecondPerSecond(2000);
      stepper_B.setAccelerationInStepsPerSecondPerSecond(2000);

      

   

    //Homing Z
      while (Z_Homed == false){
        //Move up
        stepper_Z.setAccelerationInStepsPerSecondPerSecond(10000);
        stepper_Z.setSpeedInStepsPerSecond(200);
        stepper_Z.setTargetPositionInSteps(5362);

        while (!stepper_Z.motionComplete() && !(!pcf.digitalRead(0)==true || !pcf.digitalRead(1)==true)) {
          stepper_Z.processMovement();
          mb.task();
        }
        delay(200);

        //Tighten spindle 
        stepper_Z.moveRelativeInSteps(5);
        delay(200);

        //check if top is reached
        if ((!pcf.digitalRead(0)==true) && (!pcf.digitalRead(1)==true)){
          stepper_Z.setCurrentPositionInSteps(0);
          Z_Homed = true;
        } else {
        //Turn spindle 60 degrees
        stepper_Z.moveRelativeInSteps(-100); //move down

        stepper_C.setAccelerationInStepsPerSecondPerSecond(1000);
        stepper_Z.setAccelerationInStepsPerSecondPerSecond(500);

        stepper_C.setSpeedInStepsPerSecond(400);
        stepper_Z.setSpeedInStepsPerSecond(86.68); 
        
        stepper_C.setCurrentPositionInSteps(0);
        stepper_Z.setCurrentPositionInSteps(0);

        stepper_C.setTargetPositionInSteps( 940 ); 
        stepper_Z.setTargetPositionInSteps( -200 );
          
        while (!stepper_C.motionComplete() || !stepper_Z.motionComplete()) {
          stepper_C.processMovement();
          stepper_Z.processMovement();
        }
        }
        delay(200);
      }

    //Homing C
      stepper_Z.moveRelativeInSteps(-35); // move down to clear bottom
      delay(200);

      stepper_C.setAccelerationInStepsPerSecondPerSecond(1000);
      stepper_Z.setAccelerationInStepsPerSecondPerSecond(500);

      stepper_C.setSpeedInStepsPerSecond(200);
      stepper_Z.setSpeedInStepsPerSecond(43.34); 
      
      stepper_C.setCurrentPositionInSteps(0);
      stepper_Z.setCurrentPositionInSteps(0);

      stepper_C.setTargetPositionInSteps( -9400 ); 
      stepper_Z.setTargetPositionInSteps( 2000 );
          
      while ((!stepper_C.motionComplete() || !stepper_Z.motionComplete()) && !(!pcf.digitalRead(0)==true || !pcf.digitalRead(1)==true)) {
        stepper_C.processMovement();
        stepper_Z.processMovement();
      }
      delay(200);

      if ((!pcf.digitalRead(0)==true) && (!pcf.digitalRead(1)==false)){
          C_Homed = true;
          stepper_C.setCurrentPositionInSteps(0);
          stepper_Z.setCurrentPositionInSteps(0);
      } else if ((!pcf.digitalRead(0)==false) && (!pcf.digitalRead(1)==true)){
          C_Homed = true;
          stepper_C.setCurrentPositionInSteps(940);
          stepper_Z.setCurrentPositionInSteps(0);
      } else {
          C_Homed = false;
      }
      delay(200);



    //Homing B
      delay(200);
      
      stepper_B.setTargetPositionInSteps( 3833 );
      while ((!stepper_B.motionComplete()) and pcf.digitalRead(2) != false){
        stepper_B.processMovement();
        mb.task();
      }
      stepper_B.setCurrentPositionInSteps(1800);

    //Homing A
      delay(200);
      
      stepper_A.setTargetPositionInSteps( 4500 );
      while ((!stepper_A.motionComplete()) and A.pressed() != true){
        stepper_A.processMovement();
        A.update();
        mb.task();
      }
      stepper_A.setCurrentPositionInSteps(2130);
  
    //Move axis to default position
      delay(200);
      stepper_A.setSpeedInStepsPerSecond( 400 );
      stepper_B.setSpeedInStepsPerSecond( 400 );
      stepper_A.setAccelerationInStepsPerSecondPerSecond( 200 );
      stepper_B.setAccelerationInStepsPerSecondPerSecond( 200 );

      stepper_A.setTargetPositionInSteps( 0 );
      stepper_B.setTargetPositionInSteps( 0 );

      while ((!stepper_A.motionComplete()) || (!stepper_B.motionComplete()))
      {
        stepper_A.processMovement();
        stepper_B.processMovement();
      }
  }

  void Set_values(){

    stepper_A_Position = (int)mb.hreg(A_TARGET) * A_steps_degree;
    stepper_B_Position = (int)mb.hreg(B_TARGET) * B_steps_degree;
    stepper_C_Position = (int)mb.hreg(C_TARGET) * C_steps_degree;
    stepper_Z_Position = ((int)mb.hreg(Z_TARGET) * Z_steps_mm) - ((int)mb.hreg(C_TARGET) * Z_steps_degree);

    stepper_A.setTargetPositionInSteps(stepper_A_Position);
    stepper_B.setTargetPositionInSteps(stepper_B_Position);
    stepper_C.setTargetPositionInSteps(stepper_C_Position);
    stepper_Z.setTargetPositionInSteps(stepper_Z_Position);

    stepper_A.setSpeedInStepsPerSecond(mb.hreg(A_SPEED) * A_steps_degree);
    stepper_B.setSpeedInStepsPerSecond(mb.hreg(B_SPEED) * B_steps_degree);
    stepper_C.setSpeedInStepsPerSecond(mb.hreg(C_SPEED) * C_steps_degree);
    stepper_Z.setSpeedInStepsPerSecond(mb.hreg(Z_SPEED) * Z_steps_mm);

    stepper_A.setAccelerationInStepsPerSecondPerSecond(mb.hreg(A_ACCEL) * A_steps_degree);
    stepper_B.setAccelerationInStepsPerSecondPerSecond(mb.hreg(B_ACCEL) * B_steps_degree);
    stepper_C.setAccelerationInStepsPerSecondPerSecond(mb.hreg(C_ACCEL) * C_steps_degree);
    stepper_Z.setAccelerationInStepsPerSecondPerSecond(mb.hreg(Z_ACCEL) * Z_steps_mm);
  }

  void Set_ERROR(int error, bool status){
    //Set error Input register
    if (status == true){
      mb.setIreg(ERROR_CODE, (mb.ireg(ERROR_CODE) | (1 << error)));
    }
    else{
      mb.setIreg(ERROR_CODE, (mb.ireg(ERROR_CODE) ^ (1 << error)));
    }
    //Set ERROR coil
    if (mb.ireg(ERROR_CODE) > 0){
      mb.setIsts(ERROR,true);
    }
    else{
      mb.setIsts(ERROR,false);
    }

  }
//


void setup() {

  //Modbus
    MySerial.begin (Baudrate, SERIAL_8E1);
    mb.config (Baudrate);
    mb.setAdditionalServerData ("SCARA"); // for Report Server ID function (0x11)

    // configure Registers
    mb.addIsts (RUNNING   , false);
    mb.addIsts (T_REACHED , false);
    mb.addIsts (ERROR     , false);
  
    mb.addCoil (ENABLE_M  , false);
    mb.addHreg (MODE, 0);
  
    mb.addHreg (A_TARGET, 0);
    mb.addHreg (B_TARGET, 0);
    mb.addHreg (C_TARGET, 0);
    mb.addHreg (Z_TARGET, 0);

    mb.addHreg (A_SPEED , 100);
    mb.addHreg (B_SPEED , 100);
    mb.addHreg (C_SPEED , 100);
    mb.addHreg (Z_SPEED , 100);

    mb.addHreg (A_ACCEL , 50);
    mb.addHreg (B_ACCEL , 50);
    mb.addHreg (C_ACCEL , 50);
    mb.addHreg (Z_ACCEL , 50);
    
    mb.addIreg (A_POS   , 0);
    mb.addIreg (B_POS   , 0);
    mb.addIreg (C_POS   , 0);
    mb.addIreg (Z_POS   , 0);
    mb.addIreg (STEP_A_POS   , 0);
    mb.addIreg (STEP_B_POS   , 0);
    mb.addIreg (STEP_C_POS   , 0);
    mb.addIreg (STEP_Z_POS   , 0);

    mb.addIreg (ERROR_CODE , 0);
  //

  //Switches
    A.attach(limitSwitch1, INPUT_PULLUP);  
    A.interval(2);
    A.setPressedState(LOW);

    if (!pcf.begin(0x21, &Wire)){
      Set_ERROR(1,true);
    }
    else{
      pcf.pinMode(0, INPUT_PULLUP);
      pcf.pinMode(1, INPUT_PULLUP);
      pcf.pinMode(2, INPUT_PULLUP);
    }
  //

  //Motors
    stepper_A.connectToPins(2, 5);   //A-as
    stepper_B.connectToPins(3, 6);   //B-as
    stepper_C.connectToPins(4, 7);   //C-as
    stepper_Z.connectToPins(12, 13); //Z-as

    pinMode(8, OUTPUT); //Enable pin
  //


  //Homing
  homing();
  // 

}

void loop() {
  //update data
    mb.task();

  //Write data
    digitalWrite (8, !mb.Coil(ENABLE_M));
    Set_values();

    // switch 

    //   case


  //Movement loop
    while ((!stepper_A.motionComplete()) || (!stepper_B.motionComplete()) || (!stepper_C.motionComplete()) || (!stepper_Z.motionComplete()) || (mb.Coil (ENABLE_M))) {
      mb.task();
      Set_values();
      mb.setIsts(RUNNING, true);
      mb.setIsts(T_REACHED, false);

      stepper_A.processMovement();
      stepper_B.processMovement();
      stepper_C.processMovement();
      stepper_Z.processMovement();
      
      mb.setIreg(STEP_A_POS, stepper_A.getCurrentPositionInSteps());
      mb.setIreg(STEP_C_POS, stepper_B.getCurrentPositionInSteps());
      mb.setIreg(STEP_B_POS, stepper_C.getCurrentPositionInSteps());
      mb.setIreg(STEP_Z_POS, stepper_Z.getCurrentPositionInSteps());
    }

    mb.setIsts(RUNNING, false);
    mb.setIsts(T_REACHED, true);
}
