#include <Arduino.h>
#include <FlexyStepper.h>
#include <Bounce2.h>
#include <Adafruit_PCF8574.h>

//define components;
#define limitSwitch1 A2  //A-as
Bounce2::Button A = Bounce2::Button();

Adafruit_PCF8574 pcf;

FlexyStepper stepper_A;  // A-as
FlexyStepper stepper_B;  // B-as
FlexyStepper stepper_C;  // C-as
FlexyStepper stepper_Z;  // Z-as

//motor variables;
int         stepper_A_Position, stepper_B_Position, stepper_C_Position, stepper_Z_Position;

const float A_steps_degree  = 24;           // A-as
const float B_steps_degree  = 16.6667;      // B-as
const float C_steps_degree  = 20.8333;      // C-as
const float Z_steps_mm      = 162.5;        // Z-as
const float Z_steps_degree  = 4.5138;       // Z-as graden
const float Z_degree_mm     = 0.0277777;    // Z-as graden/milimeters

//process variables;
float A_Current_Position    = 0;
float B_Current_Position    = 0;
float C_Current_Position    = 0;
float Z_Current_Position    = 0;         

int   Robot_status          = 0;   //0 = Initializing //1 = ready for new command //2 = moving //3 = homing
bool  status_send           = false;
int   i                     = 0;
//communication variables
int   EnableRobot           = 0;
float motorData[12];
int   GripperData           = 0;
bool  HommingData           = false;

bool  newData               = false;
bool  DataAssigned          = true;

const byte  numChars        = 92;
char        receivedChars[numChars];
char        tempChars[numChars]; 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void homing() {
  //set homing speeds
    stepper_A.setSpeedInStepsPerSecond(200);
    stepper_B.setSpeedInStepsPerSecond(200);
    stepper_C.setSpeedInStepsPerSecond(400);
    stepper_Z.setSpeedInStepsPerSecond(400);
    
    stepper_A.setAccelerationInStepsPerSecondPerSecond(2000);
    stepper_B.setAccelerationInStepsPerSecondPerSecond(2000);
    stepper_C.setAccelerationInStepsPerSecondPerSecond(5000);
    stepper_Z.setAccelerationInStepsPerSecondPerSecond(5000);

  //Homing Z
    Serial.println("Starting Z");
    stepper_Z.setTargetPositionInSteps( -26812 );
    // while ((!stepper_Z.motionComplete()) and Z.pressed() != true)
    Serial.println(1);
    while ((!stepper_Z.motionComplete()) and pcf.digitalRead(0) != false) 
    {
      stepper_Z.processMovement();
      Serial.println(2);
      // Z.update();
    }
    Serial.println("Z is done");

  // //Tension spindel
  //   delay(200);
  //   stepper_Z.moveRelativeInSteps(10);
    
  //Homing C
    Serial.println("Starting C");
    delay(200);

    stepper_C.setTargetPositionInSteps( -6542 );
    stepper_Z.setTargetPositionInSteps(  1417 );
    stepper_C.setSpeedInStepsPerSecond(700);
    stepper_Z.setSpeedInStepsPerSecond(2);
    
    // while ((!stepper_C.motionComplete()) and C.pressed() != true)
    while ((!stepper_C.motionComplete()) and pcf.digitalRead(1) == false)
    {
      stepper_C.processMovement();
      stepper_Z.processMovement();
      // C.update();
    }
    Serial.println("C is done");

  //Move C to zero and Z to 40  
    delay(200);

    stepper_Z.setSpeedInStepsPerSecond(600);
    stepper_Z.setCurrentPositionInSteps(0);
    stepper_Z.moveToPositionInSteps(5791);
    stepper_Z.setCurrentPositionInSteps(6500);

    stepper_C.setCurrentPositionInSteps(-1395);
    stepper_C.moveToPositionInSteps(0);


  //Homing B
    Serial.println("Starting B");
    delay(200);
    
    stepper_B.setTargetPositionInSteps( 3833 );
    // while ((!stepper_B.motionComplete()) and B.pressed() != true )
    while ((!stepper_B.motionComplete()) and pcf.digitalRead(2) != false)
    {
      stepper_B.processMovement();
      // B.update();
    }
    stepper_B.setCurrentPositionInSteps(1800);
    Serial.println("B is done");


  //Homing A
    Serial.println("Starting A");
    delay(200);
    
    stepper_A.setTargetPositionInSteps( 4500 );
    while ((!stepper_A.motionComplete()) and A.pressed() != true)
    {
      stepper_A.processMovement();
      A.update();
    }
    stepper_A.setCurrentPositionInSteps(2130);
    Serial.println("A is done");



  //Move A and B to zero
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

void receive_data() {
    static boolean recvInProgress = false;  //function variables
    static byte ndx = 0;
    char   startMarker = '<';
    char   endMarker = '>';
    char   rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read(); //read incoming data

        if (recvInProgress == true) {       //save incoming data
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) { //check if data is correct
            recvInProgress = true;
        }
    }

  if (newData == true) {                        //only peform when new data
        strcpy(tempChars, receivedChars);
        char * strtokIndx;                      // this temporary copy is necessary to protect the original data

        strtokIndx = strtok(tempChars,",");     // get the first part - the string
        EnableRobot = atoi(strtokIndx); 

        for (int K=0; K<=11; K++){               //put data in array
         strtokIndx = strtok(NULL, ",");
        motorData[K] = atof(strtokIndx);     
        }
    
        strtokIndx = strtok(NULL,",");
        GripperData = atoi(strtokIndx);
        strtokIndx = strtok(NULL,",");
        if ( atoi(strtokIndx) == 1){
          HommingData = true;
          }

        newData = false;
        DataAssigned = false;
    }
}

void Set_values(){
  stepper_A.setSpeedInStepsPerSecond(motorData[0] * A_steps_degree);
  stepper_B.setSpeedInStepsPerSecond(motorData[1] * B_steps_degree);
  stepper_C.setSpeedInStepsPerSecond(motorData[2] * C_steps_degree);
  stepper_Z.setSpeedInStepsPerSecond(motorData[3] * Z_steps_mm);

  stepper_A.setAccelerationInStepsPerSecondPerSecond(motorData[4] * A_steps_degree);
  stepper_B.setAccelerationInStepsPerSecondPerSecond(motorData[5] * B_steps_degree);
  stepper_C.setAccelerationInStepsPerSecondPerSecond(motorData[6] * C_steps_degree);
  stepper_Z.setAccelerationInStepsPerSecondPerSecond(motorData[7] * Z_steps_mm);

  stepper_A_Position = 1 * motorData[8] * A_steps_degree;
  stepper_B_Position = 1 * motorData[9] * B_steps_degree;
  stepper_C_Position = 1 * motorData[10] * C_steps_degree;
  stepper_Z_Position =-1 * motorData[11] * Z_steps_mm + -1 * motorData[10] * Z_steps_degree;

  stepper_A.setTargetPositionInSteps(stepper_A_Position);
  stepper_B.setTargetPositionInSteps(stepper_B_Position);
  stepper_C.setTargetPositionInSteps(stepper_C_Position);
  stepper_Z.setTargetPositionInSteps(stepper_Z_Position);

  DataAssigned = true;
}

void Send_status(int Robot_status){

  bool Serial_status_send = false;
    while (Serial.availableForWrite()>0 && Serial_status_send == false){
      Serial.print("<");
      Serial.print(Robot_status);
      Serial.print(",");
      Serial.print(A_Current_Position,1);
      Serial.print(",");
      Serial.print(B_Current_Position,1);
      Serial.print(",");
      Serial.print(C_Current_Position,1);
      Serial.print(",");
      Serial.print(Z_Current_Position,1);
      Serial.println(">");

      Serial_status_send = true;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println(__FILE__);

  Send_status(0);

  if (!pcf.begin(0x21, &Wire)) {
  Serial.println("Couldn't find PCF8574");
  while (1);
  }


  //switch setup
    A.attach(limitSwitch1, INPUT_PULLUP);  
    A.interval(2);
    A.setPressedState(LOW);

    pcf.pinMode(0, INPUT_PULLUP);
    pcf.pinMode(1, INPUT_PULLUP);
    pcf.pinMode(2, INPUT_PULLUP);

  //stepper setup
    stepper_A.connectToPins(2, 5);   //A-as
    stepper_B.connectToPins(3, 6);   //B-as
    stepper_C.connectToPins(4, 7);   //C-as
    stepper_Z.connectToPins(12, 13); //Z-as

  delay(1000);

  //Homing
    Send_status(3);
    homing();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //extract data
    receive_data();
    // Serial.println("Data received");
    

  //set stepper values
    if (DataAssigned == false){
      Set_values();
      //Serial.println("Data assigned");
    }

  //enable mid-process homing
    if (HommingData == true) {
        memset(motorData, 0, sizeof(motorData));
        GripperData = 0;

        Send_status(4);
        homing();
        
        HommingData = false;
      }


  //process movement
    while (((!stepper_A.motionComplete()) || (!stepper_B.motionComplete()) || (!stepper_C.motionComplete()) || (!stepper_Z.motionComplete())) && EnableRobot == 1 ) {
      stepper_A.processMovement();
      stepper_B.processMovement();
      stepper_C.processMovement();
      stepper_Z.processMovement();
    
      status_send=false;
      
    //Read new incoming data;
      receive_data();
    //assign new values;
      if (DataAssigned == false){
        Set_values();
      } 

    }
  
  //send_status
  if (status_send == false){
    Send_status(1);
    status_send = true;
  }

  delay(10);
      
}

