#include "Motion_Controller.h"
#include <Arduino.h>

#include "MOD_VAR.h"
#include "Parameters.h"

Motion_Controller::Motion_Controller(ESP_FlexyStepper & a, ESP_FlexyStepper & b, ESP_FlexyStepper & c, ESP_FlexyStepper & z)
    : stepper_A(a), stepper_B(b), stepper_C(c), stepper_Z(z)
{}

void Motion_Controller::Set_Params(){
    // Calculate target positions in steps
    int stepper_A_Position = (int)mb.hreg(MB_A_TARGET) * A_steps_degree;
    int stepper_B_Position = (int)mb.hreg(MB_B_TARGET) * B_steps_degree;
    int stepper_C_Position = (int)mb.hreg(MB_C_TARGET) * C_steps_degree;
    int stepper_Z_Position = ((int)mb.hreg(MB_Z_TARGET) * Z_steps_mm) - ((int)mb.hreg(MB_C_TARGET) * Z_steps_degree);
    //set target positions
    stepper_A.setTargetPositionInSteps(stepper_A_Position);
    stepper_B.setTargetPositionInSteps(stepper_B_Position);
    stepper_C.setTargetPositionInSteps(stepper_C_Position);
    stepper_Z.setTargetPositionInSteps(stepper_Z_Position);
    //set speeds
    stepper_A.setSpeedInStepsPerSecond(mb.hreg(MB_A_VEl) * A_steps_degree);
    stepper_B.setSpeedInStepsPerSecond(mb.hreg(MB_B_VEL) * B_steps_degree);
    stepper_C.setSpeedInStepsPerSecond(mb.hreg(MB_C_VEL) * C_steps_degree);
    stepper_Z.setSpeedInStepsPerSecond(mb.hreg(MB_Z_VEL) * Z_steps_mm);
    //set accelerations
    stepper_A.setAccelerationInStepsPerSecondPerSecond(mb.hreg(MB_A_ACC) * A_steps_degree);
    stepper_B.setAccelerationInStepsPerSecondPerSecond(mb.hreg(MB_B_ACC) * B_steps_degree);
    stepper_C.setAccelerationInStepsPerSecondPerSecond(mb.hreg(MB_C_ACC) * C_steps_degree);
    stepper_Z.setAccelerationInStepsPerSecondPerSecond(mb.hreg(MB_Z_ACC) * Z_steps_mm);
    //set decelerations
    stepper_A.setDecelerationInStepsPerSecondPerSecond(mb.hreg(MB_A_DEC) * A_steps_degree);
    stepper_B.setDecelerationInStepsPerSecondPerSecond(mb.hreg(MB_B_DEC) * B_steps_degree);
    stepper_C.setDecelerationInStepsPerSecondPerSecond(mb.hreg(MB_C_DEC) * C_steps_degree);
    stepper_Z.setDecelerationInStepsPerSecondPerSecond(mb.hreg(MB_Z_DEC) * Z_steps_mm);
}


bool Motion_Controller::Target_Reached() {
    if (stepper_A.motionComplete() && stepper_B.motionComplete() && stepper_C.motionComplete() && stepper_Z.motionComplete()) {
        return true;
    } else {
        return false;
    }
}

bool Motion_Controller::Standstill() {
    if (stepper_A.getCurrentVelocityInStepsPerSecond() == 0 && stepper_B.getCurrentVelocityInStepsPerSecond() == 0 && stepper_C.getCurrentVelocityInStepsPerSecond() == 0 && stepper_Z.getCurrentVelocityInStepsPerSecond() == 0) {
        return true;
    } else {
        return false;
    }
}

bool Motion_Controller::Homing() {

    // bool A_Homed = false;
    // bool B_Homed = false;
    // bool C_Homed = false;
    // bool Z_Homed = false;


    // //set homing speeds
    //     stepper_A.setSpeedInStepsPerSecond(200);
    //     stepper_B.setSpeedInStepsPerSecond(200);

    //     stepper_Z.setSpeedInStepsPerSecond(200);

    //     stepper_A.setAccelerationInStepsPerSecondPerSecond(2000);
    //     stepper_B.setAccelerationInStepsPerSecondPerSecond(2000);

    // stepper_Z.setAccelerationInStepsPerSecondPerSecond(10000);

    // //Homing Z
    //     while (Z_Homed == false){
    //     //Move up
    //     stepper_Z.setTargetPositionInSteps(5362);

    //     while (!stepper_Z.motionComplete() && !(!pcf.digitalRead(0)==true || !pcf.digitalRead(1)==true)) {
    //         stepper_Z.processMovement();
    //         mb.task();
    //     }
    //     delay(200);

    //     //Tighten spindle 
    //     stepper_Z.moveRelativeInSteps(5);
    //     delay(200);

    //     //check if top is reached
    //     if ((!pcf.digitalRead(0)==true) && (!pcf.digitalRead(1)==true)){
    //         stepper_Z.setCurrentPositionInSteps(0);
    //         Z_Homed = true;
    //     } else {
    //     //Turn spindle 60 degrees
    //     stepper_Z.moveRelativeInSteps(-100); //move down

    //     stepper_C.setAccelerationInStepsPerSecondPerSecond(1000);
    //     stepper_Z.setAccelerationInStepsPerSecondPerSecond(500);

    //     stepper_C.setSpeedInStepsPerSecond(400);
    //     stepper_Z.setSpeedInStepsPerSecond(86.68); 
        
    //     stepper_C.setCurrentPositionInSteps(0);
    //     stepper_Z.setCurrentPositionInSteps(0);

    //     stepper_C.setTargetPositionInSteps( 940 ); 
    //     stepper_Z.setTargetPositionInSteps( -200 );
            
    //     while (!stepper_C.motionComplete() || !stepper_Z.motionComplete()) {
    //         stepper_C.processMovement();
    //         stepper_Z.processMovement();
    //     }
    //     }
    //     delay(200);
    //     }

    // //Homing C
    //     stepper_Z.moveRelativeInSteps(-35); // move down to clear bottom
    //     delay(200);

    //     stepper_C.setAccelerationInStepsPerSecondPerSecond(1000);
    //     stepper_Z.setAccelerationInStepsPerSecondPerSecond(500);

    //     stepper_C.setSpeedInStepsPerSecond(200);
    //     stepper_Z.setSpeedInStepsPerSecond(43.34); 
        
    //     stepper_C.setCurrentPositionInSteps(0);
    //     stepper_Z.setCurrentPositionInSteps(0);

    //     stepper_C.setTargetPositionInSteps( -9400 ); 
    //     stepper_Z.setTargetPositionInSteps( 2000 );
            
    //     while ((!stepper_C.motionComplete() || !stepper_Z.motionComplete()) && !(!pcf.digitalRead(0)==true || !pcf.digitalRead(1)==true)) {
    //     stepper_C.processMovement();
    //     stepper_Z.processMovement();
    //     }
    //     delay(200);

    //     if ((!pcf.digitalRead(0)==true) && (!pcf.digitalRead(1)==false)){
    //         C_Homed = true;
    //         stepper_C.setCurrentPositionInSteps(0);
    //         stepper_Z.setCurrentPositionInSteps(0);
    //     } else if ((!pcf.digitalRead(0)==false) && (!pcf.digitalRead(1)==true)){
    //         C_Homed = true;
    //         stepper_C.setCurrentPositionInSteps(940);
    //         stepper_Z.setCurrentPositionInSteps(0);
    //     } else {
    //         C_Homed = false;
    //     }
    //     delay(200);



    // //Homing B
    //     delay(200);
        
    //     stepper_B.setTargetPositionInSteps( 3833 );
    //     while ((!stepper_B.motionComplete()) and pcf.digitalRead(2) != false){
    //     stepper_B.processMovement();
    //     mb.task();
    //     }
    //     stepper_B.setCurrentPositionInSteps(1800);

    // //Homing A
    //     delay(200);
        
    //     stepper_A.setTargetPositionInSteps( 4500 );
    //     while ((!stepper_A.motionComplete()) and A.pressed() != true){
    //     stepper_A.processMovement();
    //     A.update();
    //     mb.task();
    //     }
    //     stepper_A.setCurrentPositionInSteps(2130);

    // //Move axis to default position
    //     delay(200);
    //     stepper_A.setSpeedInStepsPerSecond( 400 );
    //     stepper_B.setSpeedInStepsPerSecond( 400 );
    //     stepper_A.setAccelerationInStepsPerSecondPerSecond( 200 );
    //     stepper_B.setAccelerationInStepsPerSecondPerSecond( 200 );

    //     stepper_A.setTargetPositionInSteps( 0 );
    //     stepper_B.setTargetPositionInSteps( 0 );

    //     while ((!stepper_A.motionComplete()) || (!stepper_B.motionComplete()))
    //     {
    //     stepper_A.processMovement();
    //     stepper_B.processMovement();
    //     }

    return false;
}

void Motion_Controller::Process_Movement() {
    stepper_A.processMovement();
    stepper_B.processMovement();
    stepper_C.processMovement();
    stepper_Z.processMovement();
}


void Motion_Controller::Set_Stop() {
    stepper_A.setTargetPositionToStop();
    stepper_B.setTargetPositionToStop();
    stepper_C.setTargetPositionToStop();
    stepper_Z.setTargetPositionToStop();
}

void Motion_Controller::Reset_Stop() {
    stepper_A.releaseEmergencyStop();
    stepper_B.releaseEmergencyStop();
    stepper_C.releaseEmergencyStop();
    stepper_Z.releaseEmergencyStop();
}