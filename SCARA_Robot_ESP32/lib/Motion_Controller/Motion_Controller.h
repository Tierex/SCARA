#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include <ESP_FlexyStepper.h>
#include <ModbusSerial.h>
#include <MOD_VAR.h>
#include "Parameters.h"

class Motion_Controller {
public:
    Motion_Controller(ESP_FlexyStepper & a, ESP_FlexyStepper & b, ESP_FlexyStepper & c, ESP_FlexyStepper & z);
    
    void Set_Params();
    void Process_Movement();
    bool Homing();
    bool Target_Reached();
    bool Standstill();
    void Set_Stop();
    void Reset_Stop();

private:
    ESP_FlexyStepper & stepper_A;
    ESP_FlexyStepper & stepper_B;
    ESP_FlexyStepper & stepper_C;
    ESP_FlexyStepper & stepper_Z;
};


#endif