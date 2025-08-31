#include "FAN_Controller.h"
#include <Arduino.h>
#include <ModbusSerial.h>
#include "MOD_VAR.h"


FAN_Controller::FAN_Controller(int pin) : fanPin(pin) {
    pinMode(fanPin, OUTPUT);
    digitalWrite(fanPin, LOW); // Ensure fan is off initially
}

void FAN_Controller::turnOn() {
    digitalWrite(fanPin, HIGH);
    fanState = true;
    mb.setIsts(FAN_ON, true); // Update Modbus state
}

void FAN_Controller::turnOff() {
    digitalWrite(fanPin, LOW);
    fanState = false;
    mb.setIsts(FAN_ON, false); // Update Modbus state
}

bool FAN_Controller::isOn() {
    return fanState;
}