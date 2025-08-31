#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

    #include <Arduino.h>
    #include <ModbusSerial.h>
    #include "MOD_VAR.h"

    class FAN_Controller {
        public:
            FAN_Controller(int pin);
            void turnOn();
            void turnOff();
            bool isOn();

        private:
            int  fanPin;
            bool fanState = false;
    };

#endif