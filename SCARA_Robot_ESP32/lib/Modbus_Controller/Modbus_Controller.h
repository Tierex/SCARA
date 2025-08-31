#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

    #include <Arduino.h>
    #include <ModbusSerial.h>
    #include "MOD_VAR.h"


    class Modbus_Controller {
        public:
            Modbus_Controller(ModbusSerial* mb); // Pass pointer
            void Init_Registers();
            // Add other methods as needed

        private:
            ModbusSerial* modbus; // Pointer to shared ModbusSerial
            // Add other private members as needed
    };

#endif