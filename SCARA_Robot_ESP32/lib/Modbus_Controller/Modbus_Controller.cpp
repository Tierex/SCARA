#include "Modbus_Controller.h"
#include <ModbusSerial.h>       //https://github.com/epsilonrt/modbus-serial?tab=readme-ov-file
#include "MOD_VAR.h"


Modbus_Controller::Modbus_Controller(ModbusSerial* mb)
    : modbus(mb)
{}

void Modbus_Controller::Init_Registers() {
// --- Coils ---
    modbus->addCoil(MB_ENABLE, false);
    modbus->addCoil(MB_RESET_ERROR, false);

    modbus->addCoil(MB_GPIO0, false);
    modbus->addCoil(MB_GPIO1, false);
    modbus->addCoil(MB_GPIO2, false);
    modbus->addCoil(MB_GPIO3, false);
    modbus->addCoil(MB_GPIO4, false);
    modbus->addCoil(MB_GPIO5, false);
    modbus->addCoil(MB_GPIO6, false);
    modbus->addCoil(MB_GPIO7, false);
    modbus->addCoil(MB_GPIO8, false);
    modbus->addCoil(MB_GPIO9, false);

    modbus->addCoil(MB_RESET, false);

// --- Discrete Inputs ---
    modbus->addIsts(MB_POWER, false);
    modbus->addIsts(MB_ENABLE_STATE, false);
    modbus->addIsts(MB_REFERENCED, false);
    modbus->addIsts(MB_ERROR, false);

    modbus->addIsts(MB_POS_DONE, false);
    modbus->addIsts(MB_LIM_SW_A, false);
    modbus->addIsts(MB_LIM_SW_B, false);
    modbus->addIsts(MB_LIM_SW_C, false);
    modbus->addIsts(MB_LIM_SW_Z, false);

    modbus->addIsts(FAN_ON, false);

// --- Input Registers ---
    modbus->addIreg(MB_ERROR_CODE_1, 0);
    modbus->addIreg(MB_ERROR_CODE_2, 0);

    modbus->addIreg(MB_A_POS, 0);
    modbus->addIreg(MB_B_POS, 0);
    modbus->addIreg(MB_C_POS, 0);
    modbus->addIreg(MB_Z_POS, 0);
    modbus->addIreg(MB_M_A_POS, 0);
    modbus->addIreg(MB_M_B_POS, 0);
    modbus->addIreg(MB_M_C_POS, 0);
    modbus->addIreg(MB_M_Z_POS, 0);

    modbus->addIreg(MB_A_TEMP, 0);
    modbus->addIreg(MB_B_TEMP, 0);
    modbus->addIreg(MB_C_TEMP, 0);
    modbus->addIreg(MB_Z_TEMP, 0);

// --- Holding Registers ---
    modbus->addHreg(MB_STATE, 0);

    modbus->addHreg(MB_A_TARGET, 0);
    modbus->addHreg(MB_B_TARGET, 0);
    modbus->addHreg(MB_C_TARGET, 0);
    modbus->addHreg(MB_Z_TARGET, 0);
    modbus->addHreg(MB_A_VEl, 100);
    modbus->addHreg(MB_B_VEL, 100);
    modbus->addHreg(MB_C_VEL, 100);
    modbus->addHreg(MB_Z_VEL, 100);
    modbus->addHreg(MB_A_ACC, 500);
    modbus->addHreg(MB_B_ACC, 500);
    modbus->addHreg(MB_C_ACC, 500);
    modbus->addHreg(MB_Z_ACC, 500);
    modbus->addHreg(MB_A_JERK, 0);
    modbus->addHreg(MB_B_JERK, 0);
    modbus->addHreg(MB_C_JERK, 0);
    modbus->addHreg(MB_Z_JERK, 0);
}