//This file contains all the register adresses for the Modbus communication
//All registers are 16-bit unsigned integers
//All definitions start with MB_ to avoid conflicts with other libraries
//The addresses are split into different sections/functions;

//1xxxx Coils               --> PC Read/Write -- Offset = 00000
//2xxxx Discrete Inputs     --> PC Read only  -- Offset = 10000
//3xxxx Input Registers     --> PC Read only  -- Offset = 30000
//4xxxx Holding Registers   --> PC Read/Write -- Offset = 40000

//x0xxx Basic robot state 
//x1xxx Movement related
//x2xxx GPIO related
//x3xxx Reserved for future use
//x4xxx Uncommen functions
#pragma once
#include <ModbusSerial.h>


//Modbus Serial Object
    extern ModbusSerial mb; //Defined in main.cpp

//Coils 
    #define MB_ENABLE           00001 //Enable the robot // 1 = enabled, 0 = disabled
    #define MB_RESET_ERROR      00002 //Reset the error state of the robot //rising edge to reset error state
    
    #define MB_GPIO0            02001 //GPIO_0
    #define MB_GPIO1            02002 //GPIO_1
    #define MB_GPIO2            02003 //GPIO_2
    #define MB_GPIO3            02004 //GPIO_3
    #define MB_GPIO4            02005 //GPIO_4
    #define MB_GPIO5            02006 //GPIO_5
    #define MB_GPIO6            02007 //GPIO_6
    #define MB_GPIO7             2008 //GPIO_7
    #define MB_GPIO8             2009 //GPIO_8
    #define MB_GPIO9            02010 //GPIO_9

    #define MB_RESET            04001 //Reset the robot control system only used for debugging purposes


//Discrete Inputs
    #define MB_POWER            10001  //Power state of the robot 1 = powered, 0 = not powered
    #define MB_ENABLE_STATE     10002  //Enable state of the robot 1 = enabled, 0 = disabled
    #define MB_REFERENCED       10003  //Referenced state of the robot 1 = referenced, 0 = not referenced
    #define MB_ERROR            10004  //Error state of the robot 1 = error, 0 = no error

    #define MB_POS_DONE         11001  //The robot TCP is on target 1 = on target, 0 = not on target
    #define MB_LIM_SW_A         11002  //Limit switch A state 1 = triggered, 0 = not triggered
    #define MB_LIM_SW_B         11003  //Limit switch B state 1 = triggered, 0 = not triggered
    #define MB_LIM_SW_C         11004  //Limit switch C state 1 = triggered, 0 = not triggered
    #define MB_LIM_SW_Z         11005  //Limit switch Z state

    #define FAN_ON              12005  //Fan state 1 = on, 0 = off

//Input Registers
    #define MB_ERROR_CODE_1     30001 //Error code of the robot
    #define MB_ERROR_CODE_2     30002 //Error code of the robot

    #define MB_A_POS            31001 // Current position of the A axis
    #define MB_B_POS            31002 // Current position of the B axis
    #define MB_C_POS            31003 // Current position of the C axis
    #define MB_Z_POS            31004 // Current position of the Z axis 
    #define MB_M_A_POS          31005 // Motor position of the A axis
    #define MB_M_B_POS          31006 // Motor position of the B axis
    #define MB_M_C_POS          31007 // Motor position of the C axis
    #define MB_M_Z_POS          31008 // Motor position of the Z axis

    #define MB_A_TEMP           32001 // Temperature of the A axis
    #define MB_B_TEMP           32002 // Temperature of the B axis
    #define MB_C_TEMP           32003 // Temperature of the C axis
    #define MB_Z_TEMP           32004 // Temperature of the Z axis



//Holding Registers
    #define  MB_STATE           40001 // Current state of the robot

    #define  MB_A_TARGET        41001 // Target position of the A axis in degrees *10
    #define  MB_B_TARGET        41002 // Target position of the B axis in degrees *10
    #define  MB_C_TARGET        41003 // Target position of the C axis in degrees *10
    #define  MB_Z_TARGET        41004 // Target position of the Z axis in mm *10
    #define  MB_A_VEl           41005 // Velocity of the A axis in degrees/s *10
    #define  MB_B_VEL           41006 // Velocity of the B axis in degrees/s *10
    #define  MB_C_VEL           41007 // Velocity of the C axis in degrees/s *10
    #define  MB_Z_VEL           41008 // Velocity of the Z axis in mm/s *10
    #define  MB_A_ACC           41009 // Acceleration of the A axis in degrees/s^2 *10
    #define  MB_B_ACC           41010 // Acceleration of the B axis in degrees/s^2 *10
    #define  MB_C_ACC           41011 // Acceleration of the C axis in degrees/s^2 *10
    #define  MB_Z_ACC           41012 // Acceleration of the Z axis in mm/s^2 *10
    #define  MB_A_DEC           41013 // Deacceleration of the A axis in degrees/s^2 *10
    #define  MB_B_DEC           41014 // Deacceleration of the B axis in degrees/s^2 *10
    #define  MB_C_DEC           41015 // Deacceleration of the C axis in degrees/s^2 *10
    #define  MB_Z_DEC           41016 // Deacceleration of the Z axis in mm/s^2 *10