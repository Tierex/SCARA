
#include <Arduino.h>
#include <ModbusSerial.h>


// Modbus Registers
const int RUNNING   = 0x1;    //(0001)
const int T_REACHED = 0x2;    //(0002)

const int ENABLE_M  = 0x3E8;  //(1000)
const int HOME      = 0x3E9;  //(1001)
const int RUN_INSTR = 0x3EA;  //(1002)

const int A_TARGET  = 0xFA0;  //(4000)
const int B_TARGET  = 0xFA1;  //(4001)
const int C_TARGET  = 0xFA2;  //(4002)
const int Z_TARGET  = 0xFA3;  //(4003)

const int A_SPEED   = 0xFA4;  //(4004)
const int B_SPEED   = 0xFA5;  //(4005)
const int C_SPEED   = 0xFA6;  //(4006)
const int Z_SPEED   = 0xFA7;  //(4007)

const int A_ACCEL   = 0xFA8;  //(4008)
const int B_ACCEL   = 0xFA9;  //(4009)
const int C_ACCEL   = 0xFAA;  //(4010)
const int Z_ACCEL   = 0xFAB;  //(4011)

const int A_POS     = 0xBB8;  //(3000)
const int B_POS     = 0xBB9;  //(3001)
const int C_POS     = 0xBBA;  //(3002)
const int Z_POS     = 0xBBB;  //(3003)




#define MySerial Serial // define serial port used, Serial most of the time, or Serial1, Serial2 ... if available
const unsigned long Baudrate = 9200;
const byte SlaveId = 24;

// ModbusSerial object
ModbusSerial mb (MySerial, SlaveId, -1);

void setup() {

  // MySerial.begin (Baudrate); // works on all boards but the configuration is 8N1 which is incompatible with the MODBUS standard
  // prefer the line below instead if possible
  MySerial.begin (Baudrate, MB_PARITY_EVEN);
  
  mb.config (Baudrate);
  mb.setAdditionalServerData ("SCARA"); // for Report Server ID function (0x11)

  // configure Registers
  mb.addIsts (RUNNING   , false);
  mb.addIsts (T_REACHED , false);
  
  mb.addCoil (ENABLE_M  , false);
  mb.addCoil (HOME      , false);
  mb.addCoil (RUN_INSTR , false);


  mb.addHreg (A_TARGET, 0);
  mb.addHreg (B_TARGET, 0);
  mb.addHreg (C_TARGET, 0);
  mb.addHreg (Z_TARGET, 40);

  mb.addHreg (A_SPEED , 10);
  mb.addHreg (B_SPEED , 10);
  mb.addHreg (C_SPEED , 10);
  mb.addHreg (Z_SPEED , 10);

  mb.addHreg (A_ACCEL , 5);
  mb.addHreg (B_ACCEL , 5);
  mb.addHreg (C_ACCEL , 5);
  mb.addHreg (Z_ACCEL , 5);
  
  mb.addIreg (A_POS   , 0);
  mb.addIreg (B_POS   , 0);
  mb.addIreg (C_POS   , 0);
  mb.addIreg (Z_POS   , 0);

  pinMode (13, OUTPUT);


}

void loop() {
  mb.task();
  

  digitalWrite (13, mb.Coil (ENABLE_M));


}
