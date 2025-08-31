//Robot States Definitions
// Following the same defition as used by Codesys Softmotion https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_robotics_state_machine.html
// All definitions start with ST_ to avoid conflicts with other libraries


#define ST_ERROR_STOP  0  //Error state    - no motion allowed
#define ST_DISABLED    1  //Disabled state - motors are off, but system is powered
#define ST_STANDBY     2  //standby state  - motor are powered, but not moving
#define ST_STOPPING    3  //Stopping state - in the process of stopping motion  
#define ST_MOVING      4  //Motion state   - actively moving
#define ST_HOMING      5  //Homing state   - Peforming the homing procedure


    
