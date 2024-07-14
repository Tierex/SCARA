import minimalmodbus
import time
import numpy as np
import math



class SCARA:
    """
	Python interface for SCARA robot through modbus.
    Modbus interface: https://minimalmodbus.readthedocs.io/en/stable/
	"""
    """ REGISTERS:
    RUNNING   = 0x1;    //(0001)
    T_REACHED = 0x2;    //(0002)

    ENABLE_M  = 0x3E8;  //(1000)
    HOME      = 0x3E9;  //(1001)
    AUTO_RUN  = 0x3EA;  //(1002)
    RUN_INSTR = 0x3EB;  //(1003)

    A_TARGET  = 0xFA0;  //(4000)
    B_TARGET  = 0xFA1;  //(4001)
    C_TARGET  = 0xFA2;  //(4002)
    Z_TARGET  = 0xFA3;  //(4003)

    A_SPEED   = 0xFA4;  //(4004)
    B_SPEED   = 0xFA5;  //(4005)
    C_SPEED   = 0xFA6;  //(4006)
    Z_SPEED   = 0xFA7;  //(4007)

    A_ACCEL   = 0xFA8;  //(4008)
    B_ACCEL   = 0xFA9;  //(4009)
    C_ACCEL   = 0xFAA;  //(4010)
    Z_ACCEL   = 0xFAB;  //(4011)

    A_POS     = 0xBB8;  //(3000)
    B_POS     = 0xBB9;  //(3001)
    C_POS     = 0xBBA;  //(3002)
    Z_POS     = 0xBBB;  //(3003)
    """

#Setup functions
    def __init__(self, port, address = None , mode = 0):

        self.port    = port
        self.address = 24 if address is None else address
        self._connect()
        self.mode  = mode
        self.homed = False

    #Inner functions
    def _connect(self):
        """Connect to robot"""
        print(f"\nConnecting to SCARA robot:")
        print(f"Port        |  {self.port}")
        print(f"Address     |  {self.address}")

        #setup connection parameters
        self.connection = minimalmodbus.Instrument(self.port, self.address)  # port name, slave address (in decimal)
        self.connection.serial.baudrate = 19200
        self.connection.serial.parity = "E" #even parity bit
        self.connection.serial.stopbits = 1 #1 stopbit


        #Try to connect to robot
        for attempt in range(5):
            print(f"Attempt {attempt+1}   |",end="")
            try:
                self.connection.read_bit(1000,1)
                print(f"  Succes")
                break  # Exit the loop if successful
            except:
                print(f"  Failed")
                time.sleep(1)

                if attempt == 4:
                    print(f"All connection attempts failed - Exiting \n")
                    exit(1)  # Exit the loop after max attempts
                    
        print(f"Succesfully connected to SCARA robot \n\n")
   
    def _read(self,register,nmb_of_registers):
        try:
            data = self.connection.read_registers(register,nmb_of_registers)
        except IOError as error:
            print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return None
        return data 
        

#mode functions
    def enable_motor(self, bool: bool) -> bool:
        """Enables power the robot motors"""
        try:
            self.connection.write_bit(1000,bool)
            return 1
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0
    
    def set_mode (self, mode: int) -> bool:
        """ 0 = default non-mode
            1 = Homing
            2 = Controlled-mode
            3 = Auto-run
            4 = CFP
        """

        try:
            self.connection.write_register(4000, mode)
            self.mode = mode
            return 1
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0



#status functions
    def motor_enabled(self) -> bool:
        """Check if robot motors are powered"""
        try:
            status = bool(self.connection.read_bit(1000,1))
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
            status = False
        return status

    def is_running(self) -> bool:
        """Check if robot is running"""
        try:
            status = bool(self.connection.read_bit(1, 2))
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
            status = False
        return status
    
    def target_reached(self) -> bool:
        """Check if target is reached"""
        try:
            status = bool(self.connection.read_bit(2 ,2))
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
            status = False
        return status
    
    def homed(self) -> bool:
        """Check if robot is homed """
        try:
            status = bool(self.connection.read_bit(4 ,2))
            self.homed = status
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
        return status


    def print_status (self) -> None:
        """Print robot status in terminal"""
        self.read_motor_data()


        print(F"SCARA robot:")
        print(F"_____________________________________________________")
        print(F"port            | {self.port}")
        print(F"address         | {self.address}")
        print(F"baudrate        | {self.connection.serial.baudrate}")
        print(F"_____________________________________________________")
        print(F"Motor ON        | {None}")
        print(F"Running         | {self.is_running()}")
        print(F"On target       | {self.target_reached()}")

        print(F"Homing          | {self.home()}")

        print(F"_____________________________________________________")
        print(F"Target          | {self.targets}")
        print(F"speeds          | {self.speeds}")
        print(F"acceleration    | {self.accelerations}")
        print(F"_____________________________________________________")
        print(F"Position:       | {self.positions}")

    
#read variables functions
    def read_targets(self) -> list[int]:
        """Return current robot targets"""
        targets = self._read(4100,4)

        try:
            if self.targets != targets:
                raise(f"Robot and computer targets do not match")
        except:
            self.targets = targets
        return self.targets
            
    def read_speeds(self) -> list[int]:
        """Return current motor speeds"""
        speeds = self._read(4104,4)
        try:
            if self.speeds != speeds:
                raise(f"Robot and computer targets do not match")
        except:
            self.speeds = speeds
        return self.speeds
    
    def read_accelerations(self)-> list[int]:
        """Return current motor accelerations"""
        accelerations = self._read(4108,4)
        try:
            if self.accelerations != accelerations:
                raise(f"Robot and computer targets do not match")
        except:
            self.accelerations = accelerations
        return self.accelerations

    def read_positions(self)-> list[int]:
        """Return current robot position"""
        try:
            self.positions = self.connection.read_registers(3000,4,4)     
        except IOError as error:
            print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return
        return self.positions
    
    def read_motor_positions(self)-> list[int]:
        """Return current robot position"""
        try:
            self.motor_positions = self.connection.read_registers(3004,4,4)     
        except IOError as error:
            print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return
        return self.motor_positions
    


#Set variables functions
    def set_targets(self,targets: list[float]) -> bool:
        """Set robot target position
        input: List .1 precision 
        """
        if len(targets) > 4:
            raise TypeError("input must be a list of [Target A, Target B, Target C, Target Z]")
        
        
        try:
            self.connection.write_registers(4100, np.uint16(np.multiply(targets,10)).tolist())
            self.targets = targets
            return 1
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0

    def set_speeds(self,speeds: list[float]) -> bool:
        """Set motor speeds"""
        if len(speeds) > 4 :
            raise TypeError("input must be a list of [Speed A, Speed B, Speed C, Speed Z]")
        
        try:
            self.connection.write_registers(4104, np.multiply(speeds,10).tolist())
            self.speeds = speeds
            return 1
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0

    def set_accelerations(self,accelerations: list[float]) -> bool:
        """Set motor acceleration"""
        if len(accelerations) > 4:
            raise TypeError("input must be a list of [Accel A, Accel B, Accel C, Accel Z]")
        
        try:
            self.connection.write_registers(4108, np.multiply(accelerations,10).tolist())
            self.accelerations = accelerations
            return 1
        
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0


#TCP calculations
    #[X,Y,Z,RZ]
def _scara_inverse_kinematics(x, y, z, rz):
    l1 = 180
    l2 = 160
    theta1_max = 95
    theta2_max = 122

    # Calculate theta2
    cos_theta2 = (y**2 + x**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Ensure the value is within the domain of arccos
    if cos_theta2 < -1 or cos_theta2 > 1:
        return []  # No valid configurations
    
    theta2 = np.arccos(cos_theta2)
    
    # Calculate theta1
    term1 = np.arctan2(x, y)
    term2 = np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    theta1 = term1 - term2
    
    # Another possible solution for theta1
    theta1_alt = term1 + term2
    
    # Calculate the other possible solution for theta2
    theta2_alt = -theta2
    
    # Store all possible configurations
    configurations = np.asarray([[theta1,theta2,z,rz],[theta1_alt,theta2_alt,z,rz]])
    configurations[:,0:2] = np.rad2deg(configurations[:,0:2])

    # Filter configurations based on the angle limits
    valid_configurations = [config for config in configurations if np.abs(config[0]) <= theta1_max and np.abs(config[1]) <= theta2_max]
    
    return valid_configurations
    


if __name__ == "__main__":



    robot = SCARA("COM3")
    robot.enable_motor(True)

    speed = 100
    accel = 1000

    while True:
        robot.set_speeds([100,100,100,speed])
        robot.set_accelerations([100,100,100,accel])
        
        print(f"speed {speed}")
        print(f"accel {accel}")
        input("presse enter to test...")

        robot.set_targets([50,50,0,-50])
        while robot.target_reached() == False:
            time.sleep(0.1)
            print("Running")
        print("done, moving to next target")
        robot.set_targets([0,0,0,0])


    print(robot.read_targets()) 