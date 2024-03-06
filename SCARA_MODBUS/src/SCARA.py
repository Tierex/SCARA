import minimalmodbus
import time


class SCARA:
    """
	Python interface for SCARA robot through modbus.
    Modbus interface: https://minimalmodbus.readthedocs.io/en/stable/
	"""
    """ REGISTERS:
    RUN       = 0x1;    //(0001)
    ENABLE    = 0x3E8;  //(1000)
    HOME      = 0x3E9;  //(1001)

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
    def __init__(self, port, address = None , autosend = False):



        self.port    = port
        self.address = 24 if address is None else address
        self.connect()
        self.autosend = autosend

    def connect(self):
        print(f"Connecting to SCARA robot: {self.port} - address: {self.address}")
        self.connection = minimalmodbus.Instrument(self.port, self.address)  # port name, slave address (in decimal)
        self.connection.serial.baudrate = 115200
        print(f"succesfully connected to SCARA")
   

    def run_command(self):
        try:
            self.connection.write_bit(1001,True)
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return 0
        else: return 1

    def home (self, blocking = False):
        try:
            self.connection.write_bit(1002,True)
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")
            return
        
        if blocking == True:
            done = False
            while done == False:
                try:
                    done = self.connection.read_bit(1002)
                    time.sleep(0.2)
                except IOError as error:
                    print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return
        
    def print_status (self):
        self.read_targets
        self.read_speeds
        self.read_accelerations
        self.read_positions


        print(F"SCARA robot:")
        print(F"_____________________________________________________")
        print(F"port            | {self.port}")
        print(F"address         | {self.address}")
        print(F"baudrate        | {self.connection.serial.baudrate}")
        print(F"_____________________________________________________")
        print(F"Motor ON        | {None}")
        print(F"Running         | {self.is_running}")
        print(F"On target       | {self.target_reached}")

        print(F"Homing          | {self.home}")

        print(F"_____________________________________________________")
        print(F"Target          | {self.targets}")
        print(F"speeds          | {self.speeds}")
        print(F"acceleration    | {self.accelerations}")
        print(F"_____________________________________________________")
        print(F"Position:       | {self.positions}")


    def is_running(self):
        try:
            status = bool(self.connection.read_bit(1))
            time.sleep(0.2)
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
            status = False
        return status
    
    def target_reached(self):
        try:
            status = bool(self.connection.read_bit(2))
            time.sleep(0.2)
        except IOError as error:
            print (f"Failed to get status from robot: {type(error).__name__} - {error}")
            status = False
        return status
        
    
#read variables functions
    def read_targets(self):
        targets = self._read(4000,4)

        try:
            if self.targets != targets:
                raise(f"Robot and computer targets do not match")
        except:
            self.targets = targets
        return self.targets
            
    def read_speeds(self):
        speeds = self._read(4004,4)
        try:
            if self.speeds != speeds:
                raise(f"Robot and computer targets do not match")
        except:
            self.speeds = speeds
        return self.speeds
    
    def read_accelerations(self):
        accelerations = self._read(4008,4)
        try:
            if self.accelerations != accelerations:
                raise(f"Robot and computer targets do not match")
        except:
            self.accelerations = accelerations
        return self.accelerations

    def read_positions(self):
        try:
            self.positions = self.connection.read_registers(3000,4,4)    
        except IOError as error:
            print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return
        return self.positions
    

#Set variables functions
    def set_targets(self,targets):
        if not isinstance(targets, list) or len(targets) > 4:
            raise TypeError("input must be a list of [Target A, Target B, Target C, Target Z]")
        
        try:
            self.connection.write_registers(4000,targets)
            self.targets = targets
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")

    def set_speeds(self,speeds):
        if not isinstance(speeds, list) or len(speeds) > 4 :
            raise TypeError("input must be a list of [speed A, speed B, speed C, speed Z]")
        
        try:
            self.connection.write_registers(4004, speeds)
            self.speeds = speeds
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")

    def set_accelerations(self,accelerations):
        if not isinstance(accelerations, list) or len(accelerations) > 4:
            raise TypeError("input must be a list of [Accel A, Accel B, Accel C, Accel Z]")
        
        try:
            self.connection.write_registers(4008, accelerations)
            self.accelerations = accelerations
        except IOError as error:
            print (f"Failed to write data to robot: {type(error).__name__} - {error}")

#Inner functions
    def _read(self,register,nmb_of_registers):
        try:
            data = self.connection.read_registers(register,nmb_of_registers)
        except IOError as error:
            print (f"Failed to read data from robot: {type(error).__name__} - {error}")
            return None
        return data 
        
    


if __name__ == "__main__":
    robot = SCARA("COM4")
    robot.set_targets([10,30,30,40])
    robot.print_status()
    robot.read_targets()
    print(robot.connection.read_register(3000,0,4))

    robot.connection.write_registers(4000,[20,20,20,20,30,30,30,30,40,40,40,40])
    print(robot.connection.read_register(3000,0,4))
    print(robot.connection._latest_roundtrip_time)
    print('done')           