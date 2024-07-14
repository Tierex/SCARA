from pymodbus.client import ModbusSerialClient
import time
import minimalmodbus


    

if __name__ == "__main__":

    robot = minimalmodbus.Instrument('COM4', 24)  # port name, slave address (in decimal)

    robot.serial.baudrate = 9200  
    

    # try:
    #     print(robot.read_register(4001))
    # except IOError:
    #     print("Failed to read from instrument")

    while True:
            robot.write_bit(1000,True)
            robot.write_bit(1000,False)
        #     print(robot.read_register(3000,functioncode=4))







    # client = ModbusSerialClient("COM4",baudrate=38400)

    # client.connect()


    # print(client.read_input_registers(1000, 1, 24))
    


    # # client.write_registers(1, [10]*8, unit=0x1)
    
    # # client.read_coils()
        

    # client.close()