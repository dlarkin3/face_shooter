#!/usr/bin/env python
from __future__ import print_function

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


# Control table address
ADDR_CW_ANGLE_LIMIT         = 6
ADDR_CCW_ANGLE_LIMIT        = 8
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_LED_ONOFF           = 25               # http://emanual.robotis.com/docs/en/dxl/mx/mx-28/#control-table-of-ram-area
ADDR_MX_D_GAIN              = 26
ADDR_MX_I_GAIN              = 27
ADDR_MX_P_GAIN              = 28
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_MOVING_SPEED        = 32
ADDR_MX_TORQUE_LIMIT        = 34
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_SPEED       = 38
ADDR_MX_PRESENT_LOAD        = 40
ADDR_MX_PRESENT_VOLTAGE     = 42
ADDR_MX_PRESENT_TEMPERATURE = 43
ADDR_MX_REGISTERED          = 44
ADDR_MX_MOVING              = 46
ADDR_MX_LOCK                = 47
ADDR_MX_PUNCH               = 48
ADDR_MX_REALTIME_TICK       = 50
ADDR_MX_GOAL_ACCELERATION   = 73

PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

STOPPED = 0
MOVING_CW = 1
MOVING_CCW = 2

class dynamixel_mx28:

    def __init__(self, dxl_id=3, baud=1000000, device='/dev/dynamixel'):
        self.dxl_id = dxl_id
        self.baud = baud
        self.device = device
        self.left_limit=0
        self.right_limit=0
        self.brake=0 #off
        self.moving_state=STOPPED
                
        # Initialize PortHandler instance
        # Get methods and members of PortHandlerLinux
        self.portHandler = PortHandler(self.device)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol1PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)  
        
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baud):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()   
            
    def __del__(self):
        self.portHandler.closePort()    
        
    def set_right_limit(self,limit):
        if(limit <0 or limit>4095):
            self.right_limit=0
            print("Illegal limit. Must between 0 and 4096.")            
        self.right_limit=limit

    def set_left_limit(self,limit):
        if(limit <0 or limit>4095):
            self.right_limit=0
            print("Illegal limit. Must between 0 and 4096.")      
        self.left_limit=limit        
        
            
    def set_cw_limit(self,limit=0,dev_id=None):
        if(limit <0 or limit>4095):
            self.right_limit=0
            print("Illegal limit. Must between 0 and 4096.")      
        self.left_limit=limit        
        if dev_id is None:
            dev_id = self.dxl_id    
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dev_id, ADDR_CW_ANGLE_LIMIT, limit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel CW limit set to %d" % limit)
            
    def set_ccw_limit(self,limit=0,dev_id=None):
        if(limit <0 or limit>4095):
            self.right_limit=0
            print("Illegal limit. Must between 0 and 4096.")      
        self.left_limit=limit        
        if dev_id is None:
            dev_id = self.dxl_id
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dev_id, ADDR_CCW_ANGLE_LIMIT, limit)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel CCW limit set to %d" % limit)

    def set_wheel_mode(self,dev_id=None):
        if dev_id is None:
            dev_id = self.dxl_id
        self.set_cw_limit(0,dev_id)
        self.set_ccw_limit(0,dev_id)
        
    def get_present_position(self,dev_id=None):
        if dev_id is None:
            dev_id = self.dxl_id
        dxl_position = 99999 # error code
        dxl_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dev_id, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))  
        return dxl_position # returns 99999 if not successful else returns valid position        
        
    def get_present_speed(self,dev_id=None):
        # Speed: 0 - 1023 = CCW, 1024 - 2047 = CW
        if dev_id is None:
            dev_id = self.dxl_id
        dxl_speed = 99999 # error code
        dxl_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dev_id, ADDR_MX_PRESENT_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error)) 
            
        if (0 > dxl_speed > 1023):
            self.moving_state = MOVING_CCW
        elif (1024 < dxl_speed < 2048):
            self.moving_state = MOVING_CW
        else:
            self.moving_state = STOPPED   
              
        return dxl_speed # returns 99999 if not successful else returns valid position
        
    def set_moving_speed(self,set_speed,dev_id=None):
        if dev_id is None:
            dev_id = self.dxl_id
        
        dxl_position=self.get_present_position()     
        dxl_speed=self.get_present_speed()        
        
        if (self.left_limit + self.right_limit != 0): # if limits are set then enforce them  
            if (dxl_position < self.right_limit and set_speed > 1024): 
                    set_speed = 1024
                    #print("1024ing === ", end='')
            elif (dxl_position > self.left_limit and set_speed < 1024):
                    #print("zeroing === ", end='')
                    set_speed = 0   
                
        if (self.brake):
            #print("==========BRAKE ON==========", end='')
            if (dxl_speed >  1024):
                set_speed = 1024
            else:
                set_speed = 0     
                             
        #print("Speed: %d,%d Position: %d, movestate: %d" % (set_speed,dxl_speed,dxl_position,self.moving_state), end=' ' )    
        
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dev_id, ADDR_MX_MOVING_SPEED, set_speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))                                

    def toggle_brake(self,dev_id=None):
        if dev_id is None:
            dev_id = self.dxl_id
        self.brake = not self.brake
        if(self.brake):
            print("BRAKE ENABLED FOR ID: %d" % dev_id)
        else:
            print("BRAKE DISABLED FOR ID: %d" % dev_id)   
            
    def set_torque(self,dev_id=None):
        if dev_id is None:
            dev_id = self.dxl_id            
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dev_id, ADDR_MX_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))          
        
if __name__ == "__main__":
    dyna1 = dynamixel_mx28(dxl_id=3)
    dyna2 = dynamixel_mx28(dxl_id=4)   
    dyna1.set_wheel_mode()
    dyna2.set_wheel_mode()
    dyna1.set_torque(3)
    dyna1.set_torque(4)
    print("Pos1: %d, Pose2: %d" % (dyna1.get_present_position(), dyna2.get_present_position()))   

