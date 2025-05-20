import numpy as np
import zmq

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Used the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.

#LEAP hand controlconventions:
#180 is flat out (In radian)
#Applying a positive angle closes the joints more and more to curl closed.
#The MCP is centered at 180 and can move positive or negative to that.

#The joint numbering goes from Thumb (0-3), Index(4-7), Middle(8-11) and from MCP flex, MCP side (splay), PIP, DIP for each finger.
#For instance, the MCP side of Index is ID 5, the MCP flex of middle is 8

"""
########################################################
class LeapNode:
    def __init__(self, com, motors):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 650
        self.kI = 0
        self.kD = 650
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(len(motors)))
        self.com = com

        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = motors

        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, com, 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility joint angles.  It adds 180 to make the fully open position at 0 instead of 180
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility for policies, it assumes the ranges are [-1,1] and then convert to leap hand ranges.
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position of the robot
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    #These combined commands are faster FYI and return a list of data
    def pos_vel(self):
        return self.dxl_client.read_pos_vel()
    #These combined commands are faster FYI and return a list of data
    def pos_vel_eff_srv(self):
        return self.dxl_client.read_pos_vel_cur()
    
#init the node
def main(**kwargs):

    # Motor IDs, can be changed in Dynamixel Wizard
    # Thumb  -> 0: MCP flex, 1: MCP side, 2: PIP flex, 3: DIP flex
    # Index -> 4: MCP flex, 5: MCP side, 6: PIP flex, 7: DIP flex
    # Middle  -> 8: MCP flex, 9: MCP side, 10: PIP flex, 11: DIP flex
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]
    leap_hand_left = LeapNode("COM5", motor_ids_left)
    leap_hand_right = LeapNode("COM6", motor_ids_right)
    time.sleep(1)
    for n in range(300):
        newAng = 3.141 + n*0.01

        # Starts at an open pose then slowly clenches, splay is maintained
        leap_hand_left.set_leap([newAng,newAng,newAng,newAng,newAng,3.141,newAng,newAng,newAng,3.141,newAng,newAng]) 
        leap_hand_right.set_leap([newAng,newAng,newAng,newAng,newAng,3.141,newAng,newAng,newAng,3.141,newAng,newAng]) 
        time.sleep(.1)


if __name__ == "__main__":
    main()
