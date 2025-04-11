import numpy as np
import zmq

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Used the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more.

#LEAP hand conventions:
#180 is flat out home pose for the index, middle, ring, finger MCPs. (In radian)
#Applying a positive angle closes the joints more and more to curl closed.
#The MCP is centered at 180 and can move positive or negative to that.

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side(Splay), MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

"""
########################################################
class LeapNode:
    def __init__(self):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(12))

        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0

        # Motor IDs, can be changed in Dynamixel Wizard
        # Index  -> 0: MCP side, 1: MCP flex, 2: Pip flex, 3: Dip flex
        # Middle -> 4: MCP side, 5: MCP flex, 6: Pip flex, 7: Dip flex
        # THumb  -> 8: MCP side, 9: MCP flex, 10: Pip flex, 11: Dip flex
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11]

        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM3', 4000000)
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
    leap_hand = LeapNode()

    context = zmq.Context()
    #Socket to talk to Manus SDK

    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000")

    while True:
        message = socket.recv()
        #receive the message from the socket

        message = message.decode('utf-8')
        print("Received reply %s" % (message))
        right = message.split(",")   
        if len(right) == 12:
            rightData = list(map(float,right[0:12]))
            # Current joint angle order: thumb, index, middle


        # Adjustments for index and middle joints
        # +180 because zero position (open pose) for LEAP is 180 degrees (MANUS origin is 0 degrees)
        for n in range(4,12):
            if (n == 4):
                # Correcting polarity and amplifying splay
                rightData[n] = (rightData[n] + 4) * -1.7 + 180
            elif (n == 8):
                # Correcting polarity and amplifying splay
                rightData[n] = (rightData[n] + 9) * -1.5 + 180
            elif (n == 5 or n == 9):
                # Increase MCP flexion
                rightData[n] = rightData[n] + 10 + 180
            else:
                rightData[n] = rightData[n] + 180
        

        # Adjusting thumb joints and reordering joint angles in rightData
        rightData = rightData[4:12] + [-20 + 2.3*rightData[0] + 180] + [55 - 1.7*rightData[1]+180] + [3.5*rightData[2]+180] + [rightData[3]+180]
        # Current joint angle order: index, middle, thumb
        
        # Convert to radian and set LEAP position
        leap_hand.set_leap(np.deg2rad(rightData))
        print(rightData)
        time.sleep(.03)


if __name__ == "__main__":
    main()
