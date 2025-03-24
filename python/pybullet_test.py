import pybullet as p
import time
import pybullet_data
import os
import numpy as np
import zmq
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time

class LeapNode:
    def __init__(self):
        ####Some parameters
        # I recommend you keep the current limit from 350 for the lite, and 550 for the full hand
        # Increase KP if the hand is too weak, decrease if it's jittery.
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        # self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(12))

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0

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

class LeapPybulletIK():
    def __init__(self):
        # super().__init__('leap_pyb_ik')  
        # start pybullet
        #clid = p.connect(p.SHARED_MEMORY)
        #clid = p.connect(p.DIRECT)
        physicsClient = p.connect(p.GUI)
        self.glove_to_leap_mapping_scale = 1.6
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src = os.path.join(path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
        self.LeapId = p.loadURDF(
            path_src,
            [-0.05, -0.03, -0.125],
            p.getQuaternionFromEuler([0, 1.57, 1.57]),
            useFixedBase = True
        )
        self.numJoints = p.getNumJoints(self.LeapId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)

        # for i in range (10000):
        #     p.stepSimulation()
        #     time.sleep(1./240.)
        # p.disconnect()
        

        self.create_target_vis()

    def create_target_vis(self):
            # load balls
            small_ball_radius = 0.01
            small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
            ball_radius = 0.01
            ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
            baseMass = 0.001
            basePosition = [0.25, 0.25, 0]
            
            self.ballMbt = []
            for i in range(0,4):
                self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
                no_collision_group = 0
                no_collision_mask = 0
                p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
            p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
            p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
            p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
            p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])
    
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[1], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[3], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[5], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[7], current_orientation)

    def get_glove_data(self):
        #gets the data converts it and then computes IK and visualizes
        hand_pos = []  
        # for i in range(0,10):
        #     hand_pos.append([poses[i].position.x * self.glove_to_leap_mapping_scale * 1.15, poses[i].position.y * self.glove_to_leap_mapping_scale, -poses[i].position.z * self.glove_to_leap_mapping_scale])
        # hand_pos[2][0] = hand_pos[2][0] - 0.02  this isn't great because they won't oppose properly
        # hand_pos[3][0] = hand_pos[3][0] - 0.02    
        # hand_pos[6][0] = hand_pos[6][0] + 0.02
        # hand_pos[7][0] = hand_pos[7][0] + 0.02
        #hand_pos[2][1] = hand_pos[2][1] + 0.002
        hand_pos[4][1] = hand_pos[4][1] + 0.002
        hand_pos[6][1] = hand_pos[6][1] + 0.002
        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)



def main(**kwargs):
    # leap_hand = LeapNode()
    
    left_glove_sn = "558097A3"
    right_glove_sn = "E13E29F2"

    context = zmq.Context()
    #Socket to talk to Manus SDK
    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000")
    leappybulletik = LeapPybulletIK()
    glove_to_leap_mapping_scale = 1.6

    while True:
        message = socket.recv()
        #receive the message from the socket
        message = message.decode('utf-8')

        # right[0] is gloveID, the nodes use for IK are (3, 4, 8, 9, 13, 14, 18, 19, 23, 24)
        # right[1:31] are the x,y,z values for the above nodes; 10 nodes * 3 axes = 30
        right = message.split(",")   
        right = list(map(float,right[1:31]))
        right = [round(n, 2) for n in right]

        hand_pos = []  
        for i in range(0,10):
            hand_pos.append([right[i * 3] * glove_to_leap_mapping_scale * 1.15, right[i * 3 + 1]* glove_to_leap_mapping_scale, -i * right[i * 3 + 2] * glove_to_leap_mapping_scale])
        # hand_pos[2][0] = hand_pos[2][0] - 0.02  this isn't great because they won't oppose properly
        # hand_pos[3][0] = hand_pos[3][0] - 0.02    
        # hand_pos[6][0] = hand_pos[6][0] + 0.02
        # hand_pos[7][0] = hand_pos[7][0] + 0.02
        #hand_pos[2][1] = hand_pos[2][1] + 0.002
        hand_pos[4][1] = hand_pos[4][1] + 0.002
        hand_pos[6][1] = hand_pos[6][1] + 0.002
        # compute_IK(hand_pos)

        leappybulletik.update_target_vis(hand_pos)
        print(hand_pos)
        time.sleep(1)

if __name__ == "__main__":
    main()