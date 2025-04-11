import pybullet as p
import time
import pybullet_data
import os
import numpy as np
import zmq
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
from scipy.spatial.transform import Rotation as R


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
        self.glove_to_leap_mapping_scale = 1
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src = os.path.join(path_src, "for_urdf_export/for_urdf_export.urdf")
        # here endEffector refers to the target joints(not link)
        # by right its [3,4,8,9,13,14]
        self.leapEndEffectorIndex = [3, 4, 8, 9,13,14]
        self.LeapId = p.loadURDF(
            path_src,
            # [-0.05, -0.03, -0.125],
            [0.015,0.1,-0.008],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
            globalScaling = 0.55
            
        )
        self.numJoints = p.getNumJoints(self.LeapId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)

        # self.create_initial_vis()
        self.create_target_vis()

    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.003
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]

        self.ballMbt = []
        for i in range(0,6):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[0, 0, 1, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[1, 0, 1, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 0, 1, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[5], -1, rgbaColor=[1, 0, 1, 1])
    
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[0], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[1], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[2], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[3], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[4])
        p.resetBasePositionAndOrientation(self.ballMbt[4], hand_pos[4], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[5])
        p.resetBasePositionAndOrientation(self.ballMbt[5], hand_pos[5], current_orientation)

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
        # self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)

    def compute_IK(self, hand_pos):
        p.stepSimulation()     

        rightHandThumb_middle_pos = hand_pos[0]
        rightHandThumb_pos = hand_pos[1]

        rightHandIndex_middle_pos = hand_pos[2]
        rightHandIndex_pos = hand_pos[3]
        
        rightHandMiddle_middle_pos = hand_pos[4]
        rightHandMiddle_pos = hand_pos[5]
        
        

        leapEndEffectorPos = [

            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandThumb_middle_pos,
            rightHandThumb_pos,
        ]

        # returns resultant joint angles
        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        # print(len(jointPoses))
        
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,))
        # print(combined_jointPoses[0:5])

        # update the hand joints in simulator
        for i in range(15):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        # real_robot_hand_q = np.array([float(0.0) for _ in range(16)])
        # #real_left_robot_hand_q = np.array([0.0 for _ in range(16)])

        # real_robot_hand_q[0:4] = jointPoses[0:4]
        # real_robot_hand_q[4:8] = jointPoses[4:8]
        # real_robot_hand_q[8:12] = jointPoses[8:12]
        # real_robot_hand_q[12:16] = jointPoses[12:16]
        # real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        # real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        # real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        # stater = JointState()
        # stater.position = [float(i) for i in real_robot_hand_q]
        # self.pub_hand.publish(stater)



def main(**kwargs):
    # leap_hand = LeapNode()

    context = zmq.Context()
    #Socket to talk to Manus SDK
    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000")

    leappybulletik = LeapPybulletIK()

    # iniPos = []
    targetPosList = []
    transformPosList = []

    # linkStates = p.getLinkStates(leappybulletik.LeapId, leappybulletik.leapEndEffectorIndex)

    for n in range(10000):        
        targetPosList.clear()
        transformPosList.clear()
        
        # for i in range (4):
        #     newPos.append((iniPos[i][0], iniPos[i][1] - 0.001 * n, iniPos[i][2] - 0.002 * n))
        # print(newPos)
        

        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")   

        # gloveID = manusData[0]
        axisRot = list(map(float,manusData[1:5]))
        targetCoord = list(map(float,manusData[5:]))
        

        targetPosList.append(targetCoord[0:3]) #thumb_distal
        targetPosList.append(targetCoord[3:6]) #thumb_tip
        targetPosList.append(targetCoord[6:9]) #index_distal
        targetPosList.append(targetCoord[9:12]) #index_tip
        targetPosList.append(targetCoord[12:15]) #middle_distal
        targetPosList.append(targetCoord[15:18]) #middle_tip

        #applying vector transformation to generate unrotated coordinates of target pose
        r = R.from_quat([axisRot[0], axisRot[1], axisRot[2], axisRot[3]])
        
        for count in range(6):
            transformVec = r.apply(targetPosList[count], inverse = True)
            transformPosList.append(transformVec)

        leappybulletik.update_target_vis(transformPosList)
        leappybulletik.compute_IK(transformPosList)


        time.sleep(.01)

    p.disconnect()

if __name__ == "__main__":
    main()