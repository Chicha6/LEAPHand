import pybullet as p
import time
import os
import numpy as np
import zmq
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
import sys


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

        self.motors = motors = [1,0,2,3,5,4,6,7,9,8,10,11]
        # self.motors = motors = [1,0,2,3,5,4,6,7]


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
    def __init__(self, basePosAndScaling=None):
        
        #start pybullet
        physicsClient = p.connect(p.GUI)
        self.basePosAndScaling = basePosAndScaling
        self.glove_to_leap_mapping_scale = 1
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src = os.path.join(path_src, "right_hand_mesh/right_hand.urdf")

        # Here endEffector refers to the target joints(not links)
        # These correspond to the dip joints and fingertips of thumb, index, and middle
        self.leapEndEffectorIndex = [3, 4, 8, 9, 13, 14]
        self.LeapId = p.loadURDF( 
            path_src,
            [basePosAndScaling[0], basePosAndScaling[1], basePosAndScaling[2]] if self.basePosAndScaling != None 
            else [0.019,0.120,-0.016],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )
        self.numJoints = p.getNumJoints(self.LeapId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)

        self.create_target_vis()

    def create_target_vis(self):
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
    
    def update_target_vis(self, hand_pos, enable_offset):
        if (enable_offset):
            _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
            p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[0], current_orientation)
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
            p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[1], current_orientation)
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
            p.resetBasePositionAndOrientation(self.ballMbt[2], [hand_pos[2][0] + 0.02, hand_pos[2][1], hand_pos[2][2]], current_orientation)
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
            p.resetBasePositionAndOrientation(self.ballMbt[3], [hand_pos[3][0] + 0.02, hand_pos[3][1], hand_pos[3][2]], current_orientation)
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[4])
            p.resetBasePositionAndOrientation(self.ballMbt[4], [hand_pos[4][0] + 0.02, hand_pos[4][1], hand_pos[4][2]], current_orientation)
            _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[5])
            p.resetBasePositionAndOrientation(self.ballMbt[5], [hand_pos[5][0] + 0.02, hand_pos[5][1], hand_pos[5][2]] , current_orientation)
        else:
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

        

    # Computes and returns joint angles using IK and updates joints in simulation
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

        # Returns resultant joint angles
        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        
        # Additional zero values are for the rigid joints between fingertip and realtip
        # Because the above IK function takes the origin of the joint of the child link as the coordinates of the child link.
        # Hence to reference the extreme tip of the finger for IK, a realtip link was created at the end of each finger, with the joint positioned at the tip.
        # The additional realtip joint at each of the 3 fingers results in a total of 15 joints (rather than 12).
        # These additional joints are rigid, but stil recognised by setJointMotorControl2(), hence the 3 additional zero values
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,))

        # Updates the hand joints in simulator
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
        return jointPoses

def pybulletCalibration(socket):
    avgSampleIndexMiddle = [0,0,0,0,0,0,0,0,0,0,0,0]
    avgSampleThumb = [0,0,0,0,0,0]
    calibrationPoints = []
    URDFBaseOffset = [0,0,0]
    ManusToLeapScaling = [0,0,0]

    input("Press Enter to start index and middle sampling. Make sure you're in the correct calibration pose!")
    # Index and middle fingers sampling
    # Calibration pose: all fingers relaxed and closed, flat on table
    for n in range(50):
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")
        currentSample = list(map(float,manusData[7:]))

        for m in range(len(currentSample)):
            if (avgSampleIndexMiddle[m] != 0):
                avgSampleIndexMiddle[m] = (avgSampleIndexMiddle[m] + currentSample[m]) / 2
            else:
                avgSampleIndexMiddle[m] = currentSample[m]
    
    input("Press Enter to start thumb sampling. Make sure you're in the correct calibration pose!")
    # Thumb sampling
    # Calibration pose: all fingers relaxed and closed, flat on table, thumb pointing outwards a far as possible while keeping it straight
    for n in range(50):
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")
        currentSample = list(map(float,manusData[1:7]))

        for m in range(6):
            if (avgSampleThumb[m] != 0):
                avgSampleThumb[m] = (avgSampleThumb[m] + currentSample[m]) / 2
            else:
                avgSampleThumb[m] = currentSample[m]

    calibrationPoints.append(avgSampleThumb[0:3]) #thumb_distal
    calibrationPoints.append(avgSampleThumb[3:6]) #thumb_tip
    calibrationPoints.append(avgSampleIndexMiddle[0:3]) #index_distal
    calibrationPoints.append(avgSampleIndexMiddle[3:6]) #index_tip
    calibrationPoints.append(avgSampleIndexMiddle[6:9]) #middle_distal
    calibrationPoints.append(avgSampleIndexMiddle[9:12]) #middle_tip

    URDFBaseOffset[0] = 0.02 + calibrationPoints[5][0] #URDF base x-position
    URDFBaseOffset[1] = 0.062 + (calibrationPoints[0][1] + calibrationPoints[1][1]) / 2 #URDF base y-position
    URDFBaseOffset[2] = -0.007 #URDF base z-position
    ManusToLeapScaling[0] = (-0.209 + URDFBaseOffset[0]) / calibrationPoints[1][0] #thumb scaling
    ManusToLeapScaling[1] = (0.152 + URDFBaseOffset[1]) / calibrationPoints[3][1] #index scaling 
    ManusToLeapScaling[2] = (0.152 + URDFBaseOffset[1]) / calibrationPoints[5][1] #middle scaling
    
    print(URDFBaseOffset)
    return URDFBaseOffset, ManusToLeapScaling
    


def main(**kwargs):

    # leap_hand = LeapNode()
    context = zmq.Context()
    #Socket to talk to Manus SDK
    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000")
    URDFbaseOffset = []
    ManusToLeapScaling = []
    targetPosList = []
    leapInputAngles = []
    isCalibrated = False

    # Generate calibration data (optional) and initialise pybullet
    # Usage: "python MANUS_IK.py c" or "python MANUS_IK.py C"
    if(sys.argv[1] == "c" or sys.argv[1] == "C"):
        URDFbaseOffset, ManusToLeapScaling = pybulletCalibration(socket)
        print(URDFbaseOffset, ManusToLeapScaling)
        leappybulletik = LeapPybulletIK(URDFbaseOffset)
        isCalibrated = True
    else:
        leappybulletik = LeapPybulletIK()

    for n in range(10000):        
        targetPosList.clear()
        leapInputAngles.clear()
    
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")   

        # gloveID = manusData[0]
        targetCoord = list(map(float,manusData[1:]))
        targetPosList.append(targetCoord[0:3]) #thumb_distal
        targetPosList.append(targetCoord[3:6]) #thumb_tip
        targetPosList.append(targetCoord[6:9]) #index_distal
        targetPosList.append(targetCoord[9:12]) #index_tip
        targetPosList.append(targetCoord[12:15]) #middle_distal
        targetPosList.append(targetCoord[15:18]) #middle_tip

        # MANUS to LEAP scaling
        if (isCalibrated):
            for n in range(len(targetPosList)):
                for i in range(3):
                    if(n == 0 or n == 1):
                        targetPosList[n][i] = targetPosList[n][i] * ManusToLeapScaling[0]
                    elif(n == 2 or n == 3):
                        if(i == 0):
                            targetPosList[n][i] -= .015
                        targetPosList[n][i] = targetPosList[n][i] * ManusToLeapScaling[1]
                    else:
                        targetPosList[n][i] = targetPosList[n][i] * ManusToLeapScaling[2]
        else:        
            # Manually change MANUS to LEAP scaling if user chose not to calibrate 
            for n in range(len(targetPosList)):
                for i in range(3):
                    if(n == 0 or n == 1):
                        targetPosList[n][i] = targetPosList[n][i] * 1.45
                    elif(n == 2 or n == 3):
                        if(i == 0):
                            targetPosList[n][i] -= .015
                        targetPosList[n][i] = targetPosList[n][i] * 1.42
                    else:
                        targetPosList[n][i] = targetPosList[n][i] * 1.35

        leappybulletik.update_target_vis(targetPosList, False)
        IKAngles = leappybulletik.compute_IK(targetPosList)

        # joint angle polarity and magnitude correction for leap control
        for count in range(len(IKAngles)):
            if(count == 1 or count == 5):
                leapInputAngles.append(IKAngles[count] * -1 + 3.14)
            elif(count == 8 or count == 9 or count == 10 or count == 11 ):
                leapInputAngles.append(IKAngles[count] * -1 + 3.14)
            else:
                leapInputAngles.append(IKAngles[count] + 3.14)

        # leap_hand.set_leap(leapInputAngles)
        time.sleep(.02)

    p.disconnect()

if __name__ == "__main__":
    main()