import pybullet as p
import time
import os
import numpy as np
import zmq
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time
import sys

leftHandID = "558097a3"
rightHandID = "e13e29f2"

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
        # self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

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
    def __init__(self, endEffectors, URDFOffset_left=None, URDFOffset_right=None):
        #start pybullet
        p.connect(p.GUI)
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src_right = os.path.join(path_src, "right_hand_mesh/right_hand.urdf")
        path_src_left = os.path.join(path_src, "left_hand_mesh/left_hand.urdf")

        # Here endEffector refers to the target joints(not links)
        # These correspond to the dip joints and fingertips of thumb, index, and middle
        self.leapEndEffectorIndex = endEffectors

        #load left hand
        self.bodyIdLeft = p.loadURDF( 
            path_src_left,
            [URDFOffset_left[0], URDFOffset_left[1], URDFOffset_left[2]] if URDFOffset_left != None 
            else [-0.45,0.120,-0.016],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )

        # load right hand
        self.bodyIdRight = p.loadURDF( 
            path_src_right,
            [URDFOffset_right[0], URDFOffset_right[1], URDFOffset_right[2]] if URDFOffset_right != None 
            else [0.019,0.120,-0.016],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )
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
        for i in range(len(self.leapEndEffectorIndex) * 2):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)

        for i in range(len(self.leapEndEffectorIndex)):
            p.changeVisualShape(self.ballMbt[2*i], -1, rgbaColor=[0, 0, 1, 1])
            p.changeVisualShape(self.ballMbt[2*i+1], -1, rgbaColor=[1, 0, 0, 1])

    def update_target_vis(self, hand_pos_left=None, hand_pos_right=None):
        if(hand_pos_left):
            for i in range(len(self.leapEndEffectorIndex)):
                _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[i])
                p.resetBasePositionAndOrientation(self.ballMbt[i], hand_pos_left[i], current_orientation)
        if(hand_pos_right):
            for i in range(len(self.leapEndEffectorIndex)):
                _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[i+len(self.leapEndEffectorIndex)])
                p.resetBasePositionAndOrientation(self.ballMbt[i+len(self.leapEndEffectorIndex)], hand_pos_right[i], current_orientation)
        
    # Computes and returns joint angles using IK and updates joints in simulation
    def compute_IK(self, hand_pos_left=None, hand_pos_right=None):
        p.stepSimulation()     
        jointAnglesLeft = None
        jointAnglesRight = None
        if (hand_pos_left):
            endEffectorPosLeft = []
            for pos in hand_pos_left:
                endEffectorPosLeft.append(pos)
            jointAnglesLeft = p.calculateInverseKinematics2(
            self.bodyIdLeft,
            self.leapEndEffectorIndex,
            endEffectorPosLeft,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )

        if (hand_pos_right):
            endEffectorPosRight = []
            for pos in hand_pos_right:
                endEffectorPosRight.append(pos)
            jointAnglesRight = p.calculateInverseKinematics2(
            self.bodyIdRight,
            self.leapEndEffectorIndex,
            endEffectorPosRight,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        return jointAnglesLeft, jointAnglesRight
    
    def setMotorControl(self, jointAnglesLeft, jointAnglesRight):

        combined_jointAngles_left = None
        combined_jointAngles_right = None
         
        # Additional zero values are for the rigid joints between fingertip and realtip
        # Because the above IK function takes the origin of the joint of the child link as the coordinates of the child link.
        # Hence to reference the extreme tip of the finger for IK, a realtip link was created at the end of each finger, with the joint positioned at the tip.
        # The additional realtip joint at each of the 3 fingers results in a total of 15 joints (rather than 12).
        # These additional joints are rigid, but stil recognised by setJointMotorControl2(), hence the 3 additional zero values
        if(jointAnglesLeft):
            combined_jointAngles_left = (jointAnglesLeft[0:4] + (0.0,) + jointAnglesLeft[4:8] + (0.0,) + jointAnglesLeft[8:12] + (0.0,))
        if(jointAnglesRight):
            combined_jointAngles_right = (jointAnglesRight[0:4] + (0.0,) + jointAnglesRight[4:8] + (0.0,) + jointAnglesRight[8:12] + (0.0,))

        for i in range(15):
            if (combined_jointAngles_left):
                p.setJointMotorControl2(
                    bodyIndex=self.bodyIdLeft,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=combined_jointAngles_left[i],
                    targetVelocity=0,
                    force=500,
                    positionGain=0.3,
                    velocityGain=1,
                )
            if (combined_jointAngles_right):
                p.setJointMotorControl2(
                    bodyIndex=self.bodyIdRight,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=combined_jointAngles_right[i],
                    targetVelocity=0,
                    force=500,
                    positionGain=0.3,
                    velocityGain=1,
                )

def pybulletCalibration(socket, side):
    avgSampleIndexMiddle = [0,0,0,0,0,0,0,0,0,0,0,0]
    avgSampleThumb = [0,0,0,0,0,0]
    calibrationPoints = []

    URDFBaseOffset = [0,0,0]
    ManusToLeapScaling = [0,0,0]
    indexXOffset = 0

    # Index and middle fingers sampling
    # Calibration pose: all fingers relaxed (not closed), flat on table
    input(f"Press Enter to start {"left hand" if side == 0 else "right hand"} index and middle sampling. Make sure you're in the correct calibration pose!")
    for n in range(50):
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")

        #if both hands in use
        if(len(manusData) > 19):
            #if sampling left hand
            if(side == 0):
                #checks if the left hand data comes first or second in manusData
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[7:19]))
                else:
                    currentSample = list(map(float,manusData[26:38]))
            #if sampling right hand
            else:
                #checks if the right hand data comes first or second in manusData
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[26:38]))
                else:
                    currentSample = list(map(float,manusData[7:19]))
        #if only one hand in use
        else:
            currentSample = list(map(float,manusData[7:19]))

        #calculate average of samples  
        for m in range(len(currentSample)):
            if (avgSampleIndexMiddle[m] != 0):
                avgSampleIndexMiddle[m] = (avgSampleIndexMiddle[m] + currentSample[m]) / 2
            else:
                avgSampleIndexMiddle[m] = currentSample[m]
    
    # Thumb sampling
    # Calibration pose: all fingers relaxed (not closed), flat on table, thumb pointing outwards a far as possible while keeping it straight
    input(f"Press Enter to start {"left hand" if side == 0 else "right hand"} thumb sampling. Make sure you're in the correct calibration pose!")
    for n in range(50):
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")

        #if both hands
        if(len(manusData) > 20):
            #if sampling left hand
            if(side == 0):
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[1:7]))
                else:
                    currentSample = list(map(float,manusData[20:26]))

            #if sampling right hand                
            else:
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[20:26]))
                else:
                    currentSample = list(map(float,manusData[1:7]))
        #if only one hand in use
        else:
            currentSample = list(map(float,manusData[1:7]))

        #calculate average of samples
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

    if(side == 0):
        URDFBaseOffset[0] = -0.02 + calibrationPoints[5][0] #left URDF base x-position
        ManusToLeapScaling[0] = (0.209 + URDFBaseOffset[0]) / calibrationPoints[1][0] #left thumb scaling
    else:
        URDFBaseOffset[0] = 0.02 + calibrationPoints[5][0] #right URDF base x-position
        ManusToLeapScaling[0] = (-0.209 + URDFBaseOffset[0]) / calibrationPoints[1][0] #right thumb scaling

    URDFBaseOffset[1] = 0.062 + (calibrationPoints[0][1]) * ManusToLeapScaling[0] #URDF base y-position
    URDFBaseOffset[2] = -0.007 #URDF base z-position
    ManusToLeapScaling[1] = (0.152 + URDFBaseOffset[1]) / calibrationPoints[3][1] #index scaling 
    ManusToLeapScaling[2] = (0.152 + URDFBaseOffset[1]) / calibrationPoints[5][1] #middle scaling
    indexXOffset = ((0.0762 if side == 0 else -0.0762) + URDFBaseOffset[0] - calibrationPoints[3][0] * ManusToLeapScaling[1]) #index x-axis offset

    if(side == 0):
        URDFBaseOffset[0] += -0.45
    return URDFBaseOffset, ManusToLeapScaling, indexXOffset
    

def main(**kwargs):

    # leap_hand = LeapNode()
    context = zmq.Context()
    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000")
    URDFOffset_left = []
    URDFOffset_right = []
    ManusToLeapScaling_left = []
    ManusToLeapScaling_right = []
    indexXOffset_left = 0
    indexXOffset_right = 0
    leftTargetPosList = []
    rightTargetPosList = []
    leapInputAnglesLeft = []
    leapInputAnglesRight = []
    isCalibrated = False
    isLeftOn = False
    isRightOn = False
    leappybulletik = None
    endEffectors = [3, 4, 8, 9, 13, 14]

    message = socket.recv()
    message = message.decode('utf-8')
    manusData = message.split(",")   

    # Generate calibration data (optional) and initialise pybullet
    # Usage: "python MANUS_IK.py c" or "python MANUS_IK.py C"
    if (len(sys.argv) == 2):
        if(sys.argv[1] == "c" or sys.argv[1] == "C"):
        
            # Checks to initiate IK for both hands or one hand only
            if(len(manusData) > 19):
                URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socket, 0)
                URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socket, 1)
                leappybulletik = LeapPybulletIK(endEffectors,URDFOffset_left=URDFOffset_left, URDFOffset_right=URDFOffset_right)
                isCalibrated = True
                isLeftOn, isRightOn = True, True
            else:
                #left hand only
                if(manusData[0] == leftHandID):
                    URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socket, 0)
                    leappybulletik = LeapPybulletIK(endEffectors,URDFOffset_left=URDFOffset_left)
                    isCalibrated = True
                    isLeftOn = True

                #right hand only
                if(manusData[0] == rightHandID):
                    URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socket, 1)
                    leappybulletik = LeapPybulletIK(endEffectors,URDFOffset_right=URDFOffset_right)
                    isCalibrated = True
                    isRightOn = True
    #initialise IK without calibration (to use presets)
    else:
        if(len(manusData) > 20):
            isLeftOn, isRightOn = True, True
        elif(manusData[0] == leftHandID):
            isLeftOn = True
        else:
            isRightOn = True
        leappybulletik = LeapPybulletIK([3, 4, 8, 9, 13, 14])


    #start of teleop
    for n in range(100000):        
        leftTargetPosList.clear()
        rightTargetPosList.clear()
        leapInputAnglesRight.clear()
    
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")   

        #both hands in use
        if(len(manusData) > 19):
            if(manusData[0] == leftHandID):
                targetCoord = list(map(float,manusData[1:19]))
                for n in range(6):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
                targetCoord = list(map(float,manusData[20:38]))
                for n in range(6):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                targetCoord = list(map(float,manusData[1:19]))
                for n in range(6):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
                targetCoord = list(map(float,manusData[20:38]))
                for n in range(6):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
            targetCoord = list(map(float,manusData[1:19]))
            if(manusData[0] == leftHandID):
                for n in range(6):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                for n in range(6):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
        #one hand in use only
        else:
            targetCoord = list(map(float,manusData[1:19]))
            if(manusData[0] == leftHandID):
                for n in range(6):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                for n in range(6):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])

        # MANUS to LEAP scaling
        if(isLeftOn):
            if(isCalibrated):
                for n in range(6):
                    for i in range(3):
                        leftTargetPosList[n][i] = leftTargetPosList[n][i] * ManusToLeapScaling_left[n//2]
                        if((n == 2 or n == 3) and i == 0):
                            leftTargetPosList[n][0] += indexXOffset_left
                for n in range(6):
                    leftTargetPosList[n][0] += -0.45
            else:
                for i in range(3):
                    leftTargetPosList[0][i] = leftTargetPosList[0][i] * 1.45
                    leftTargetPosList[1][i] = leftTargetPosList[1][i] * 1.45
                    leftTargetPosList[2][i] = leftTargetPosList[2][i] * 1.42
                    leftTargetPosList[3][i] = leftTargetPosList[3][i] * 1.42
                    leftTargetPosList[4][i] = leftTargetPosList[4][i] * 1.35
                    leftTargetPosList[5][i] = leftTargetPosList[5][i] * 1.35
                leftTargetPosList[2][0] += .015
                leftTargetPosList[3][0] += .015
                for n in range(6):
                    leftTargetPosList[n][0] += -0.45 + 0.019
         
        if(isRightOn):
            if(isCalibrated):
                for n in range(6):
                    for i in range(3):
                        rightTargetPosList[n][i] = rightTargetPosList[n][i] * ManusToLeapScaling_right[n//2]
                        if((n == 2 or n == 3) and i == 0):
                            rightTargetPosList[n][0] += indexXOffset_right
            else:
                for i in range(3):
                    rightTargetPosList[0][i] = rightTargetPosList[0][i] * 1.45
                    rightTargetPosList[1][i] = rightTargetPosList[1][i] * 1.45
                    rightTargetPosList[2][i] = rightTargetPosList[2][i] * 1.42
                    rightTargetPosList[3][i] = rightTargetPosList[3][i] * 1.42
                    rightTargetPosList[4][i] = rightTargetPosList[4][i] * 1.35
                    rightTargetPosList[5][i] = rightTargetPosList[5][i] * 1.35
                rightTargetPosList[2][0] -= .015
                rightTargetPosList[3][0] -= .015

        leappybulletik.update_target_vis(leftTargetPosList, rightTargetPosList)
        IKAngles_left, IKAngles_right = leappybulletik.compute_IK(hand_pos_left=leftTargetPosList, hand_pos_right=rightTargetPosList)
        leappybulletik.setMotorControl(IKAngles_left, IKAngles_right)

        # joint angle polarity and magnitude correction for LEAP hand control
        # for count in range(12):
        #     if(count in [0,1,2,3,5,9]):
        #         leapInputAnglesRight.append(IKAngles_right[count] * -1 + 3.14)
        #     else:
        #         leapInputAnglesRight.append(IKAngles_right[count] + 3.14)

        # print(leapInputAnglesRight)
        # leap_hand.set_leap(leapInputAnglesRight)

        time.sleep(.015)

    p.disconnect()

if __name__ == "__main__":
    main()