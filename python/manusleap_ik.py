import pybullet as p
import time
import os
import zmq
import time
import sys
from leap_main import LeapNode

"""
This script takes joint positions from MANUS SDK and computes inverse kinematics to generate joint angles to control robot hand.
Additional calibration, as compared to the FK mode, is required to account for morphological differences between human and robot hand.

"""

# Set glove ID here
leftHandID = "558097a3"
rightHandID = "e13e29f2"

# Set U2D2 COM port here
leftCOM = "COM5"
rightCOM = "COM6"

class LeapPybulletIK():
    def __init__(self, endEffectors, URDFOffset_left, URDFOffset_right):
        #start pybullet
        p.connect(p.GUI)
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src_left = os.path.join(path_src, "left_hand_mesh/left_hand.urdf")
        path_src_right = os.path.join(path_src, "right_hand_mesh/right_hand.urdf")
        
        # Here endEffector refers to the target joints(not links)
        # These correspond to the DIP and fingertips of thumb, index, and middle
        self.leapEndEffectorIndex = endEffectors

        # Load left hand URDF
        # URDFOffset sets the base position of the URDF model. If its not provided, the default values will be used.
        self.bodyIdLeft = p.loadURDF( 
            path_src_left,
            [URDFOffset_left[0], URDFOffset_left[1], URDFOffset_left[2]],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )

        # Load right hand URDF
        self.bodyIdRight = p.loadURDF( 
            path_src_right,
            [URDFOffset_right[0], URDFOffset_right[1], URDFOffset_right[2]],
            p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase = True,
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()

    # Initialise visualisers for target positions of end effectors
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

    # Update position of visualisers of target positions
    def update_target_vis(self, hand_pos_left=None, hand_pos_right=None):
        if(hand_pos_left):
            for i in range(len(self.leapEndEffectorIndex)):
                _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[i])
                p.resetBasePositionAndOrientation(self.ballMbt[i], hand_pos_left[i], current_orientation)
        if(hand_pos_right):
            for i in range(len(self.leapEndEffectorIndex)):
                _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[i+len(self.leapEndEffectorIndex)])
                p.resetBasePositionAndOrientation(self.ballMbt[i+len(self.leapEndEffectorIndex)], hand_pos_right[i], current_orientation)
        
    # Solves IK and return joint angles
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
    
    # Set motor position in simulation
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


# This can be called before the computing IK to generate 3 sets of calibration values.
# It samples 50 fingertip positions for every finger and calculates the calibration values.
## URDFBaseOffset: Used when loading URDF models into simulation during Pybullet intialisation to match the coordinate systems of the MANUS glove and the robot hand in the simulation scene. 
## ManusToLeapScaling: Used during teleoperation phase to account for the difference in finger lengths between the hand and the robot
## indexXOffset: Used during teleoperation phase to account for the difference in the index-middle spacing between the hand and the robot

def pybulletCalibration(socket, side):
    avgSampleIndexMiddle = [0,0,0,0,0,0,0,0,0,0,0,0]
    avgSampleThumb = [0,0,0,0,0,0]
    samplePoints = []

    # Calibration values to be generated
    URDFBaseOffset = [0,0,0]
    ManusToLeapScaling = [0,0,0]
    indexXOffset = 0

    # Index and middle fingers sampling
    # Calibration pose: all fingers relaxed (not closed), flat on table
    input(f"Press Enter to start {"left hand" if side == 0 else "right hand"} index and middle sampling. Make sure you're in the correct calibration pose!")
    for n in range(50):

        # This message contains data for both left and right hands 
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")

        # If both hands in use
        if(len(manusData) > 19):
            # If sampling left hand
            if(side == 0):
                # Checks if the left hand data comes first or second in manusData
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[7:19]))
                else:
                    currentSample = list(map(float,manusData[26:38]))
            # If sampling right hand
            else:
                # Checks if the right hand data comes first or second in manusData
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[26:38]))
                else:
                    currentSample = list(map(float,manusData[7:19]))
        # If only one hand in use
        else:
            currentSample = list(map(float,manusData[7:19]))

        # Calculate average of samples  
        for m in range(len(currentSample)):
            if (avgSampleIndexMiddle[m] != 0):
                avgSampleIndexMiddle[m] = (avgSampleIndexMiddle[m] + currentSample[m]) / 2
            else:
                avgSampleIndexMiddle[m] = currentSample[m]
    
    # Thumb sampling
    # Calibration pose: all fingers relaxed (not closed), flat on table, thumb pointing outwards as far as possible while keeping it straight
    input(f"Press Enter to start {"left hand" if side == 0 else "right hand"} thumb sampling. Make sure you're in the correct calibration pose!")
    for n in range(50):
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")

        # If both hands
        if(len(manusData) > 19):
            # If sampling left hand
            if(side == 0):
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[1:7]))
                else:
                    currentSample = list(map(float,manusData[20:26]))

            # If sampling right hand                
            else:
                if(manusData[0] == leftHandID):
                    currentSample = list(map(float,manusData[20:26]))
                else:
                    currentSample = list(map(float,manusData[1:7]))
        # If only one hand in use
        else:
            currentSample = list(map(float,manusData[1:7]))

        # Calculate average of samples
        for m in range(6):
            if (avgSampleThumb[m] != 0):
                avgSampleThumb[m] = (avgSampleThumb[m] + currentSample[m]) / 2
            else:
                avgSampleThumb[m] = currentSample[m]

    
    samplePoints.append(avgSampleThumb[0:3]) #thumb_distal
    samplePoints.append(avgSampleThumb[3:6]) #thumb_tip
    samplePoints.append(avgSampleIndexMiddle[0:3]) #index_distal
    samplePoints.append(avgSampleIndexMiddle[3:6]) #index_tip
    samplePoints.append(avgSampleIndexMiddle[6:9]) #middle_distal
    samplePoints.append(avgSampleIndexMiddle[9:12]) #middle_tip
    # These sample points are used in the following calibration value calculation

    if(side == 0):
        URDFBaseOffset[0] = -0.02 + samplePoints[5][0] #left URDF base x-position
        ManusToLeapScaling[0] = (0.209 + URDFBaseOffset[0]) / samplePoints[1][0] #left thumb scaling
    else:
        URDFBaseOffset[0] = 0.02 + samplePoints[5][0] #right URDF base x-position
        ManusToLeapScaling[0] = (-0.209 + URDFBaseOffset[0]) / samplePoints[1][0] #right thumb scaling

    URDFBaseOffset[1] = 0.062 + (samplePoints[0][1]) * ManusToLeapScaling[0] #URDF base y-position
    URDFBaseOffset[2] = -0.016 #URDF base z-position
    ManusToLeapScaling[1] = (0.152 + URDFBaseOffset[1]) / samplePoints[3][1] #index scaling 
    ManusToLeapScaling[2] = (0.152 + URDFBaseOffset[1]) / samplePoints[5][1] #middle scaling
    indexXOffset = (0.0762 if side == 0 else -0.0762) + URDFBaseOffset[0] - samplePoints[3][0] * ManusToLeapScaling[1] #index x-axis offset
    
    if(side == 0):
        URDFBaseOffset[0] += -0.45 # This is so that the left hand does not overlap with the right hand in the simulation

    # print(URDFBaseOffset, ManusToLeapScaling, indexXOffset)
    
    return URDFBaseOffset, ManusToLeapScaling, indexXOffset
    

def main(**kwargs):

    # Motor IDs, can be changed in Dynamixel Wizard
    ## Thumb  -> 0: MCP flex, 1: MCP side, 2: PIP flex, 3: DIP flex
    ## Index -> 4: MCP flex, 5: MCP side, 6: PIP flex, 7: DIP flex
    ## Middle  -> 8: MCP flex, 9: MCP side, 10: PIP flex, 11: DIP flex
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]
    leap_hand_left = None
    leap_hand_right = None
    
    # Set up ZMQ to talk to Manus SDK
    context = zmq.Context()
    print("Connecting to SDK...")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:8000") # Set IP and port to receive data from MANUS SDK
    print("Connected to SDK, Waiting for message ...")

    # For IK calibration and scaling 
    URDFOffset_left = []
    URDFOffset_right = []
    ManusToLeapScaling_left = []
    ManusToLeapScaling_right = []
    indexXOffset_left = 0
    indexXOffset_right = 0

    isCalibrated = False 
    isLeftOn = False
    isRightOn = False

    leappybulletik = None
    endEffectors = [3, 4, 8, 9, 13, 14] 
    
    # Stores endeffector coordinates for IK calculation
    leftTargetPosList = [] 
    rightTargetPosList = []

    # Stores joint angles to control robot
    leapInputAnglesLeft = []
    leapInputAnglesRight = [] 

    enableLeap = False # Set to false if not using robot hand

    # Check if message is being sent from MANUS SDK
    while True:
        try:
            message = socket.recv(flags=zmq.NOBLOCK)
            print("Message received.")
            message = message.decode('utf-8')
            manusData = message.split(",")   
            break
        except zmq.error.Again:
            print("No message received yet.")
        
    # Generate run calibration (optional) and initialise pybullet
    # Usage: "python manusleap_ik.py c" or "manusleap_ik.py C"
    if (len(sys.argv) == 2):
        if(sys.argv[1] == "c" or sys.argv[1] == "C"):
            # Checks to initiate IK for both hands or one hand only
            if(len(manusData) > 19):
                URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socket, 0)
                URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socket, 1)
                leappybulletik = LeapPybulletIK(endEffectors,URDFOffset_left,URDFOffset_right)
                isCalibrated = isLeftOn = isRightOn = True
            else:
                # Left hand only
                if(manusData[0] == leftHandID):
                    URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socket, 0)
                    leappybulletik = LeapPybulletIK(endEffectors,URDFOffset_left,[0,0,0])
                    isCalibrated = isLeftOn = True
                # Light hand only
                if(manusData[0] == rightHandID):
                    URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socket, 1)
                    leappybulletik = LeapPybulletIK(endEffectors,[0,0,0],URDFOffset_right)
                    isCalibrated = isRightOn = True

    # Initialise IK without calibration (to set URDFOffset manually)
    else:
        if(len(manusData) > 19):
            isLeftOn = isRightOn = True
        elif(manusData[0] == leftHandID):
            isLeftOn = True
        else:
            isRightOn = True
        leappybulletik = LeapPybulletIK(endEffectors, [-0.481,0.126,-0.016], [0.023,0.127,-0.016])

    # Initialise robot hands
    if(enableLeap == True):
        if(len(manusData) > 19):
            leap_hand_left = LeapNode(leftCOM, motor_ids_left)
            leap_hand_right = LeapNode(rightCOM, motor_ids_right)
        else:
            if(manusData[0] == leftHandID):
                leap_hand_left = LeapNode(leftCOM, motor_ids_left)
            else:
                leap_hand_right = LeapNode(rightCOM, motor_ids_right)

    # Start of teleoperation
    for n in range(100000):        
        leftTargetPosList.clear()
        rightTargetPosList.clear()
        leapInputAnglesLeft.clear()
        leapInputAnglesRight.clear()

        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")   

        # Load leftTargetPosList and rightTargetPosList (if both hands in use)
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
        # Load leftTargetPosList or rightTargetPosList (if one hand in use)   
        else:
            targetCoord = list(map(float,manusData[1:19]))
            if(manusData[0] == leftHandID):
                for n in range(6):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                for n in range(6):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])

        # MANUS data to robot hand scaling (left hand)
        if(isLeftOn):
            # Use auto-calibration values (Scaling and index finger offset)
            if(isCalibrated):
                for n in range(6):
                    for i in range(3):
                        leftTargetPosList[n][i] = leftTargetPosList[n][i] * ManusToLeapScaling_left[n//2]
                        if((n == 2 or n == 3) and i == 0):
                            leftTargetPosList[n][0] += indexXOffset_left
                for n in range(6):
                    leftTargetPosList[n][0] += -0.45 # Translates left hand coordinates to match the translated left hand model in PyBullet
            # Use manual scaling and offsets
            else:
                # Scaling
                for i in range(3):
                    leftTargetPosList[0][i] = leftTargetPosList[0][i] * 1.65
                    leftTargetPosList[1][i] = leftTargetPosList[1][i] * 1.65
                    leftTargetPosList[2][i] = leftTargetPosList[2][i] * 1.52
                    leftTargetPosList[3][i] = leftTargetPosList[3][i] * 1.52
                    leftTargetPosList[4][i] = leftTargetPosList[4][i] * 1.48
                    leftTargetPosList[5][i] = leftTargetPosList[5][i] * 1.48
                # Index offset
                leftTargetPosList[2][0] += .015
                leftTargetPosList[3][0] += .015

                for n in range(6):
                    leftTargetPosList[n][0] += -0.45 # Translates left hand coordinates to match the translated left hand model in PyBullet
         
        # MANUS data to robot hand scaling (right hand)
        if(isRightOn):
            # Use auto-calibration values (Scaling and index finger offset)
            if(isCalibrated):
                for n in range(6):
                    for i in range(3):
                        rightTargetPosList[n][i] = rightTargetPosList[n][i] * ManusToLeapScaling_right[n//2]
                        if((n == 2 or n == 3) and i == 0):
                            rightTargetPosList[n][0] += indexXOffset_right
            # Use manual scaling and offsets
            else:
                # Scaling
                for i in range(3):
                    rightTargetPosList[0][i] = rightTargetPosList[0][i] * 1.55
                    rightTargetPosList[1][i] = rightTargetPosList[1][i] * 1.55
                    rightTargetPosList[2][i] = rightTargetPosList[2][i] * 1.58
                    rightTargetPosList[3][i] = rightTargetPosList[3][i] * 1.58
                    rightTargetPosList[4][i] = rightTargetPosList[4][i] * 1.45
                    rightTargetPosList[5][i] = rightTargetPosList[5][i] * 1.45
                # Index offset
                rightTargetPosList[2][0] -= .015
                rightTargetPosList[3][0] -= .015

        leappybulletik.update_target_vis(leftTargetPosList, rightTargetPosList)
        IKAngles_left, IKAngles_right = leappybulletik.compute_IK(hand_pos_left=leftTargetPosList, hand_pos_right=rightTargetPosList)
        leappybulletik.setMotorControl(IKAngles_left, IKAngles_right)

        # Loads leapInputAnglesLeft and leapInputAnglesRight and adds 180 degrees because motors are centered at 180
        if(enableLeap == True):
            if(leap_hand_left):
                for count in range(12):
                    leapInputAnglesLeft.append(IKAngles_left[count] + 3.14)
            if(leap_hand_right):
                for count in range(12):
                    leapInputAnglesRight.append(IKAngles_right[count] + 3.14)
        
        # Set robot hand pose
        if(enableLeap == True):
            if(leap_hand_left):
                leap_hand_left.set_leap(leapInputAnglesLeft)
            if(leap_hand_right):
                leap_hand_right.set_leap(leapInputAnglesRight)

        time.sleep(.015)

    p.disconnect()

if __name__ == "__main__":
    main()