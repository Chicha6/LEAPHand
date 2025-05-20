import time
import zmq
import time
import sys
import numpy as np
from leap_main import LeapNode
from manusleap_ik import LeapPybulletIK, pybulletCalibration, p

"""
This script takes joint positions and joint angles from MANUS SDK to combine both IK and FK modes.
The thumb is controlled using IK while the index and middle are controlled using FK.
Similar to the IK only mode, additional calibration (thumb only) is required.

"""

#Set glove ID here
leftHandID = "558097a3"
rightHandID = "e13e29f2"

#Set U2D2 COM port here
leftCOM = "COM5"
rightCOM = "COM6"

def main(**kwargs):
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]

    # Set up ZMQ to talk to Manus SDK
    context = zmq.Context()

    socketRaw = context.socket(zmq.PULL)
    socketRaw.setsockopt(zmq.CONFLATE, True)     
    socketRaw.connect("tcp://127.0.0.1:8000") # Set IP and port to receive raw skeleton data (thumb only) from MANUS SDK

    socketErgo = context.socket(zmq.PULL)
    socketErgo.setsockopt(zmq.CONFLATE, True)     
    socketErgo.connect("tcp://127.0.0.1:9000") # Set IP and port to receive ergonomic data (index and middle only) from MANUS SDK

    # For IK calibration and scaling (thumb only)
    URDFOffset_left = []
    URDFOffset_right = []
    ManusToLeapScaling_left = []
    ManusToLeapScaling_right = []

    isCalibrated = False 
    isLeftOn = False 
    isRightOn = False

    # Stores endeffector coordinates for IK calculation (thumb only)
    leftTargetPosList = []
    rightTargetPosList = []

    # Stores joint angles to control robot
    leapInputAnglesLeft = [0,0,0,0,0,0,0,0,0,0,0,0]
    leapInputAnglesRight = [0,0,0,0,0,0,0,0,0,0,0,0]

    leap_hand_left = None
    leap_hand_right = None
    isLeftOn = False
    isRightOn = False
    leappybulletik = None

    enableLeap = True # Set to false if not using robot hand

    # Checking if message is being sent from MANUS SDK
    while True:
        try:
            message = socketRaw.recv(flags=zmq.NOBLOCK)
            message = socketErgo.recv(flags=zmq.NOBLOCK)
            print("Correct data received.")
            break
        except zmq.error.Again:
            print("Correct data not received yet! Make to select option 3 in MANUS SDK.")
        time.sleep(0.1) 
    
    message = socketRaw.recv()
    message = message.decode('utf-8')
    manusData = message.split(",")   

    # Generate calibration data (optional) and initialise pybullet
    # Usage: "python MANUS_IK.py c" or "python MANUS_IK.py C"
    if (len(sys.argv) == 2):
        if(sys.argv[1] == "c" or sys.argv[1] == "C"):
        
            # Checks to initiate whether IK for both hands or either one
            if(len(manusData) > 19):
                URDFOffset_left, ManusToLeapScaling_left, _ = pybulletCalibration(socketRaw, 0)
                URDFOffset_right, ManusToLeapScaling_right, _ = pybulletCalibration(socketRaw, 1)
                leappybulletik = LeapPybulletIK([3,4],URDFOffset_left, URDFOffset_right)
                isCalibrated = isLeftOn = isRightOn = True
            else:
                # Left hand only
                if(manusData[0] == leftHandID):
                    URDFOffset_left, ManusToLeapScaling_left, _ = pybulletCalibration(socketRaw, 0)
                    leappybulletik = LeapPybulletIK([3,4],URDFOffset_left,[0,0,0])
                    isCalibrated = isLeftOn = True
                # Right hand only
                if(manusData[0] == rightHandID):
                    URDFOffset_right, ManusToLeapScaling_right, _ = pybulletCalibration(socketRaw, 1)
                    leappybulletik = LeapPybulletIK([3,4],[0,0,0],URDFOffset_right)
                    isCalibrated = isRightOn = True

    # Initialise IK without calibration (to set URDFOffset manually)
    else:
        if(len(manusData) > 19):
            isLeftOn = isRightOn = True
        elif(manusData[0] == leftHandID):
            isLeftOn = True
        else:
            isRightOn = True
        leappybulletik = LeapPybulletIK([3,4], [-0.481,0.126,-0.016], [0.023,0.127,-0.016])

    # Initialise robot hands
    if(enableLeap == True):
        if(isLeftOn):
            leap_hand_left = LeapNode(leftCOM, motor_ids_left)
        if(isRightOn):    
            leap_hand_right = LeapNode(rightCOM, motor_ids_right)

    # Start of teleop
    for count in range(100000):
        leftTargetPosList.clear()
        rightTargetPosList.clear()

        ## Start of IK
        messageRaw = socketRaw.recv()
        messageRaw = messageRaw.decode('utf-8')
        manusRawData = messageRaw.split(",")      
    	
        # Load leftTargetPosList and rightTargetPosList (if both hands in use)
        if(len(manusRawData) > 19):
            if(manusRawData[0] == leftHandID):
                targetCoord = list(map(float,manusRawData[1:7]))
                for n in range(2):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
                targetCoord = list(map(float,manusRawData[20:26]))
                for n in range(2):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                targetCoord = list(map(float,manusRawData[1:7]))
                for n in range(2):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
                targetCoord = list(map(float,manusRawData[20:26]))
                for n in range(2):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
        # Load leftTargetPosList or rightTargetPosList (if one hand in use)
        else:
            targetCoord = list(map(float,manusRawData[1:7]))
            if(manusRawData[0] == leftHandID):
                for n in range(2):
                    leftTargetPosList.append(targetCoord[3*n:3*n+3])
            else:
                for n in range(2):
                    rightTargetPosList.append(targetCoord[3*n:3*n+3])
        
        # MANUS to LEAP scaling (left hand)
        if(isLeftOn):
            # Use auto-calibration values (thumb scaling only)
            if(isCalibrated):
                for n in range(2):
                    for i in range(3):
                        leftTargetPosList[n][i] = leftTargetPosList[n][i] * ManusToLeapScaling_left[n//2]
                
                for n in range(2):
                    leftTargetPosList[n][0] += -0.45 # Translates left hand coordinates to match the translated left hand model in PyBullet
            # Use manual scaling
            else:
                for i in range(3):
                    leftTargetPosList[0][i] = leftTargetPosList[0][i] * 1.65
                    leftTargetPosList[1][i] = leftTargetPosList[1][i] * 1.65
                
                for n in range(2):
                    leftTargetPosList[n][0] += -0.45 # Translates left hand coordinates to match the translated left hand model in PyBullet

        # MANUS to LEAP scaling (right hand)
        if(isRightOn):
            # Use auto-calibration values (thumb scaling only)
            if(isCalibrated):
                for n in range(2):
                    for i in range(3):
                        rightTargetPosList[n][i] = rightTargetPosList[n][i] * ManusToLeapScaling_right[n//2]
            # Use manual scaling
            else:
                for i in range(3):
                    rightTargetPosList[0][i] = rightTargetPosList[0][i] * 1.55
                    rightTargetPosList[1][i] = rightTargetPosList[1][i] * 1.55
        
        leappybulletik.update_target_vis(leftTargetPosList, rightTargetPosList)
        IKAngles_left, IKAngles_right = leappybulletik.compute_IK(hand_pos_left=leftTargetPosList, hand_pos_right=rightTargetPosList)
        ##end of IK

        ##start of FK
        messageErgo = socketErgo.recv()
        messageErgo = messageErgo.decode('utf-8')
        manusErgoData = messageErgo.split(",")

        leftErgoData = np.deg2rad(list(map(float,manusErgoData[1:9])))
        rightErgoData = np.deg2rad(list(map(float,manusErgoData[10:18])))

        if(isLeftOn):
             for n in range(4,12):
                if (n == 5 or n == 9):
                    # Correcting polarity and amplifying splay
                    leapInputAnglesLeft[n] = leftErgoData[n-4] * -1.4 + 3.14
                else:
                    leapInputAnglesLeft[n] = leftErgoData[n-4] + 3.14

        if(isRightOn):
            for n in range(4,12):
                if (n == 5 or n == 9):
                    # Correcting polarity and amplifying splay
                    leapInputAnglesRight[n] = rightErgoData[n-4] *-1.4 + 3.14
                elif(n == 4 or n == 8):
                    leapInputAnglesRight[n] = rightErgoData[n-4] * 1.1 + 3.14
                else:
                    leapInputAnglesRight[n] = rightErgoData[n-4] + 3.14
        
        
        # Loads leapInputAnglesLeft and leapInputAnglesRight and adds 180 degrees because motors are centered at 180
        if(enableLeap == True):
            if(isLeftOn):
                for count in range(4):
                    leapInputAnglesLeft[count] = IKAngles_left[count] + 3.14
            if(isRightOn):
                for count in range(4):
                    leapInputAnglesRight[count] = IKAngles_right[count] + 3.14
    ##end of FK

        leftErgoData[1] = leftErgoData[1] * -1.4
        leftErgoData[5] = leftErgoData[5] * -1
        rightErgoData[1] = rightErgoData[1] * -1.4
        rightErgoData[5] = rightErgoData[5] * -1
        # leappybulletik.setMotorControl(IKAngles_left[:4] + tuple(leftErgoData), IKAngles_right[0:4] + tuple(rightErgoData))
        leappybulletik.setMotorControl(None,IKAngles_right[0:4] + tuple(rightErgoData))

        if(enableLeap == True):
            if(isLeftOn):
                leap_hand_left.set_leap(leapInputAnglesLeft)
            if(isRightOn):
                leap_hand_right.set_leap(leapInputAnglesRight)
        time.sleep(.015)

    p.disconnect()

if __name__ == "__main__":
    main()

