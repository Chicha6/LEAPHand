import time
import zmq
import time
import sys
import numpy as np
from leap_main import LeapNode
from manusleap_ik import LeapPybulletIK, pybulletCalibration, p

leftHandID = "558097a3"
rightHandID = "e13e29f2"

def main(**kwargs):
    
    leap_hand = LeapNode()

    #set up zmq
    context = zmq.Context()
    socketRaw = context.socket(zmq.PULL)
    socketRaw.setsockopt(zmq.CONFLATE, True)     
    socketRaw.connect("tcp://127.0.0.1:8000")
    socketErgo = context.socket(zmq.PULL)
    socketErgo.setsockopt(zmq.CONFLATE, True)     
    socketErgo.connect("tcp://127.0.0.1:9000")

    URDFOffset_left = []
    URDFOffset_right = []
    ManusToLeapScaling_left = []
    ManusToLeapScaling_right = []
    leftTargetPosList = []
    rightTargetPosList = []

    leapInputAnglesLeft = [0,0,0,0,0,0,0,0,0,0,0,0]
    leapInputAnglesRight = [0,0,0,0,0,0,0,0,0,0,0,0]
    isCalibrated = False
    isLeftOn = False
    isRightOn = False
    leappybulletik = None

    
    message = socketRaw.recv()
    message = message.decode('utf-8')
    manusData = message.split(",")   

    # Generate calibration data (optional) and initialise pybullet
    # Usage: "python MANUS_IK.py c" or "python MANUS_IK.py C"
    if (len(sys.argv) == 2):
        if(sys.argv[1] == "c" or sys.argv[1] == "C"):
        
            # Checks to initiate whether IK for both hands or either one
            if(len(manusData) > 20):
                URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socketRaw, 0)
                URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socketRaw, 1)
                leappybulletik = LeapPybulletIK([3,4],URDFOffset_left=URDFOffset_left, URDFOffset_right=URDFOffset_right)
                isCalibrated = True
                isLeftOn, isRightOn = True, True
            else:
                #left hand
                if(manusData[0] == leftHandID):
                    URDFOffset_left, ManusToLeapScaling_left, indexXOffset_left = pybulletCalibration(socketRaw, 0)
                    leappybulletik = LeapPybulletIK([3,4],URDFOffset_left=URDFOffset_left)
                    isCalibrated = True
                    isLeftOn = True

                #right hand
                if(manusData[0] == rightHandID):
                    URDFOffset_right, ManusToLeapScaling_right, indexXOffset_right = pybulletCalibration(socketRaw, 1)
                    leappybulletik = LeapPybulletIK([3,4],URDFOffset_right=URDFOffset_right)
                    isCalibrated = True
                    isRightOn = True
    else:
        if(len(manusData) > 20):
            isLeftOn, isRightOn = True, True
        elif(manusData[0] == leftHandID):
            isLeftOn = True
        else:
            isRightOn = True
        leappybulletik = LeapPybulletIK([3,4])


    #start of teleop
    for count in range(100000):
        leftTargetPosList.clear()
        rightTargetPosList.clear()

    ##start of IK
        messageRaw = socketRaw.recv()
        messageRaw = messageRaw.decode('utf-8')
        manusRawData = messageRaw.split(",")      
    	
        #load leftTargetPosList and rightTargetPosList (if both hands in use)
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
        #load leftTargetPosList or rightTargetPosList (if one hand in use)
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
            #to use auto calibration values
            if(isCalibrated):
                for n in range(2):
                    for i in range(3):
                        leftTargetPosList[n][i] = leftTargetPosList[n][i] * ManusToLeapScaling_left[n//2]
                for n in range(2):
                    leftTargetPosList[n][0] += -0.45
            #to manually adjust scaling and offsets
            else:
                for i in range(3):
                    leftTargetPosList[0][i] = leftTargetPosList[0][i] * 1.45
                    leftTargetPosList[1][i] = leftTargetPosList[1][i] * 1.45
                for n in range(2):
                    leftTargetPosList[n][0] += -0.45 + 0.019
        # MANUS to LEAP scaling (left hand)
        if(isRightOn):
            #to use auto calibration values
            if(isCalibrated):
                for n in range(2):
                    for i in range(3):
                        rightTargetPosList[n][i] = rightTargetPosList[n][i] * ManusToLeapScaling_right[n//2]
            #to manually adjust scaling
            else:
                for i in range(3):
                    rightTargetPosList[0][i] = rightTargetPosList[0][i] * 1.45
                    rightTargetPosList[1][i] = rightTargetPosList[1][i] * 1.45
        
        
        leappybulletik.update_target_vis(leftTargetPosList, rightTargetPosList)
        IKAngles_left, IKAngles_right = leappybulletik.compute_IK(hand_pos_left=leftTargetPosList, hand_pos_right=rightTargetPosList)
    ##end of IK

    ##start of FK
        messageErgo = socketErgo.recv()
        messageErgo = messageErgo.decode('utf-8')
        manusErgoData = messageErgo.split(",")

        leftErgoData = tuple(np.deg2rad(list(map(float,manusErgoData[1:9]))))
        rightErgoData = tuple(np.deg2rad(list(map(float,manusErgoData[10:18]))))

        for n in range(4,12):
            if (n == 4):
                # Correcting polarity and amplifying splay
                leapInputAnglesRight[n] = (rightErgoData[n-4]) * 1.4 + 3.14
            elif (n == 8):
                # Correcting polarity and amplifying splay
                leapInputAnglesRight[n] = (rightErgoData[n-4]) * 1.4 + 3.14
            elif (n == 5 or n == 9):
                # Increase MCP joint flexion
                leapInputAnglesRight[n] = rightErgoData[n-4] *-1 + 3.14
            else:
                leapInputAnglesRight[n] = rightErgoData[n-4] + 3.14
        
        # Adjusting thumb joints and reordering joint angles in rightData
        # rightData = [60 - .8*rightData[1] + 180] + [-30 + 50 + rightData[0] + 180] + [2.5*rightData[2] + 180] + [rightData[3] + 180] + [rightData[5], rightData[4]] + rightData[6:8] + [rightData[9], rightData[8]] + rightData[10:12]
        
        # joint angle polarity and magnitude correction for LEAP hand control
        for count in range(4):
            leapInputAnglesRight[count] = IKAngles_right[count] * -1 + 3.14

    ##end of FK
        leappybulletik.setMotorControl(IKAngles_left[:4] + leftErgoData, IKAngles_right[0:4] + rightErgoData)
        leap_hand.set_leap(leapInputAnglesRight)

        time.sleep(.015)

    p.disconnect()

if __name__ == "__main__":
    main()

