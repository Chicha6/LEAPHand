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
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]

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
    leap_hand_left = None
    leap_hand_right = None
    isCalibrated = False
    isLeftOn = False
    isRightOn = False
    leappybulletik = None
    enableLeap = True

    
    message = socketRaw.recv()
    message = message.decode('utf-8')
    manusData = message.split(",")   

    if(enableLeap == True):
        if(isLeftOn):
            leap_hand_left = LeapNode("COM5", motor_ids_left)

        if(isRightOn):    
            leap_hand_right = LeapNode("COM6", motor_ids_right)

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
                    leftTargetPosList[0][i] = leftTargetPosList[0][i] * 1.65
                    leftTargetPosList[1][i] = leftTargetPosList[1][i] * 1.65
                for n in range(2):
                    leftTargetPosList[n][0] += -0.45 
        # MANUS to LEAP scaling (right hand)
        if(isRightOn):
            #to use auto calibration values
            if(isCalibrated):
                for n in range(2):
                    for i in range(3):
                        rightTargetPosList[n][i] = rightTargetPosList[n][i] * ManusToLeapScaling_right[n//2]
            #to manually adjust scaling
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
                else:
                    leapInputAnglesRight[n] = rightErgoData[n-4] + 3.14
        
        
        # joint angle polarity and magnitude correction for LEAP hand control
        if(isLeftOn):
            for count in range(4):
                leapInputAnglesLeft[count] = IKAngles_left[count] + 3.14
        if(isRightOn):
            for count in range(4):
                leapInputAnglesRight[count] = IKAngles_right[count] + 3.14

    ##end of FK
        leftErgoData[1] = leftErgoData[1] * -1.8
        leftErgoData[5] = leftErgoData[5] * -1
        rightErgoData[1] = rightErgoData[1] * -1.8
        rightErgoData[5] = rightErgoData[5] * -1
        # leappybulletik.setMotorControl(IKAngles_left[:4] + tuple(leftErgoData), IKAngles_right[0:4] + tuple(rightErgoData))
        leappybulletik.setMotorControl(IKAngles_left[0:4] + tuple(leftErgoData), None)

        if(enableLeap == True):
            if(isLeftOn):
                leap_hand_left.set_leap(leapInputAnglesLeft)
            if(isRightOn):
                leap_hand_right.set_leap(leapInputAnglesRight)
        time.sleep(.015)

    p.disconnect()

if __name__ == "__main__":
    main()

