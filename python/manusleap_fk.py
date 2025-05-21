import numpy as np
import zmq
import time
from leap_main import LeapNode

"""
This script takes joint angles from MANUS SDK and communicates them to the robot hand motors, which is essentially forward kinematics.
The thumb joint angles value are adjusted during the teleoperation phase to account for the difference in the human hand anatomy and that of the robot.

"""

# Set U2D2 COM port here
leftCOM = "COM5"
rightCOM = "COM6"

def main(**kwargs):
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]
    leap_hand_left = None
    leap_hand_right = None

    # Set up ZMQ to talk to Manus SDK
    context = zmq.Context()
    print("Connecting to SDK...")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:9000") #Set IP and port to receive data from MANUS SDK
    print("Connected to SDK, Waiting for data ...")


    enableLeap = True # Set to false if not using robot hand

    # Checking if message is being sent from MANUS SDK
    while True:
        try:
            message = socket.recv(flags=zmq.NOBLOCK)
            message = message.decode('utf-8')
            manusData = message.split(",")   
            if(len(manusData) == 26):
                print("Correct data received.")
                break 
            else:
                print("Wrong data received! Make sure to select ergonomic data only in MANUS SDK.")
                time.sleep(0.1)
        except zmq.error.Again:
            print("No message received yet.")
        time.sleep(0.1)

    leftGloveID = manusData[0]
    rightGloveID = manusData[13]
    
    # If glove connected, initialise robot hand
    if(enableLeap):
        if(leftGloveID != "0"):
            leap_hand_left = LeapNode(leftCOM, motor_ids_left)
        if(rightGloveID != "0"):
            leap_hand_right = LeapNode(rightCOM, motor_ids_right)

    # Start of teleoperation
    while True:
        message = socket.recv()
        message = message.decode('utf-8')
        manusData = message.split(",")   
        leftData = list(map(float,manusData[1:13]))
        rightData = list(map(float,manusData[14:26]))

        # Adjustments for index and middle joints
        # +180 because zero position (open pose) for LEAP is 180 degrees (MANUS origin is 0 degrees)
        if(leftData):
            for n in range(4,12):
                if (n == 5):
                    # Correcting polarity and amplifying splay
                    leftData[n] = (leftData[n] + 4) * -1.7 + 180
                elif (n == 9):
                    # Correcting polarity and amplifying splay
                    leftData[n] = (leftData[n] + 9) * -1.5 + 180
                else:
                    leftData[n] = leftData[n] + 180
            # Adjusting thumb joints angles
            leftData = [-30 + 50 + leftData[0] + 180] + [60 - 0.8*leftData[1] + 180] +[2.5*leftData[2] + 180] + [leftData[3] + 180] + leftData[4:12]
        if(rightData):
            for n in range(4,12):
                if (n == 5):
                    # Correcting polarity and amplifying splay
                    rightData[n] = (rightData[n] + 4) * -1.7 + 180
                elif (n == 9):
                    # Correcting polarity and amplifying splay
                    rightData[n] = (rightData[n] + 9) * -1.5 + 180
                else:
                    rightData[n] = rightData[n] + 180
            # Adjusting thumb joint angles
            rightData = [-30 + 50 + rightData[0] + 180] + [60 - 0.8*rightData[1] + 180] + [2.5*rightData[2] + 180] + [rightData[3] + 180] + rightData[4:12]
        
        
        # Convert to radian and set LEAP position
        if(enableLeap):
            if(leap_hand_left):
                leap_hand_left.set_leap(np.deg2rad(leftData))
            if(leap_hand_right):
                leap_hand_right.set_leap(np.deg2rad(rightData))

        time.sleep(.03)


if __name__ == "__main__":
    main()
