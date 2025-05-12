import numpy as np
import zmq

from leap_main import LeapNode
import time


def main(**kwargs):
    motor_ids_left = [0,1,2,3,4,5,6,7,8,9,10,11]
    motor_ids_right = [0,1,2,3,4,5,6,7,8,9,10,11]
    leap_hand_left = None
    leap_hand_right = None
    context = zmq.Context()
    #Socket to talk to Manus SDK

    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:9000")
    message = socket.recv()
    message = message.decode('utf-8')
    manusData = message.split(",")   
    leftGloveID = manusData[0]
    rightGloveID = manusData[13]
    
    if(leftGloveID != "0"):
        leap_hand_left = LeapNode("COM5", motor_ids_left)
    if(rightGloveID != "0"):
        leap_hand_right = LeapNode("COM6", motor_ids_right)

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
                if (n == 4):
                    # Correcting polarity and amplifying splay
                    leftData[n] = (leftData[n] + 4) * -1.7 + 180
                elif (n == 8):
                    # Correcting polarity and amplifying splay
                    leftData[n] = (leftData[n] + 9) * -1.5 + 180
                elif (n == 5 or n == 9):
                    # Increase MCP joint flexion
                    leftData[n] = leftData[n] + 180
                else:
                    leftData[n] = leftData[n] + 180
            # Adjusting thumb joints and reordering joint angles in rightData
            leftData = [60 - 0.8*leftData[1] + 180] + [-30 + 50 + leftData[0] + 180] + [2.5*leftData[2] + 180] + [leftData[3] + 180] + [leftData[5], leftData[4]] + leftData[6:8] + [leftData[9], leftData[8]] + leftData[10:12]
        if(rightData):
            for n in range(4,12):
                if (n == 4):
                    # Correcting polarity and amplifying splay
                    rightData[n] = (rightData[n] + 4) * -1.7 + 180
                elif (n == 8):
                    # Correcting polarity and amplifying splay
                    rightData[n] = (rightData[n] + 9) * -1.5 + 180
                elif (n == 5 or n == 9):
                    # Increase MCP joint flexion
                    rightData[n] = rightData[n] + 180
                else:
                    rightData[n] = rightData[n] + 180
            # Adjusting thumb joints and reordering joint angles in rightData
            rightData = [60 - 0.8*rightData[1] + 180] + [-30 + 50 + rightData[0] + 180] + [2.5*rightData[2] + 180] + [rightData[3] + 180] + [rightData[5], rightData[4]] + rightData[6:8] + [rightData[9], rightData[8]] + rightData[10:12]
        
        
        # Convert to radian and set LEAP position
        if(leap_hand_left):
            leap_hand_left.set_leap(np.deg2rad(leftData))
        if(leap_hand_right):
            leap_hand_right.set_leap(np.deg2rad(rightData))

        time.sleep(.03)


if __name__ == "__main__":
    main()
