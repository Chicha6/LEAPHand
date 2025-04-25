import numpy as np
import zmq

from leap_main import LeapNode
import time


def main(**kwargs):
    leap_hand = LeapNode()

    context = zmq.Context()
    #Socket to talk to Manus SDK

    print("Connecting to SDK")
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, True)     
    socket.connect("tcp://127.0.0.1:9000")

    while True:
        message = socket.recv()
        message = message.decode('utf-8')
        right = message.split(",")   
        if len(right) == 12:
            rightData = list(map(float,right[0:12]))

        # Adjustments for index and middle joints
        # +180 because zero position (open pose) for LEAP is 180 degrees (MANUS origin is 0 degrees)
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
        rightData = [60 - .8*rightData[1] + 180] + [-30 + 50 + rightData[0] + 180] + [2.5*rightData[2] + 180] + [rightData[3] + 180] + [rightData[5], rightData[4]] + rightData[6:8] + [rightData[9], rightData[8]] + rightData[10:12]
        # Current joint angle order: thumb, index, middle
        
        # Convert to radian and set LEAP position
        leap_hand.set_leap(np.deg2rad(rightData))
        time.sleep(.03)


if __name__ == "__main__":
    main()
