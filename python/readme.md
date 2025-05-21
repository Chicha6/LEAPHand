# Modified Python SDK For 3-Fingered LEAP Hand 

## Hardware
[For Assembly](https://drive.google.com/drive/folders/11GmIZTVYdWAc8Pl7cqzY3sA_pOkUi0e-?usp=sharing)

## Set Up Guide
1. Clone the repo
2. This repo was tested on Python 3.13 only, and may not work on other versions. Do check the version on your device to avoid compatability issues.
3. Install the following additional python packages
    - [pybullet](https://pypi.org/project/pybullet/)
    - [dynamixel sdk](https://pypi.org/project/dynamixel-sdk/)
    - numpy
    - [pyzmq](https://pypi.org/project/pyzmq/)
## Modes of robot hand control
  1. IK mode (inverse kinematics)
     - To use raw skeleton data (joint positions) from Manus SDK
  2. FK mode (foward kinematics)
     - To use ergonomic data (joint angles) from Manus SDK
  3. Hybrid (both inverse and forward kinematics)
     - To use combined data (joint positions and angles) from Manus SDK

## IK Mode
This script takes the Manus glove joint position data, runs the IK solver from Pybullet, then outputs the joint angles for the robot hand. The IK calculation takes fingertip and the last joint (distal interphalageal joint) on every finger as the end effectors.

Scaling can be done manually or automatically, which is important for accurate finger retargetting. More details in  manusleap_ik.py

### **Advantage**
More accurate fingertip level control compared to other modes, able to replicate pinching using fingertips better. This is more evident in the thumb, where the FK mode struggles with its fingertip position and orientation

### **Dsiadvantage**
Less accurate joint level control compared to other modes, can handle slight finger curling but struggles with extensive finger flexion.

## FK Mode
This script takes the Manus glove joint angle data and directly feed them into the robot hand. Manual adjustments could be made to tune the retargetting. More details in manusleap_fk.py

### **Advantage**
More accurate joint level control compared to other modes, able to replicate finger curling better.
### **Disadvantage**
Less accurate fingertip level control, struggles with thumb position and orientation.

## Hybrid Mode
This script attempts to combine the benefits of both IK and FK. The thumb is controled using IK while the index and the middles fingers are directly controlled using FK. 
### **Advantage**
Index and middle fingers are able to curl more accurately like in FK and thumb pose can be replicated better like in IK. 
### **Dsiadvantage**
Due to having 2 different kinematic solutions on the same robot hand, accurracy in finger to finger position is reduced. (less accurate pinching)


## How To Use
1. Launch MANUS Core
2. Calibrate gloves in MANUS Core (if have not done so)
3. Check the glove ids in MANUS Core
4. Launch MANUS SDK
5. Connect robot hand, check which COM ports they are using
6. Set the glove id and COM ports in then python scripts
7. Select one of 3 data streaming modes in MANUS SDK based on your preferred robot hand control mode
8. Run your preferred mode of control (IK/FK/Hybrid)
9. Teleoperate
