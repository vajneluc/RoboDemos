"""windhsield_trajectory controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import pandas as pd

csv_path = 'C:/Users/Julius/Documents/Webots_python/joints_SERVO_filt.csv'
df = pd.read_csv(csv_path)

TIME_STEP = 32

class Joints:
    JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER = range(8)

p4 = 0.7853981633974483
p2 = 1.5707963267948966
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    motors = [robot.getDevice(f"panda_joint{i+1}") for i in range(7)]
    '''
    q_vec = [0.0,-0.7853981633974483,0.0,-2.356194490192345,-p2,1.5707963267948966,0.7853981633974483]
    joint_names = [f"panda_joint{i+1}" for i in range(7)]
    joint_indices = list(range(7))
    for joint_name, q, i in zip(joint_names, q_vec, joint_indices):
        motors[i].setPosition(q)
    '''
    for i, row in df.iterrows():
        
        motors[Joints.JOINT1].setPosition(row['panda_joint1'])
        motors[Joints.JOINT2].setPosition(row['panda_joint2'])
        motors[Joints.JOINT3].setPosition(row['panda_joint3'])
        motors[Joints.JOINT4].setPosition(row['panda_joint4'])
        motors[Joints.JOINT5].setPosition(row['panda_joint5']-p2)
        motors[Joints.JOINT6].setPosition(row['panda_joint6'])
        motors[Joints.JOINT7].setPosition(row['panda_joint7'])
        robot.step(TIME_STEP * 5)
    