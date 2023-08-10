"""windhsield_trajectory controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import math
import pandas as pd
import os


csv_path = '../../joints_moveit.csv'
df = pd.read_csv(csv_path)

TIME_STEP = 32

class Joints:
    JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER = range(8)

# PI constants
p4 = 0.7853981633974483
p2 = 1.5707963267948966



if __name__ == "__main__":
    
    # create the Robot instance.
    robot = Robot()
    
    # create Supervisor instance
    supervisor = Supervisor()
    print('this:', dir(supervisor.supervisor))
    # panda motors
    motors = [robot.getDevice(f"panda_joint{i+1}") for i in range(7)]
    
    # Set Panda to default configuration
    q_vec = [0.0,-0.7853981633974483,0.0,-2.356194490192345,0.0,1.5707963267948966,0.7853981633974483]
    joint_names = [f"panda_joint{i+1}" for i in range(7)]
    joint_indices = list(range(7))
    for joint_name, q, i in zip(joint_names, q_vec, joint_indices):
        motors[i].setPosition(q)
    
    # create the robot node for reading EE position
    robot_node = supervisor.getFromDef('GRIPPER')
    print('not_this', dir(robot_node))
    exit()
    # create list for shape nodes
    shape_nodes = []
    '''
    # Run through CSV Waypoints
    for i, row in df.iterrows():
        if i % 10 == 0:
            motors[Joints.JOINT1].setPosition(row['q0'])
            motors[Joints.JOINT2].setPosition(row['q1'])
            motors[Joints.JOINT3].setPosition(row['q2'])
            motors[Joints.JOINT4].setPosition(row['q3'])
            motors[Joints.JOINT5].setPosition(row['q4']-p2) # dunno why this works, but it corrects panda robot rotation
            motors[Joints.JOINT6].setPosition(row['q5'])
            motors[Joints.JOINT7].setPosition(row['q6'])
            
            # comparing position of simulation to the one read from csv (works with joints_moveit.csv)
            translation_from_sim = robot_node.getPosition()
            translation_from_csv = [row['ee_pos_x'], row['ee_pos_y'], row['ee_pos_z']]
            offset = [translation_from_sim[i] - translation_from_csv[i] for i in range(3)]
            print(f'sim_position {translation_from_sim}\ncsv_position {translation_from_csv}\noffset {offset}')

            
            translation_string = f'{}'
            children_field.importMFNodeFromString(-1, f'DEF BALL{i} Ball { "translation" {translation_from_sim[0]} }')
            sphere = shape_node.addProto('Sphere')
            sphere.getField('size').setSFVec3f([0.05, 0.05, 0.05])
            shape_node.getField('translation').setSFVec3f(translation_from_sim)
            
            # simulation wait time
            robot.step(TIME_STEP * 5)
    '''