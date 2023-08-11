"""windhsield_trajectory controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import math
import pandas as pd
import os
import sys

csv_path = '../../joints_moveit_smooth.csv'
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
    
    # panda motors
    motors = [robot.getDevice(f"panda_joint{i+1}") for i in range(7)]
    
    # Set Panda to default configuration
    q_vec = [0.0,-0.7853981633974483,0.0,-2.356194490192345,0.0,1.5707963267948966,0.7853981633974483]
    joint_names = [f"panda_joint{i+1}" for i in range(7)]
    joint_indices = list(range(7))
    for joint_name, q, i in zip(joint_names, q_vec, joint_indices):
        motors[i].setPosition(q)
    
    # create the robot node for reading EE position
    ee_node = supervisor.getFromDef('GRIPPER')
    

    # trace node
    trace_node = supervisor.getFromDef('TRACEGROUP')

    # children of root node
    trace_children_field = trace_node.getField('children')
    


    # Run through CSV Waypoints
    for i, row in df.iterrows():
        '''
        # panda joints
        motors[Joints.JOINT1].setPosition(row['panda_joint1'])
        motors[Joints.JOINT2].setPosition(row['panda_joint2'])
        motors[Joints.JOINT3].setPosition(row['panda_joint3'])
        motors[Joints.JOINT4].setPosition(row['panda_joint4'])
        motors[Joints.JOINT5].setPosition(row['panda_joint5']-p2) # dunno why this works, but it corrects panda robot rotation
        motors[Joints.JOINT6].setPosition(row['panda_joint6'])
        motors[Joints.JOINT7].setPosition(row['panda_joint7'])
        '''
        #q
        motors[Joints.JOINT1].setPosition(row['q0'])
        motors[Joints.JOINT2].setPosition(row['q1'])
        motors[Joints.JOINT3].setPosition(row['q2'])
        motors[Joints.JOINT4].setPosition(row['q3'])
        motors[Joints.JOINT5].setPosition(row['q4']-p2) # dunno why this works, but it corrects panda robot rotation
        motors[Joints.JOINT6].setPosition(row['q5'])
        motors[Joints.JOINT7].setPosition(row['q6'])


        # Reading GRIPPER position and orientation
        ee_sim_position = ee_node.getPosition()
        ee_sim_orientation = ee_node.getOrientation()

        # Printing GRIPPER position and orientation to console
        print(f'sim_position {ee_sim_position}')
        #print(f'sim_orientation {ee_sim_orientation}')  

        z_vector = ee_sim_orientation[6:9]
        length_of_panda_hand = 0.107
        true_ee_position = [ee_sim_position[i] + z_vector[i] * length_of_panda_hand for i in range(3)]

        if i % 10 == 0:
            # Displaying spheres at each ee waypoint
            trace_children_field.importMFNodeFromString(-1,
                                                   'Pose { children [ Shape {'
                                                   'geometry Sphere { radius 0.005 subdivision 3 } } ] }')

            sphere_node = trace_children_field.getMFNode(-1)
            translation_field = sphere_node.getField('translation')
            translation_field.setSFVec3f(true_ee_position)

            # simulation wait time
            robot.step(TIME_STEP * 5)
    
    while trace_children_field.getCount() > 0:
        trace_children_field.removeMF(-1)
        
    # After goint throught all waypoints end the controller
    sys.exit(0)
        
    