"""windhsield_trajectory controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import math
import pandas as pd
import os
import sys
import numpy as np
from controller import Motor, Brake, PositionSensor

csv_path = '../../joints_moveit_smooth.csv'
# csv_path = '../../jointpath_FAIK.csv'
df = pd.read_csv(csv_path)

TIME_STEP = 32
DELTA = 0.05

class Joints:
    JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER = range(8)

# PI constants
p4 = 0.7853981633974483
p2 = 1.5707963267948966

# sensor value global list
sensor_values = [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
motor_set_values = [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]

# export webots path
joint_path = []

def motorToRange(motorPosition, i):
    if (i == 0):
        motorPosition = np.clip(motorPosition, -2.8972, 2.8972)
    elif (i == 1):
        motorPosition = np.clip(motorPosition, -1.7628, 1.7628)
    elif (i == 2):
        motorPosition = np.clip(motorPosition, -2.8972, 2.8972)
    elif (i == 3):
        motorPosition = np.clip(motorPosition, -3.0718, -0.0698)
    elif (i == 4):
        motorPosition = np.clip(motorPosition, -2.8972, 2.8972)
    elif (i == 5):
        motorPosition = np.clip(motorPosition, -0.0175, 3.7525)
    elif (i == 6):
        motorPosition = np.clip(motorPosition, -2.8972, 2.8972)
    else:
        pass
    return motorPosition


def readSensors(List):
    for s in range(7):
        value = List[s].getValue()
        sensor_values[s] = value
    print(f'sensor values: {sensor_values}')


def actuateMotors(row):
    vocab = [Joints.JOINT1, Joints.JOINT2, Joints.JOINT3, Joints.JOINT4, Joints.JOINT5, Joints.JOINT6, Joints.JOINT7]
    webots_q = []
    for k in range(7):

        # create prefix
        if 'moveit' in csv_path:
            motor_name = 'q' + str(k)
        else:
            motor_name = 'panda_joint' + str(k + 1)

        # set value
        value = row[motor_name]

        # if joint 4, substract p2
        if k == 4:
            # motorPosition = motorToRange(value, k)
            motorPosition = motorToRange(value - p2, k)
        else:
            motorPosition = motorToRange(value, k)

        # set position
        motors[vocab[k]].setPosition(motorPosition)
        motor_set_values[k] = motorPosition
        webots_q.append(motorPosition)
    joint_path.append(webots_q)
    # print(f'Set new position motor {motor_set_values}.')


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

    # position Sensor
    positionSensorList = []
    for i in range(7):
        psName = 'panda_joint' + str(i + 1) + '_sensor'
        ps = PositionSensor(psName)
        ps.enable(TIME_STEP)
        positionSensorList.append(ps)

    # create the robot node for reading EE position
    ee_node = supervisor.getFromDef('GRIPPER')
    
    # trace node
    trace_node = supervisor.getFromDef('TRACEGROUP')

    # children of root node
    trace_children_field = trace_node.getField('children')
    


    # Run through CSV Waypoints
    for i, row in df.iterrows():
        
        # panda joints, set new positions, read sensors
        actuateMotors(row)

        # simulation wait time
        destination_reached = False
        while (robot.step(TIME_STEP) != -1) and (not destination_reached):
            counter = 0
            readSensors(positionSensorList)
            for k in range(7):
                if motor_set_values[k]-DELTA <= sensor_values[k] <= motor_set_values[k]+DELTA:
                    counter+=1
                    # print(f'{k}th in limit and counter {counter}')
            if counter == 7:
                destination_reached = True

        # Reading GRIPPER position and orientation
        ee_sim_position = ee_node.getPosition()
        ee_sim_orientation = ee_node.getOrientation()

        # Printing GRIPPER position and orientation to console
        print(f'sim_position {ee_sim_position}')
        #print(f'sim_orientation {ee_sim_orientation}')  

        # create markers
        z_vector = ee_sim_orientation[6:9]
        length_of_panda_hand = 0.107
        true_ee_position = [ee_sim_position[i] + z_vector[i] * length_of_panda_hand for i in range(3)]

        if i % 2 == 0:
            # Displaying spheres at each ee waypoint
            trace_children_field.importMFNodeFromString(-1,
                                                   'Pose { children [ Shape {'
                                                   'geometry Sphere { radius 0.005 subdivision 3 } } ] }')

            sphere_node = trace_children_field.getMFNode(-1)
            translation_field = sphere_node.getField('translation')
            translation_field.setSFVec3f(true_ee_position)

    # uncomment to delete markers at end of simulation
    # while trace_children_field.getCount() > 0:
    #     trace_children_field.removeMF(-1)

    # uncomment to save joint_path in csv
    # joint_path_webots_df = pd.DataFrame(joint_path, columns=["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"])
    # joint_path_webots_df.to_csv("target/jointpath_webots.csv", index=False)

    # After goint throught all waypoints end the controller
    sys.exit(0)
        
    
