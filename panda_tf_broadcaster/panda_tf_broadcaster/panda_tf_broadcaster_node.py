import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from sympy import Matrix, Quaternion, sin, cos
from sympy.vector import matrix_to_vector, CoordSys3D

class PandaTfBroadcaster(Node):

    def __init__(self):
        super().__init__('panda_tf_broadcaster_node')

        # FREQUENCY OF NODE
        frequency = 10
        
        # /joint_states SUBSCRIBER 
        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            frequency)
        
        # tf2_msgs/TFMessage BROADCASTER
        self.broadcaster = TransformBroadcaster(
            self)
        
        # Node started
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started!".format(self.nodeName))

    def joint_state_callback(self, msg):
        t = TransformStamped()
        t.header.frame_id = 'panda_link0'
        t.child_frame_id = 'panda_link7'
        
        self.joint_var = []
        for i in range(0,7):
            self.joint_var.append((msg.position[i]))
        
        dh_parameters = self.dh_params(self.joint_var)
        
        T_01 = self.TF_matrix(0,dh_parameters)
        T_12 = self.TF_matrix(1,dh_parameters)
        T_23 = self.TF_matrix(2,dh_parameters)
        T_34 = self.TF_matrix(3,dh_parameters)
        T_45 = self.TF_matrix(4,dh_parameters)
        T_56 = self.TF_matrix(5,dh_parameters)
        T_67 = self.TF_matrix(6,dh_parameters)

        T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67 
        
       
        T_07_rot = T_07[0:3, 0:3]
        quaternion = Quaternion.from_rotation_matrix(T_07_rot)
        submat = T_07[0:3,3:4]


        # Writing data for publishing
        t.transform.translation.x = float(submat[0])
        t.transform.translation.y = float(submat[1])
        t.transform.translation.z = float(submat[2])
        t.transform.rotation.x = float(quaternion.a)
        t.transform.rotation.y = float(quaternion.b)
        t.transform.rotation.z = float(quaternion.c)
        t.transform.rotation.w = float(quaternion.d)
        
        self.broadcaster.sendTransform(t)
    
    def dh_params(self,joint_variable):

        joint_var = joint_variable
        M_PI = math.pi

        # Create DH parameters (data given by maker franka-emika)
        self.dh = [[ 0,      0,        0.333,   joint_var[0]],
            [-M_PI/2,   0,        0,       joint_var[1]],
            [ M_PI/2,   0,        0.316,   joint_var[2]],
            [ M_PI/2,   0.0825,   0,       joint_var[3]],
            [-M_PI/2,  -0.0825,   0.384,   joint_var[4]],
            [ M_PI/2,   0,        0,       joint_var[5]],
            [ M_PI/2,   0.088,    0.107,   joint_var[6]]]
        
        return self.dh
      
    def TF_matrix(self,i,dh):
        # Define Transformation matrix based on DH params
        alpha = dh[i][0]
        a = dh[i][1]
        d = dh[i][2]
        q = dh[i][3]
        
        TF = Matrix([[cos(q),-sin(q), 0, a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [   0,  0,  0,  1]])
        return TF

def main():
    rclpy.init()
    node = PandaTfBroadcaster()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()