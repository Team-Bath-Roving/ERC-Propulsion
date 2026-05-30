import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry

import time
import numpy as np


########################## TelepresenceOperations ###########################

class TelepresenceOperations(Node):
    def __init__(self):
        super().__init__("teleop")

        self.declare_parameter("speed", 1.2) # float (turns/s) // Parameter not directly used
        self.declare_parameter("ramp_rate", 1.0) # float
        self.declare_parameter("wheel_seperation", 0.4) # float
        # 8cm from measurement, 1cm uncertainty
        self.declare_parameter("wheel_radius", 0.08) # float
        self.declare_parameter("wheel_radius_uncertainty", 0.01)

        self.declare_parameter("alpha_angles", [np.pi/4, 3/4*np.pi, 5/4 * np.pi, 7/4 * np.pi])
        self.declare_parameter("l_distances", [1, 1, 1, 1])

        """
        PYRIGHT COMPLAINS: It seems function description is written incorrectly in the source. 
        self.declare_parameters(
                namespace="", 
                parameters=[("speed", 1.0), # float
                            ("ramp_rate", 1.0),  ("wheel_seperation", 0.4)]
            )
        """

        # Set Dimensions for Kinematics
        self.alphas = np.array(self.get_parameter("alpha_angles").value)
        self.ls = np.array(self.get_parameter("l_distances").value)

        self.kinematic_matrix = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ])

        self.rotation_angles_left = [np.pi/4, 7/4 * np.pi, 7/4 * np.pi, np.pi/4]
        self.rotation_angles_right = [5/4 * np.pi, 3/4 * np.pi, 3/4 * np.pi, 5/4 * np.pi] 


        node_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Subscriptions
        self.cmdvel_ = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_set_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )
        self.wheel_angles_ = self.create_subscription(
            Float32MultiArray,
            "/current_angles",
            self.wheel_angles_set_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )
        self.wheel_velocities = self.create_subscription(
            Float32MultiArray,
            "/wheel_velocities",
            self.wheel_velocities_set_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )

        # Publishers
        self.encoder_odom_pub_ = self.create_publisher(
                Odometry, "/propulsion/odom", qos_profile=qos_profile_sensor_data
        )
        self.steering_angles_pub_ = self.create_publisher(
                Float32MultiArray, "/target_angles", qos_profile=qos_profile_sensor_data
        )
        self.drive_velocities_pub_ = self.create_publisher(
                Float32MultiArray, "/target_wheel_velocities", qos_profile=qos_profile_sensor_data
        )

        # State -
        self.target = Twist()
        self.wheel_velocities = np.array([0.0, 0.0, 0.0, 0.0])

        self.stationary = False

        # Connection timer
        self.last_connection_ = time.monotonic()
        self.connection_timer_ = self.create_timer(0.5, self.shutdownCB_, node_cb_group)
        self.odom_timer_ = self.create_timer(0.05, self.odomCB_, node_cb_group)
        self.driver_timer_ = self.create_timer(0.02, self.driveCB_, node_cb_group)


########################### TeleOp Functions ###########################
    
    def shutdownCB_(self):
        if time.monotonic() > self.last_connection_ + 1.2:
            self.get_logger().warn("Lost connection, setting movement to zero.")
            self.target.linear.x = 0.0
            self.target.linear.y = 0.0
            self.target.angular.z = 0.0
            self.cmd_set_(self.target)

    
    def cmd_set_(self, msg: Twist):
        self.last_connection_ = time.monotonic()
        
        self.target.linear.x = msg.linear.x
        self.target.linear.y = msg.linear.y
        self.target.angular.z = msg.angular.z  

        self.steer()


    def steer(self):
        linear_angle = np.arctan2(self.target.linear.y, self.target.linear.x)
        linear_array = np.empty(4)
        linear_array.fill(linear_angle)

        linear_mag = np.sqrt(self.target.linear.x ** 2 + self.target.linear.y ** 2)
       
        # 0.5 weighting for the rotation velocity, to favour the directional 
        # cmdvel
        lin_ratio = linear_mag / (abs( 0.5 * self.target.angular.z) + linear_mag)
        
        if self.target.angular.z > 0:
            target_angles = lin_ratio * linear_array + (1 - lin_ratio) * self.rotation_angles_left
        else:
            target_angles = lin_ratio * linear_array + (1 - lin_ratio) * self.rotation_angles_right

        ang_msg = Float32MultiArray()
        ang_msg.data = target_angles.tolist()

        self.steering_angles_pub_.publish(ang_msg)

    def drive(self):
        kinematic_matrix = self.setup_kin_mat()

        kin_inverse = np.linalg.pinv(kinematic_matrix)

        control_vector = np.array([self.target.linear.x, self.target.linear.y, self.target.angular.z])

        target_wheel_velocities = kin_inverse @ control_vector

        msg = Float32MultiArray()
        msg.data = target_wheel_velocities.tolist()

        self.drive_velocities_pub_.publish(msg)

    def driveCB_(self):
        self.drive()
    
    def wheel_angles_set_(self, msg: Float32MultiArray):
        self.current_angles = np.array(list(msg.data))

    def wheel_velocities_set_(self, msg: Float32MultiArray):
        self.wheel_velocities = np.array(list(msg.data))
    
    def setup_kin_mat(self):
        mat = np.array(
            [
                [np.cos(self.current_angles[0]), np.cos(self.current_angles[1]), 
                 np.cos(self.current_angles[2]), np.cos(self.current_angles[3])],
                
                [np.sin(self.current_angles[0]), np.sin(self.current_angles[1]), 
                 np.sin(self.current_angles[2]), np.sin(self.current_angles[3])],
                
                [np.sin(self.current_angles[0] - self.alphas[0])/self.ls[0], 
                 np.sin(self.current_angles[1] - self.alphas[1])/self.ls[1], 
                 np.sin(self.current_angles[2] - self.alphas[2])/self.ls[2], 
                 np.sin(self.current_angles[3] - self.alphas[3])/self.ls[3]]
            ]
        )
        return mat

########################### OdomCB and Covariance ###########################

    def odomCB_(self):

        state_vector = self.kinematic_matrix @ self.wheel_velocities

        odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="odom_frame",
            ),
            child_frame_id="base_frame",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=float(state_vector[0]),
                        y=float(state_vector[1]),
                        z=float(0),
                    ),
                    angular=Vector3(
                        x=float(0),
                        y=float(0),
                        z=float(state_vector[2])
                    )
                ),
            ),
        )
    
        self.encoder_odom_pub_.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    tele = TelepresenceOperations()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tele)
    try:
        executor.spin()
    except KeyboardInterrupt:
        tele.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        tele.destroy_node()
        rclpy.utilities.try_shutdown()
