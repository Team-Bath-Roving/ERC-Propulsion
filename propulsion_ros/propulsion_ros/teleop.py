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
        # Model assumes all l_distances are equal,
        # but can tolerate slight deviation
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
        
        # speed threshold
        self.movement_threshold = 0.015
        # angle threshold above which drive control is suspended
        self.angle_threshold = 10.0
        
        # Assumes centre of rotation is roughly on the centre of geometry
        # (angles stored in radians)
        self.rotation_angles = np.pi/4 - self.alphas
        self.target_angles = np.array([0, 0, 0, 0])
        
        # (angles stored in degrees)
        self.max_ang = 170
        self.min_ang = -170

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
        self.driver_timer_ = self.create_timer(0.02, self.drive, node_cb_group)


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


    # UNFINISHED
    def steer(self):
        linear_mag = np.sqrt(self.target.linear.x ** 2 + self.target.linear.y ** 2)
        # arbitrary threshold for now which only re-orients wheel 
        # if there enough velocity input
        if linear_mag < self.movement_threshold and self.target.angular.z < self.movement_threshold:
            return
      
        # arctan2 returns -pi, pi
        linear_angle = np.arctan2(self.target.linear.y, self.target.linear.x)
       
        linear_array = np.empty(4)
        linear_array.fill(linear_angle)
        
        # convert arrays to degrees and wrap into the correct domain
        linear_array_degrees = self.wrap_angles_to_deg(linear_array)
        rotational_array_degrees = self.wrap_angles_to_deg(self.rotation_angles)

        # find the rotational angles which are closest to the linear target
        target_rotational_array = self.find_closest_rotation_angles(linear_array_degrees, rotational_array_degrees)
        
        # 0.5 weighting for the rotation velocity, to favour the directional velocity
        lin_ratio = linear_mag / (abs( 0.5 * self.target.angular.z) + linear_mag)
        
        target_angles = lin_ratio * linear_array_degrees + (1 - lin_ratio) * target_rotational_array
        self.target_angles = self.find_minimised_target_angles(target_angles)

        ang_msg = Float32MultiArray()
        ang_msg.data = self.target_angles.tolist()

        self.steering_angles_pub_.publish(ang_msg)


    def drive(self):
        kinematic_matrix = self.setup_kin_mat()
        
        # check_whether wheels are aligned within tolerance
        # if false then then set velocites to 0
        if not self.wheels_aligned:
            target_wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        else:
            kin_inverse = np.linalg.pinv(kinematic_matrix)

            control_vector = np.array([self.target.linear.x, self.target.linear.y, self.target.angular.z])

            target_wheel_velocities = (4 * kin_inverse @ control_vector).tolist()

        msg = Float32MultiArray()
        msg.data = target_wheel_velocities

        self.drive_velocities_pub_.publish(msg)

    
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


    # returns whether wheels are aligned within tolerance of the steering direction
    @property
    def wheels_aligned(self):
        if np.amax(np.abs(self.target_angles - self.current_angles)) > self.angle_threshold:
            return False
        else:
            return True


    # returns an equivalent compliment angle in degrees
    def find_compliment(self, angle):
        return self.wrap_ang(angle + 180.0)


    # finds the shortest of two compliment angles with respect to the linear angle
    def find_closest_rotation_angles(self, linear_array, rotational_array):
        min_rotation_angles = np.array([0, 0, 0, 0])
        
        for index, angle in enumerate(rotational_array):
            rot_ang_1 = angle
            rot_ang_2 = self.find_compliment(angle)
            if abs(rot_ang_1 - linear_array[index]) < abs(rot_ang_2 - linear_array[index]):
                min_rotation_angles[index] = rot_ang_1
            else:
                min_rotation_angles[index] = rot_ang_2

        return np.array(min_rotation_angles)


    # takes array with the same indexing as self.current_angles
    # assumes angle_array is in degrees
    def find_minimised_target_angles(self, angle_array):

        target_angles = np.array([0, 0, 0, 0])

        for index, target_ang in enumerate(angle_array):
            t_ang_1 = target_ang
            t_ang_2 = self.find_compliment(target_ang)
           
            # finds the compliment angle with that is closest
            if abs(t_ang_1 - self.current_angles[index]) < abs(t_ang_2 - self.current_angles[index]):
                target_angles[index] = t_ang_1
            else:
                target_angles[index] = t_ang_2
            
            # checks whether solution is allowed, and sets the allowed compliment if not
            if t_ang_1 < self.min_ang or t_ang_1 > self.max_ang:
                target_angles[index] = t_ang_2
            elif t_ang_2 < self.min_ang or t_ang_2 > self.max_ang:
                target_angles[index] = t_ang_1

        return np.array(target_angles)


    # clamps between [-180, 180] degrees, uses angle wrapping to find 
    # similar angle in this range.
    @staticmethod
    def wrap_ang(angle):
        min_angle = -180.0
        max_angle = 180.0
        range_size = max_angle - min_angle

        return (angle - min_angle) % range_size + min_angle

    
    # takes whole array as input
    # converts from rads -> degrees, then wraps angles between [-180, 180]
    @staticmethod
    def wrap_angles_to_deg(angles):
        # convert to degrees
        angles *= 180/np.pi
        min_ang = -180
        max_ang = 180

        range_size = max_ang - min_ang 
        return [(ang - min_ang) % range_size + min_ang for ang in list(angles)]

    

########################### OdomCB and Covariance ###########################

    def odomCB_(self):

        state_vector = self.kinematic_matrix @ self.wheel_velocities / 4

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
