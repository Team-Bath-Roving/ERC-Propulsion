import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import odrive
from odrive.enums import AxisState, InputMode

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from propulsion_ros.config.serial import drives

import time
from typing import List
import numpy as np


########################### DriveMapping ###########################

class DriveMapping:
    def __init__(
            self, 
            serial: str, 
            index: int, 
            polarity: int, 
            identifier: str,
            wheel_radius: float, 
            wheel_radius_uncertainty: float
            ):

        self.serial = serial
        self.index = index
        self.polarity = int(polarity)
        self.drive = None
        self.wheel_radius = wheel_radius
        self.wheel_radius_uncertainty = wheel_radius_uncertainty
        self.identifier = identifier

    def apply_speed(self, speeds):
        if self.drive is None:
            return

        v = speeds[self.index] / (2 * np.pi * self.wheel_radius)
        
        try:
            self.drive.axis0.controller.input_vel = v
        except Exception as e:
            print(f"Failed to set velocity on {self.serial}: {e}")
    
    @property
    def speed(self):
        if self.drive is None:
            return
        # convert from turn/s to rads/s, then multiply by wheel_radius to return in m/s
        return float(2 * np.pi * self.drive.axis0.encoder.vel_estimate * self.polarity * self.wheel_radius) 
    
    @property
    def uncertainty(self):
        if self.drive is None:
            return 
        return float(2 * np.pi * self.drive.axis0.encoder.vel_estimate * self.wheel_radius_uncertainty) ** 2 # pyright: ignore

########################### TelepresenceOperations ###########################

class Odrive(Node):
    def __init__(self):
        super().__init__("odrive")

        # self.declare_parameter("speed", 1.2) # float (turns/s) // Parameter not directly used
        self.declare_parameter("ramp_rate", 1.0) # float
        # self.declare_parameter("wheel_seperation", 0.4) # float
        # # 8cm from measurement, 1cm uncertainty
        self.declare_parameter("wheel_radius", 0.08) # float
        self.declare_parameter("wheel_radius_uncertainty", 0.01)

        # [front_left, front_right, back_right, back_left]
        self.wheel_velocities_ = np.array([0.0, 0.0, 0.0, 0.0])

        """
        PYRIGHT COMPLAINS: It seems function description is written incorrectly in the source. 
        self.declare_parameters(
                namespace="", 
                parameters=[("speed", 1.0), # float
                            ("ramp_rate", 1.0),  ("wheel_seperation", 0.4)]
            )
        """

        # Set Wheel seperation for Odometry
        # self.wheel_seperation_ = self.get_parameter("wheel_seperation").value

        # Set wheel radius for calculations
        # self.wheel_radius_ = self.get_parameter("wheel_radius").value

        self.mappings = []
        for e in drives:
            serial = e['serial']
            index = e['index']
            polarity = e['polarity']
            idenitfier = e['identifier']
            self.mappings.append(
                    DriveMapping(
                        serial,
                        index, 
                        polarity,
                        idenitfier,
                        self.get_parameter("wheel_radius").value, # pyright: ignore
                        self.get_parameter("wheel_radius_uncertainty").value, # pyright: ignore
                        )
                    ) 

        self.find_drives(self.mappings)
    
        # Set Acceleration
        for d in self.mappings:
            d.drive.axis0.controller.config.input_mode = InputMode.VEL_RAMP
            d.drive.axis0.controller.config.vel_ramp_rate = self.get_parameter("ramp_rate").value

        node_cb_group = MutuallyExclusiveCallbackGroup()

        # Topics
        self.wheel_velocities_sub_ = self.create_subscription(
            Float32MultiArray,
            "/target_wheel_velocities",
            self.velocities_set_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )

        # Publishers
        self.encoder_wheel_pub_ = self.create_publisher(
                Float32MultiArray, "/wheel_velocities", qos_profile=qos_profile_sensor_data
        )

         # State -
        self.stationary = False

        # Connection timer
        self.last_connection_ = time.monotonic()
        self.connection_timer_ = self.create_timer(0.5, self.shutdownCB_, node_cb_group)
        self.driver_timer_ = self.create_timer(0.02, self.drive, node_cb_group)
        self.encoder_timer_ = self.create_timer(0.02, self.encoderCB_, node_cb_group)

########################### TeleOp Functions ###########################
    
    def shutdownCB_(self):
        # 0.2 delay shutdown time from not recieving drive commands
        if time.monotonic() > self.last_connection_ + 0.2:
            self.get_logger().warn("Lost connection, setting movement to zero.")
            self.wheel_velocities_ = np.array([0.0, 0.0, 0.0, 0.0])
            self.drive()

    def encoderCB_(self):
        # 1. Define your 2D float data (using NumPy for convenience)
        matrix_2d = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ], dtype=np.float32)

        for m in self.mappings:
            matrix_2d[0][m.index] = m.speed
            matrix_2d[1][m.index] = m.uncertainty


        # 2. Get the dimensions
        rows, cols = matrix_2d.shape

        # 3. Instantiate the ROS2 message
        msg = Float32MultiArray()

        # 4. Set up the layout metadata for Dimension 0 (Rows)
        dim_rows = MultiArrayDimension()
        dim_rows.label = "rows"
        dim_rows.size = rows
        dim_rows.stride = rows * cols # Total elements in the matrix
        
        # 5. Set up the layout metadata for Dimension 1 (Columns)
        dim_cols = MultiArrayDimension()
        dim_cols.label = "columns"
        dim_cols.size = cols
        dim_cols.stride = cols        # Elements in a single row

        # 6. Assign the layout description to the message
        msg.layout.dim = [dim_rows, dim_cols]

        # 7. Flatten your 2D data into a 1D Python list and assign it
        msg.data = matrix_2d.flatten().tolist()

        self.encoder_wheel_pub_.publish(msg)
    
    def velocities_set_(self, msg: Float32MultiArray):
        self.last_connection_ = time.monotonic()
        self.wheel_velocities_ = np.array(list(msg.data))
    
    def drive(self):
        # divide to convert to turns/s
        for m in self.mappings:
            m.apply_speed(self.wheel_velocities_) #pyright: ignore

    @staticmethod
    def find_drives(mappings: List[DriveMapping]):
        if odrive is None:
            print("odrive library not available; running in dry-run mode.")
            return

        for m in mappings:
            try:
                d = odrive.find_any(serial_number=m.serial)
            except Exception as e:
                print(f"Error finding drive {m.serial}: {e}")
                d = None
            if d is None:
                print(f"Warning: could not find drive with serial '{m.serial}'")
            else:
                m.drive = d
                # try to ensure axis0 is in closed loop
                try:
                    d.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                except Exception:
                    pass
    

def main(args=None):
    rclpy.init(args=args)

    odri = Odrive()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odri)
    try:
        executor.spin()
    except KeyboardInterrupt:
        odri.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        odri.destroy_node()
        rclpy.utilities.try_shutdown()
