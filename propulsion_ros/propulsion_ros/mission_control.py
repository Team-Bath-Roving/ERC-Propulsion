import rclpy
from rclpy.node import Node
import rclpy.executors
import rclpy.utilities
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

from propulsion_ros.config.mappings import AXES


########################### MissionControl ###########################

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        # Scale factor to convert stick (-1...1) to m/s and rads/s
        self.declare_parameter("speed_max", 0.4)
        self.declare_parameter("angular_speed_max", 1.0)

        self.speed_max = self.get_parameter("speed_max").value # m/s
        self.angular_speed_max = self.get_parameter("angular_speed_max").value # rads/s

        # Subscriptions 
        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data, # Mutually exclusive callback vs Multithreaded executors?
        )
        # Publishers
        self.pubtwist = self.create_publisher(
                Twist, "/cmd_vel", qos_profile=qos_profile_sensor_data
        )
    
############################# Functions #############################
  
    def teleopCB_(self, msg: Joy): 
        # joystick is inverted from what you would expect
        rotation = msg.axes[AXES["TRIGGERLEFT"]] 
        rotation -= msg.axes[AXES["TRIGGERRIGHT"]] 
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        rotation *= self.angular_speed_max/2 # pyright: ignore

        pubtwist_msg = Twist(
                    linear=Vector3(
                        x=msg.axes[AXES["LEFTY"]] * self.speed_max, # pyright: ignore
                        y=msg.axes[AXES["LEFTX"]] * self.speed_max, # pyright: ignore
                        z=float(0),
                    ),
                    angular=Vector3(
                        x=float(0),
                        y=float(0),
                        z=rotation
                    )
        )
        self.pubtwist.publish(pubtwist_msg)

############################# Main #############################

def main(args=None):
    rclpy.init(args=args)

    mission = MissionControl()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mission)
    try:
        executor.spin()
    except KeyboardInterrupt:
        mission.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        mission.destroy_node()
        rclpy.utilities.try_shutdown()
