import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations

class TrajectoriesFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.declare_parameter("traj_topic", "/planned_trajectory")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_frame", "crazyflie/base_footprint")
        # self.declare_parameter("kp_xy", 1.5)
        # self.declare_parameter("kp_z", 1.5)
        # self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("frequence", 0.02) # 0.02 = 50Hz

        self.trajectory = None
        self.start_time = None
        self.tracking_active = False
        self.index_lst_point=0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_traj = self.create_subscription(
            MultiDOFJointTrajectory,
            self.get_parameter("traj_topic").value,
            self.traj_callback,
            10
        )
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def traj_callback(self, msg: MultiDOFJointTrajectory):
        self.get_logger().info(f"Received trajectory")
        self.trajectory = msg
        self.start_time = self.get_clock().now()
        self.tracking_active = True
        self.index_lst_point = 0

    def control_loop(self):
        if (self.tracking_active == False):
            return


def main():
    rclpy.init()
    node = TrajectoriesFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()