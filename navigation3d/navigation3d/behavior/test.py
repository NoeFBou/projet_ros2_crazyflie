import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException

class TakeOff(py_trees.behaviour.Behaviour):
    def __init__(self, name="Take Off", height=1.0, threshold=0.1): #10cm
        super(TakeOff, self).__init__(name)
        self.target_height = height #m
        self.threshold = threshold

        self.node = None
        self.publisher = None
        self.tf_buffer = None
        self.tf_listener = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        return True

    def initialise(self):
        self.node.get_logger().info(f"Décollage")

    def update(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "crazyflie/base_footprint",
                rclpy.time.Time()
            )
            current_z = t.transform.translation.z
        except LookupException:
            return Status.RUNNING

        error = self.target_height - current_z

        if abs(error) < self.threshold:
            self.publisher.publish(Twist())
            self.node.get_logger().info("Fin Décollage")

            return Status.SUCCESS


        kp = 1.0
        vel_z = kp * error

        vel_z = max(min(vel_z, 0.5), -0.5)

        if vel_z > 0 and vel_z < 0.2: vel_z = 0.2

        msg = Twist()
        msg.linear.z = float(vel_z)
        self.publisher.publish(msg)

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID:
            self.publisher.publish(Twist())

