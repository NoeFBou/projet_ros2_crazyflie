import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import py_trees
from py_trees.common import Status
import time


class WaitGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name ="WaitGoal", topic_name="/target_pose"):
        super(WaitGoal, self).__init__(name)
        self.name = name
        self.topic_name = topic_name
        self.node = None
        self.subscriber = None
        self.new_goal_received = False
        self.goal_data = None

        self.blackboard = py_trees.blackboard.Client(name=name, namespace=name)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        self.subscriber = self.node.create_subscription(
            PointStamped,
            self.topic_name,
            self._callback,
            10
        )
        return True

    def _callback(self, msg):
        self.node.get_logger().info(f"new target: {msg.point}")
        self.goal_data = msg
        self.new_goal_received = True

    def initialise(self):
        pass

    def update(self):
        if self.new_goal_received and self.goal_data:

            target = [self.goal_data.point.x, self.goal_data.point.y, self.goal_data.point.z]
            self.blackboard.target_pose = target

            self.new_goal_received = False
            self.node.get_logger().info(f"status blackboard: {self.blackboard.target_pose}")
            self.node.get_logger().info(f"status new goal rec: {self.new_goal_received}")
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        pass

class TrajectoriesPlanner(py_trees.behaviour.Behaviour):
    def __init__(self, name="Plan Path"):
        super(TrajectoriesPlanner, self).__init__(name)

        self.blackboard = py_trees.blackboard.Client(name=name, namespace="Mission")
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="trajectory", access=py_trees.common.Access.WRITE)

        self.node = None
        self.goal_pub = None
        self.traj_sub = None
        self.received_traj = None
        self.published = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)

        self.traj_sub = self.node.create_subscription(
            MultiDOFJointTrajectory,
            '/planned_trajectory_final',
            self._traj_callback,
            10
        )
        return True

    def _traj_callback(self, msg):
        self.received_traj = msg

    def initialise(self):
        self.received_traj = None
        self.published = False

    def update(self):
        if self.blackboard.target_pose is None:
            return Status.FAILURE

        if not self.published:
            target = self.blackboard.target_pose

            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = target[0]
            msg.pose.position.y = target[1]
            msg.pose.position.z = target[2]
            msg.pose.orientation.w = 1.0

            self.goal_pub.publish(msg)
            self.node.get_logger().info(f"New Goal")

            self.published = True
            return Status.RUNNING

        if self.received_traj:
            self.blackboard.trajectory = self.received_traj
            return Status.SUCCESS

        return Status.RUNNING