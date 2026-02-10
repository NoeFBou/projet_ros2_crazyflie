import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory
from rclpy.action import ActionClient
import py_trees
from py_trees.common import Status
import time
from navigation3d_interfaces.action import FollowTrajectory
from tf2_ros import Buffer, TransformListener


class WaitGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name ="WaitGoal", topic_name="/target_pose"):
        super(WaitGoal, self).__init__(name)
        self.name = name
        self.topic_name = topic_name
        self.node = None
        self.subscriber = None
        self.new_goal_received = False
        self.goal_data = None

        self.blackboard = py_trees.blackboard.Client(name=name, namespace="navigation3d")
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        self.subscriber = self.node.create_subscription(
            PoseStamped,
            self.topic_name,
            self._callback,
            10
        )
        return True

    def _callback(self, msg):

        p = msg.pose.position
        self.node.get_logger().info(f"New target : x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}")

        self.goal_data = msg
        self.new_goal_received = True

    def initialise(self):
        pass

    def update(self):
        if self.new_goal_received and self.goal_data:

            target = [
                self.goal_data.pose.position.x,
                self.goal_data.pose.position.y,
                self.goal_data.pose.position.z
            ]
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

        self.blackboard = py_trees.blackboard.Client(name=name, namespace="navigation3d")
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="trajectory", access=py_trees.common.Access.WRITE)

        self.node = None
        self.goal_pub = None
        self.traj_sub = None
        self.received_traj = None
        self.published = False
        self.start_time = None

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
        self.start_time = time.time()

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
            self.start_time = time.time()
            return Status.RUNNING

        if self.received_traj:
            self.blackboard.trajectory = self.received_traj
            return Status.SUCCESS

        if (time.time() - self.start_time) > 10.0:
            self.node.get_logger().error("PLANNER TIMEOUT")
            return Status.FAILURE
        return Status.RUNNING

class ChangeHeight(py_trees.behaviour.Behaviour):
    def __init__(self, name="Take Off", height=1.0, threshold=0.1): #10cm
        super(ChangeHeight, self).__init__(name)
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
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(f"TF Error (TakeOff): {e}", throttle_duration_sec=1.0)
            return Status.RUNNING

        error = self.target_height - current_z
        #self.node.get_logger().info(f"H_actuelle: {current_z:.2f}m | Cible: {self.target_height:.2f}m", throttle_duration_sec=1.0)

        if abs(error) < self.threshold:
            self.publisher.publish(Twist())
            self.node.get_logger().info("Fin Décollage")

            return Status.SUCCESS

        if self.target_height > 0.3 and current_z > self.target_height:
            self.node.get_logger().info(f"Déjà a {current_z:.2f}m")
            return Status.SUCCESS
        kp = 1.0
        vel_z = kp * error
        vel_z = max(min(vel_z, 0.5), -0.5) # Clamp

        if vel_z > 0 and vel_z < 0.1: vel_z = 0.2
        elif vel_z < 0 and vel_z > -0.1: vel_z = -0.1

        msg = Twist()
        msg.linear.z = float(vel_z)
        self.publisher.publish(msg)

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID:
            self.publisher.publish(Twist())

class FollowTrajectoryBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name="Follow Trajectory Behavior"):
        super(FollowTrajectoryBehavior, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name, namespace="navigation3d")
        self.blackboard.register_key(key="trajectory", access=py_trees.common.Access.READ)
        self.action_client = None
        self.goal_handle = None
        self.future = None
        self.result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, FollowTrajectory, 'follow_trajectory')
        return True

    def initialise(self):
        traj = self.blackboard.trajectory
        if traj is None:
            return

        if traj is None:
            self.node.get_logger().error("pas de traj")
            return

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action Server introuvable")
            return

        goal_msg = FollowTrajectory.Goal()
        goal_msg.trajectory = traj

        self.node.get_logger().info("FollowTrajectory")
        self.future = self.action_client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None

    def feedback_cb(self, feedback_msg):
        # self.node.get_logger().info(f"Feedback: {feedback_msg.feedback.time_remaining}")
        pass

    def update(self):
        if self.blackboard.trajectory is None or self.future is None:
            return Status.FAILURE

        if self.goal_handle is None:
            if self.future.done():
                self.goal_handle = self.future.result()
                if not self.goal_handle.accepted:
                    self.node.get_logger().error("FollowTrajectory: Goal reject")
                    return Status.FAILURE
                self.node.get_logger().info("FollowTrajectory: Goal accept")
                self.result_future = self.goal_handle.get_result_async()
                #return Status.RUNNING
            return Status.RUNNING

        if self.result_future.done():
            res = self.result_future.result()
            if res.result.success:
                return Status.SUCCESS
            else:
                return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.goal_handle:
            if self.result_future and not self.result_future.done():
                self.node.get_logger().warn("cancel trajectory")
                self.goal_handle.cancel_goal_async()