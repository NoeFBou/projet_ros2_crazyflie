import py_trees
import rclpy
from rclpy.node import Node
from py_trees_ros import trees
from py_trees.common import Status
from navigation3d.behavior.action import TrajectoriesPlanner, FollowTrajectoryBehavior,ChangeHeight,ClearTrajectory,CheckNewGoalAvailable,CheckHasTrajectory

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.declare_parameter("height_take_off",0.5)
        self.declare_parameter("height_landing",0.05)
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("traj_topic", "/planned_trajectory_final")
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_frame", "crazyflie/base_footprint")
        self.declare_parameter("speed_take_off", 0.2)
        self.declare_parameter("speed_landing", -0.1)
        self.declare_parameter("frame_id", "map")

    def setup_tree(self):
        bb = py_trees.blackboard.Client(name="Init", namespace="navigation3d")
        bb.register_key(key="trajectory", access=py_trees.common.Access.WRITE)
        bb.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

        bb.trajectory = None
        bb.target_pose = None
        root = py_trees.composites.Selector(name="Supervisor", memory=False)

        planning_branch = py_trees.composites.Sequence(name="PlanningBranch", memory=True)

        check_new_goal = CheckNewGoalAvailable(name="NewGoal?")
        planner = TrajectoriesPlanner(name="Planner")

        planning_branch.add_children([check_new_goal, planner])

        exec_branch = py_trees.composites.Sequence(name="ExecutionBranch", memory=True)

        has_traj = CheckHasTrajectory(name="HasTraj?")
        takeoff = ChangeHeight(name="TakeOff", height=self.get_parameter("height_take_off").value)
        follow = FollowTrajectoryBehavior(name="Follow")
        land = ChangeHeight(name="Land", height=self.get_parameter("height_landing").value)

        clear_traj = ClearTrajectory(name="ClearTraj")

        exec_branch.add_children([has_traj, takeoff, follow, land, clear_traj])

        idle = py_trees.behaviours.Running(name="Idle")

        root.add_children([planning_branch, exec_branch, idle])
        return root

def main():
    rclpy.init()
    supervisor_node = Supervisor()

    tree = trees.BehaviourTree(supervisor_node.setup_tree())
    tree.setup(node=supervisor_node)

    try:
        tree.tick_tock(period_ms=100)
        rclpy.spin(supervisor_node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()