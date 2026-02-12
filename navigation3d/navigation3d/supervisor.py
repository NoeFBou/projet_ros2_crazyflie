import py_trees
import rclpy
from rclpy.node import Node
from py_trees_ros import trees
from py_trees.common import Status
from navigation3d.behavior.action import WaitGoal, TrajectoriesPlanner, FollowTrajectoryBehavior,ChangeHeight

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

    def setup_tree(self) :

        root = py_trees.composites.Selector(name="Supervisor", memory=False)
        #branch batterie
        #battery_branch = py_trees.composites.Sequence(name="battery branch", memory=True)


        #branche navigation
        navigation_branch = py_trees.composites.Sequence(name="navigation branch", memory=True)

        wait_goal = WaitGoal()
        trajectory = TrajectoriesPlanner()
        #check_battey_trajectory = CheckBatteryTrajectory()
        takeoff = ChangeHeight(height = self.get_parameter("height_take_off").value)
        follow_trajectory = FollowTrajectoryBehavior(name="Follow Trajectory")
        land = ChangeHeight(height = self.get_parameter("height_landing").value)


        navigation_branch.add_children([wait_goal,trajectory, #check_battey_trajectory
                                     takeoff, follow_trajectory,land])

        root.add_child(navigation_branch)
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