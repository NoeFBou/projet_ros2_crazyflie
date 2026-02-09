import py_trees
import rclpy
from rclpy.node import Node
from py_trees_ros import trees
from py_trees.common import Status
from navigation3d.navigation3d.behavior.action import WaitGoal, TrajectoriesPlanner
from navigation3d.navigation3d.behavior.test import TakeOff

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.declare_parameter("topic_name","/target_pose")
        self.declare_parameter("topic_name","/target_pose")

    def setup_tree(self) :

        root = py_trees.composites.Selector(name="Supervisor")
        #branch batterie
        #battery_branch = py_trees.composites.Sequence(name="battery branch", memory=True)


        #branche navigation
        navigation_branch = py_trees.composites.Sequence(name="navigation branch", memory=True)

        wait_goal = WaitGoal()
        trajectory = TrajectoriesPlanner()
        check_battey_trajectory = CheckBatteryTrajectory()
        takeoff = TakeOff(height = 0.5)
        follow_trajectory = FollowTrajectory()
        land = TakeOff(height = 0.05)


        navigation_battery.add_child(wait_goal,trajectory,check_battey_trajectory, takeoff, follow_trajectory,land)

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