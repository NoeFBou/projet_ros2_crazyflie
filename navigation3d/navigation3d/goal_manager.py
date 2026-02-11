# #!/usr/bin/env python3
# from typing import Optional, List
# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
#
# from geometry_msgs.msg import PointStamped
# from rcl_interfaces.srv import SetParameters
#
# #tmp
# class GoalManager(Node):
#
#     def __init__(self) -> None:
#         super().__init__("goal_manager")
#
#         self.declare_parameter("mode", "goal_only")  # "goal_only" or "two_click"
#         self.declare_parameter("planner_node", "/planner")
#         self.declare_parameter("clicked_topic", "/clicked_point")
#         self.declare_parameter("default_start", [10.0, 0.0, 2.5])
#         self.declare_parameter("fixed_z", 0.5)  # if you want to override z from click (optional)
#         self.declare_parameter("use_fixed_z", True)
#
#         self.mode = self.get_parameter("mode").value
#         self.planner_node = self.get_parameter("planner_node").value
#         self.default_start = list(self.get_parameter("default_start").value)
#         self.use_fixed_z = bool(self.get_parameter("use_fixed_z").value)
#         self.fixed_z = float(self.get_parameter("fixed_z").value)
#
#         self._awaiting_start = True
#         self._start: List[float] = self.default_start
#
#         self.cli = self.create_client(SetParameters, f"{self.planner_node}/set_parameters")
#         if not self.cli.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn(
#                 f"Service {self.planner_node}/set_parameters not available yet. "
#                 "GoalManager will keep trying when clicks arrive."
#             )
#
#         self.sub = self.create_subscription(
#             PointStamped,
#             self.get_parameter("clicked_topic").value,
#             self._on_click,
#             10,
#         )
#
#         self.get_logger().info(
#             f"GoalManager ready. mode={self.mode}, planner_node={self.planner_node}, topic={self.get_parameter('clicked_topic').value}"
#         )
#
#         # Initialize planner start once in goal_only mode
#         if self.mode == "goal_only":
#             self._set_planner_param("start", self._start)
#
#     def _on_click(self, msg: PointStamped) -> None:
#         x = float(msg.point.x)
#         y = float(msg.point.y)
#         z = self.fixed_z if self.use_fixed_z else float(msg.point.z)
#
#         if self.mode == "goal_only":
#             z +=1
#             goal = [x, y, z]
#             self.get_logger().info(f"Clicked goal: {goal}")
#             self._set_planner_param("goal", goal)
#             return
#
#         if self.mode == "two_click":
#             if self._awaiting_start:
#                 self._start = [x, y, z]
#                 self.get_logger().info(f"Clicked start: {self._start}")
#                 self._set_planner_param("start", self._start)
#                 self._awaiting_start = False
#             else:
#                 goal = [x, y, z]
#                 self.get_logger().info(f"Clicked goal: {goal}")
#                 self._set_planner_param("goal", goal)
#                 self._awaiting_start = True
#             return
#
#         self.get_logger().warn(f"Unknown mode: {self.mode}")
#
#     def _set_planner_param(self, name: str, value: List[float]) -> None:
#         if not self.cli.service_is_ready():
#             # Try again quickly
#             if not self.cli.wait_for_service(timeout_sec=0.2):
#                 self.get_logger().warn("Planner set_parameters service not ready.")
#                 return
#
#         req = SetParameters.Request()
#         req.parameters = [
#             Parameter(name=name, value=value).to_parameter_msg()
#         ]
#         fut = self.cli.call_async(req)
#
#         def _done(_):
#             try:
#                 resp = fut.result()
#                 if not resp.results or not resp.results[0].successful:
#                     reason = resp.results[0].reason if resp.results else "unknown"
#                     self.get_logger().warn(f"Failed to set {name} on planner: {reason}")
#             except Exception as e:
#                 self.get_logger().warn(f"Service call failed: {e}")
#
#         fut.add_done_callback(_done)
#
#
# def main():
#     rclpy.init()
#     node = GoalManager()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()
