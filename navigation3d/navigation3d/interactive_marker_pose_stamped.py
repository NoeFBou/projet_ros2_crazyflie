#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker
)


class InteractiveGoal(Node):

    def __init__(self):
        super().__init__('interactive_target_pose')
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("default_x_pose", 17.63)
        self.declare_parameter("default_y_pose", -0.84)
        self.declare_parameter("default_z_pose", 5.39)

        # Publisher PoseStamped
        self.pose_pub = self.create_publisher(PoseStamped,self.get_parameter("target_topic").value,10)

        # Interactive Marker server 
        self.server = InteractiveMarkerServer(self,'goal_3d_marker')

        self.make_marker()
        self.server.applyChanges()

        self.get_logger().info("Interactive Marker 3D + PoseStamped Ready! ")

    def feedback_marker(self, feedback: InteractiveMarkerFeedback):

        # Publish only on after the user releases the mouse button
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter("frame_id").value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = feedback.pose


        self.pose_pub.publish(pose)

        p = pose.pose.position
        self.get_logger().info(
            f"Goal published: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
        )

    def make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.get_parameter("frame_id").value
        int_marker.name = 'goal_3d'
        int_marker.description = 'Goal 3D'
        int_marker.scale = 0.4

        # Initial position
        int_marker.pose.position.x = self.get_parameter("default_x_pose").value
        int_marker.pose.position.y = self.get_parameter("default_y_pose").value
        int_marker.pose.position.z = self.get_parameter("default_z_pose").value

        # Rviz visual
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.15
        sphere.scale.y = 0.15
        sphere.scale.z = 0.15
        sphere.color.r = 0.2
        sphere.color.g = 0.9
        sphere.color.b = 0.2
        sphere.color.a = 1.0

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.markers.append(sphere)
        visual.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(visual)

        # Add axis controls
        self.add_axis_control(int_marker, "move_x", 1.0, 0.0, 0.0)
        self.add_axis_control(int_marker, "move_y", 0.0, 1.0, 0.0)
        self.add_axis_control(int_marker, "move_z", 0.0, 0.0, 1.0)

        # Insert and set callback
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.feedback_marker)

    def add_axis_control(self, marker, name, x, y, z):
        control = InteractiveMarkerControl()
        q = [1.0, 0.0, 0.0, 0.0]

        if x == 1:
            control.name = "move_x"
            q = [1.0, 0.0, 0.0, 0.0]
        elif y == 1:
            control.name = "move_y"
            val = math.sqrt(2)/2
            q = [val, 0.0, 0.0, val]
        elif z == 1:
            control.name = "move_z"
            val = math.sqrt(2)/2
            q = [val, 0.0, -val, 0.0]

        control.orientation.w = q[0]
        control.orientation.x = q[1]
        control.orientation.y = q[2]
        control.orientation.z = q[3]

        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

def main():
    rclpy.init()
    node = InteractiveGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
