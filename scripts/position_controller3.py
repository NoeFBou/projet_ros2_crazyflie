#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
import yaml
import math
import sys
from tf_transformations import euler_from_quaternion


class MappingController(Node):
    def __init__(self, waypoint_file):
        super().__init__('mapping_controller')
        self.get_logger().info("Initialisation : Scan Rapide + Sécurité Altitude...")

        self.P_GAIN_XY = 0.8
        self.P_GAIN_Z = 1.2
        self.P_GAIN_YAW = 1.0

        self.SCAN_YAW_RATE = 0.8
        self.MAX_SPEED_XY = 0.3
        self.MAX_SPEED_Z = 0.6

        self.MIN_SAFE_HEIGHT = 0.4
        self.TAKEOFF_HEIGHT = 0.5
        self.is_taking_off = True

        self.WAYPOINT_THRESHOLD_POS = 0.15
        self.PATH_DISTANCE_THRESHOLD = 0.1

        self.current_pos = None
        self.current_yaw = 0.0
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        self.odom_frame_id = None
        self.path_msg = Path()

        self.odom_sub = self.create_subscription(Odometry, '/crazyflie/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/crazyflie/visualization_markers', 10)
        self.path_pub = self.create_publisher(Path, '/crazyflie/path', 10)

        self.control_timer = self.create_timer(0.05, self.control_loop)

    def load_waypoints(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                return data['waypoints']
        except Exception as e:
            self.get_logger().error(f"Erreur YAML: {e}")
            return []

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        if self.odom_frame_id is None:
            self.odom_frame_id = msg.header.frame_id
            self.path_msg.header.frame_id = self.odom_frame_id

        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

        drone_marker = Marker()
        drone_marker.header = msg.header
        drone_marker.ns = "drone_body"
        drone_marker.id = 0
        drone_marker.type = Marker.CUBE
        drone_marker.action = Marker.ADD
        drone_marker.pose = msg.pose.pose
        drone_marker.scale.x = 0.15
        drone_marker.scale.y = 0.15
        drone_marker.scale.z = 0.05
        drone_marker.color.r = 0.0
        drone_marker.color.g = 1.0
        drone_marker.color.b = 1.0
        drone_marker.color.a = 1.0
        self.marker_pub.publish(drone_marker)

        self.update_path(msg)

    def update_path(self, odom_msg):
        if not self.path_msg.poses:
            self.path_msg.poses.append(PoseStamped(header=odom_msg.header, pose=odom_msg.pose.pose))
        else:
            last_pos = self.path_msg.poses[-1].pose.position
            dist = math.sqrt((self.current_pos.x - last_pos.x) ** 2 + (self.current_pos.y - last_pos.y) ** 2)
            if dist > self.PATH_DISTANCE_THRESHOLD:
                self.path_msg.poses.append(PoseStamped(header=odom_msg.header, pose=odom_msg.pose.pose))
                self.path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.path_msg)

    def control_loop(self):
        if self.current_pos is None:
            return

        cmd_vel = Twist()

        if self.is_taking_off:
            if self.current_pos.z < (self.TAKEOFF_HEIGHT - 0.1):
                self.get_logger().info(f"Décollage... Z: {self.current_pos.z:.2f}m")
                cmd_vel.linear.z = 0.5
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                return
            else:
                self.get_logger().info("Décollage terminé.")
                self.is_taking_off = False

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Mission terminée.")
            self.stop_and_hover(land=True)
            return

        target_wp = self.waypoints[self.current_waypoint_index]
        target_z_safe = max(float(target_wp['z']), self.MIN_SAFE_HEIGHT)
        target_pos = {'x': target_wp['x'], 'y': target_wp['y'], 'z': target_z_safe}

        target_marker = Marker()
        target_marker.header.frame_id = self.odom_frame_id or "odom"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_wp"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target_pos['x']
        target_marker.pose.position.y = target_pos['y']
        target_marker.pose.position.z = target_pos['z']
        target_marker.scale.x = 0.2;
        target_marker.scale.y = 0.2;
        target_marker.scale.z = 0.2
        target_marker.color.r = 1.0;
        target_marker.color.g = 0.0;
        target_marker.color.b = 0.0;
        target_marker.color.a = 0.8
        self.marker_pub.publish(target_marker)

        error_x_world = target_pos['x'] - self.current_pos.x
        error_y_world = target_pos['y'] - self.current_pos.y
        error_z = target_pos['z'] - self.current_pos.z
        dist_error_xy = math.sqrt(error_x_world ** 2 + error_y_world ** 2)

        if self.current_pos.z < (self.MIN_SAFE_HEIGHT - 0.1):
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.6
            self.cmd_vel_pub.publish(cmd_vel)
            return

        if dist_error_xy > self.WAYPOINT_THRESHOLD_POS:
            self.waypoint_reached_time = None

            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            body_vel_x = (cos_yaw * error_x_world) + (sin_yaw * error_y_world)
            body_vel_y = (-sin_yaw * error_x_world) + (cos_yaw * error_y_world)

            cmd_vel.linear.x = self.P_GAIN_XY * body_vel_x
            cmd_vel.linear.y = self.P_GAIN_XY * body_vel_y

            cmd_vel.angular.z = self.SCAN_YAW_RATE
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0

            if self.waypoint_reached_time is None:
                self.waypoint_reached_time = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.waypoint_reached_time).nanoseconds / 1e9
            hold_time = target_wp.get('hold_time', 2.0)
            if elapsed >= hold_time:
                self.current_waypoint_index += 1
                self.waypoint_reached_time = None

        cmd_vel.linear.z = self.P_GAIN_Z * error_z

        speed_xy = math.sqrt(cmd_vel.linear.x ** 2 + cmd_vel.linear.y ** 2)
        if speed_xy > self.MAX_SPEED_XY:
            scale = self.MAX_SPEED_XY / speed_xy
            cmd_vel.linear.x *= scale
            cmd_vel.linear.y *= scale

        cmd_vel.linear.z = max(min(cmd_vel.linear.z, self.MAX_SPEED_Z), -self.MAX_SPEED_Z)
        self.cmd_vel_pub.publish(cmd_vel)

    def stop_and_hover(self, land=False):
        cmd = Twist()
        if land: cmd.linear.z = -0.15
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python3 mapping_safe.py <waypoints.yaml>")
        return
    node = MappingController(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_hover(land=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()