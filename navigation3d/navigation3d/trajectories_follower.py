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

        self.declare_parameter("traj_topic", "/planned_trajectory_final")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_frame", "crazyflie/base_footprint")
        self.declare_parameter("kp_xy", 1.5)
        self.declare_parameter("kp_z", 1.5)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("period", 0.02) # 0.02 = 50Hz

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

        self.timer = self.create_timer(self.get_parameter("period").value, self.control_loop)

    def traj_callback(self, msg: MultiDOFJointTrajectory):
        self.get_logger().info(f"Received trajectory")
        self.trajectory = msg
        self.start_time = self.get_clock().now()
        self.tracking_active = True
        self.index_lst_point = 0

    def get_drone_state(self):
        try:
            t = self.tf_buffer.lookup_transform("map", self.get_parameter("robot_frame").value, rclpy.time.Time())
            pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(q)
            return pos, yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

    def get_point_at_time(self, time_from_start_sec):

        points = self.trajectory.points
        total_points = len(points)

        # self.get_logger().info(f"Total points: {total_points}")
        # self.get_logger().info(f"Time since start: {time_from_start_sec}")
        # self.get_logger().info(f"lst index point: {self.index_lst_point}")

        if self.index_lst_point >= total_points:
            self.index_lst_point = total_points - 1


        while self.index_lst_point < total_points - 1:
            next_idx = self.index_lst_point + 1
            pt_next = points[next_idx]


            t_next = pt_next.time_from_start.sec + pt_next.time_from_start.nanosec * 1e-9

            if t_next > time_from_start_sec: #bon segment
                break

            self.index_lst_point += 1

        finished = False
        if self.index_lst_point == total_points - 1:
            last_pt = points[-1]
            t_last = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9
            if time_from_start_sec >= t_last:
                finished = True

        pt = points[self.index_lst_point]

        #self.get_logger().info(f"Finished: {finished}")

        point_position = np.array([
            pt.transforms[0].translation.x,
            pt.transforms[0].translation.y,
            pt.transforms[0].translation.z
        ])

        point_velocities = np.array([
            pt.velocities[0].linear.x,
            pt.velocities[0].linear.y,
            pt.velocities[0].linear.z
        ])

        point_orientation = [
            pt.transforms[0].rotation.x,
            pt.transforms[0].rotation.y,
            pt.transforms[0].rotation.z,
            pt.transforms[0].rotation.w
        ]

        point_accel = np.array([
            pt.accelerations[0].linear.x,
            pt.accelerations[0].linear.y,
            pt.accelerations[0].linear.z
        ])

        _, _, point_drone_dir = tf_transformations.euler_from_quaternion(point_orientation)
        point_drone_dir_rot = pt.velocities[0].angular.z

        return (point_position, point_velocities, point_drone_dir, point_drone_dir_rot, point_accel), finished

    def control_loop(self):
        if not self.tracking_active or self.trajectory is None:
            return

        now = self.get_clock().now()
        time_elapsed = (now - self.start_time).nanoseconds * 1e-9

        setpoint, finished = self.get_point_at_time(time_elapsed)

        if finished:
            self.pub_cmd.publish(Twist())
            if time_elapsed > (self.trajectory.points[-1].time_from_start.sec + 2.0):
                self.tracking_active = False
                self.get_logger().info("Trajectory Finished.")
            return

        point_position, point_velocities, point_drone_dir, point_drone_dir_rot, point_accel = setpoint

        curr_pos, curr_yaw = self.get_drone_state()
        if curr_pos is None:
            return


        pos_err = point_position - curr_pos

        # Vitesse Désirée
        kx = self.get_parameter("kp_xy").value
        kz = self.get_parameter("kp_z").value

        v_cmd_world = point_velocities +(point_accel * 0.1)+ np.array([
            kx * pos_err[0],
            kx * pos_err[1],
            kz * pos_err[2]
        ])

        # Erreur de Yaw
        yaw_err = point_drone_dir - curr_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err)) # Normalisation -pi, pi

        yaw_cmd = point_drone_dir_rot + self.get_parameter("kp_yaw").value * yaw_err

        c = math.cos(curr_yaw)
        s = math.sin(curr_yaw)

        v_x_body =  c * v_cmd_world[0] + s * v_cmd_world[1]
        v_y_body = -s * v_cmd_world[0] + c * v_cmd_world[1]
        v_z_body = v_cmd_world[2]

        # secu
        max_v = self.get_parameter("max_speed").value
        norm_xy = math.sqrt(v_x_body**2 + v_y_body**2)
        if norm_xy > max_v:
            scale = max_v / norm_xy
            v_x_body *= scale
            v_y_body *= scale

        msg = Twist()
        msg.linear.x = v_x_body
        msg.linear.y = v_y_body
        msg.linear.z = v_z_body
        msg.angular.z = yaw_cmd

        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = TrajectoriesFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()