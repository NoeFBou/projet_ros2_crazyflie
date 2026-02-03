import math
import heapq
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from pointcloud_builder.lib.minimum_snap import Waypoint, compute_trajectory_derivatives,generate_trajectory, compute_quadrotor_trajectory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rcl_interfaces.msg import SetParametersResult
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from pointcloud_builder.lib.astar import AStar
from pointcloud_builder.lib.octomap_reader import OctomapReader

# todo voir  Ramer-Douglas-Peucker

GridIdx = Tuple[int, int, int]

class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        # Params
        self.declare_parameter("occupied_cloud_topic", "/octomap_point_cloud_centers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid_res", 0.20)          # meters 0.2
        self.declare_parameter("inflation", 0.20)         # meters
        self.declare_parameter("bbox_margin", 1.0)        # meters
        self.declare_parameter("start", [10.0, 0.0, 2.5])  # x,y,z
        self.declare_parameter("goal",  [36.63155746459961, -60.41566467285156, 3.5])  # x,y,z
        self.declare_parameter("drone_mass", 0.032 )
        self.declare_parameter("sample_rate", 0.05)
        self.declare_parameter("avg_speed", 0.5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "crazyflie/base_footprint")
        # self.frame_id = self.get_parameter("frame_id").value
        # self.step = 5
        # qos_sub = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1,
        # )

        self.map_reader = OctomapReader(
            grid_res=self.get_parameter("grid_res").value,
            inflation=self.get_parameter("inflation").value
        )

        qos_map = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.occ_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter("occupied_cloud_topic").value,
            self._on_cloud_callback,
            qos_map,
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_basic = self.create_publisher(Path, "planned_path_astar_basic", latching_qos)
        self.marker_pub_basic = self.create_publisher(Marker, "planned_path_marker", latching_qos)

        self.path_pruned = self.create_publisher(Path, "planned_path_astar_pruned", latching_qos)
        self.marker_pub_pruned = self.create_publisher(Marker, "planned_path_marker_pruned", latching_qos)

        self.path_snaped_basic = self.create_publisher(Path, "planned_path_astar_Snaped_basic", latching_qos)
        self.marker_pub_snaped = self.create_publisher(Marker, "planned_path_marker_snaped", latching_qos)

        self.path_verified_basic = self.create_publisher(Path, "planned_path_Snaped_verif", latching_qos)
        self.marker_pub_verif = self.create_publisher(Marker, "planned_path_marker_snaped_verif", latching_qos)

        self.traj_pub = self.create_publisher(MultiDOFJointTrajectory, "planned_trajectory_final", latching_qos)

        self.occupied: Set[GridIdx] = set()
        self.occupied_inflated: Set[GridIdx] = set()
        self.map_ready = False



        self.current_start = list(self.get_parameter("start").value)
        self.current_goal = list(self.get_parameter("goal").value)
        self.bbox_margin = self.get_parameter("bbox_margin").value

        # Replan when params change
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info("Planner started. Waiting for occupied cloud...")

    def _on_cloud_callback(self, msg: PointCloud2):
        self.get_logger().info("Map received, updating grid...")
        self.map_reader.update_map(msg)
        self.map_ready = True
        # if first time run planer
        #self.planner()

    # def load_octomap(self):
    #     reader  = OctomapReader(self.get_parameter("grid_res").value,self.get_parameter("inflation").value, self.get_parameter("occupied_cloud_topic").value)
    #     self.occupied=reader.read_octomap()
    #     self.map_ready = True


    #plan or replan when goal/grid are set or change
    def _on_params(self, params: List[Parameter]):
        updated = False

        for p in params:
            if p.name == "start":
                self.current_start = p.value
                updated = True
            elif p.name == "goal":
                self.current_goal = p.value
                updated = True
            elif p.name == "grid_res" or p.name == "inflation" or p.name=="bbox_margin":
                #todo
                pass

        if updated and self.map_ready:
            #self.get_logger().info(f"Params update detected. New Goal: {self.current_goal}")
            self.planner()

        return SetParametersResult(successful=True)

    # def _prune_path(self, path: List[GridIdx]) -> List[GridIdx]:
    #     if len(path) < 3: return path
    #     pruned = [path[0]]
    #     old_dir = (path[1][0]-path[0][0], path[1][1]-path[0][1], path[1][2]-path[0][2])
    #
    #     for i in range(1, len(path) - 1):
    #         cur, nxt = path[i], path[i+1]
    #         new_dir = (nxt[0]-cur[0], nxt[1]-cur[1], nxt[2]-cur[2])
    #         if new_dir != old_dir:
    #             pruned.append(cur)
    #             old_dir = new_dir
    #     pruned.append(path[-1])
    #     return pruned
    def get_drone_position(self) -> Optional[List[float]]:

        world_frame = self.get_parameter("world_frame").value
        robot_frame = self.get_parameter("robot_frame").value

        try:
            t = self.tf_buffer.lookup_transform(
                world_frame,
                robot_frame,
                rclpy.time.Time()
            )
            return [t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z]

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Impossible de récupérer la position du drone: {e}")
            return None

    def _prune_path(self, path: List[GridIdx]) -> List[GridIdx]:
        if len(path) < 3: return path

        pruned = [path[0]]
        old_dir = (path[1][0]-path[0][0], path[1][1]-path[0][1], path[1][2]-path[0][2])
        last_kept_idx = 0
        max_segment_dist = 1.5
        grid_res = self.get_parameter("grid_res").value

        for i in range(1, len(path) - 1):
            cur, nxt = path[i], path[i+1]
            new_dir = (nxt[0]-cur[0], nxt[1]-cur[1], nxt[2]-cur[2])

            last_kept = path[last_kept_idx]
            dist_sq = (cur[0]-last_kept[0])**2 + (cur[1]-last_kept[1])**2 + (cur[2]-last_kept[2])**2
            dist_meters = math.sqrt(dist_sq) * grid_res

            keep_point = False

            if new_dir != old_dir:
                keep_point = True

            elif dist_meters > max_segment_dist:
                keep_point = True

            if keep_point:
                pruned.append(cur)
                old_dir = new_dir
                last_kept_idx = i

        pruned.append(path[-1])
        return pruned

    def planner(self) -> None:
        real_pose = self.get_drone_position()

        if real_pose:
            start = real_pose
            self.get_logger().info(f"Start set to Drone Position: {start}")
        else:
            start = self.current_start
            self.get_logger().warn(f"Drone TF not found. Using param Start: {start}")

        goal = self.current_goal

        self.get_logger().info(f"Planning from {start} to {goal}")

        margin = self.get_parameter("bbox_margin").value

        astar = AStar(self.map_reader, bbox_margin=margin)
        path_astar = astar.solve_a_star(start, goal)
        if not path_astar:
            self.get_logger().warn("A* failed: No path found.")
            return


        self.get_logger().info(f"A* found: {len(path_astar)} voxels")
        self._publish_grid_path(path_astar, self.path_basic, self.marker_pub_basic, ns="basic", color=(1.0, 0.0, 0.0))

        path_pruned = self._prune_path(path_astar)
        self.get_logger().info(f"Pruned: {len(path_pruned)} waypoints")
        self._publish_grid_path(path_pruned, self.path_pruned, self.marker_pub_pruned, ns="pruned", color=(1.0, 1.0, 0.0))

        waypoints = self._path_to_timed_waypoints(path_pruned) #path_pruned
        path_snaped, _, _ = self.waypoint_to_trajectory(waypoints) #tmp

        self.get_logger().info(f"Trajectory generated: {len(path_snaped.position)} samples")
        self._publish_trajectory(path_snaped, self.path_snaped_basic, self.marker_pub_snaped, ns="snaped", color=(0.0, 1.0, 0.0))


        max_retries = 50
        path_verified, t_samples, acceleration = self.waypoint_to_trajectory(waypoints)
        for attempt in range(max_retries):
            is_safe, waypoints = self._check_and_repair(path_verified, waypoints, path_astar)
            if is_safe:
                self.get_logger().info(f"Trajectory valid after {attempt} repairs.")
                break
            else:
                if attempt == max_retries - 1:
                    self.get_logger().error("Max retries reached. Trajectory might be unsafe.")
            path_verified, t_samples, acceleration = self.waypoint_to_trajectory(waypoints)

        self.get_logger().info(f"Trajectory generated: {len(path_verified.position)} samples")
        self._publish_trajectory(path_verified, self.path_verified_basic, self.marker_pub_verif, ns="verif", color=(0.0, 1.0, 1.0))

        self.publish_final_trajectory(path_verified,t_samples, acceleration)


    def _check_and_repair(self, quad_traj, current_waypoints: List[Waypoint], raw_astar_path: List[GridIdx]) -> Tuple[bool, List[Waypoint]]:

        positions = quad_traj.position
        times = np.linspace(0, current_waypoints[-1].time, len(positions))

        collision_idx = -1

        for i, pos in enumerate(positions):
            grid_idx = self.map_reader.world_to_grid(pos)

            if self.map_reader.is_occupied(grid_idx):
                collision_idx = i
                break

        if collision_idx == -1:
            return True, current_waypoints

        self.get_logger().warn(f"Collision detected at t={times[collision_idx]:.2f}s! Repairing...")

        collision_pos = positions[collision_idx]
        best_dist = float('inf')
        safest_grid_idx = None

        for idx in raw_astar_path:
            pt_world = np.array(self.map_reader.grid_to_world(idx))
            dist = np.linalg.norm(pt_world - collision_pos)

            if dist < best_dist:
                best_dist = dist
                safest_grid_idx = idx

        safe_pos = np.array(self.map_reader.grid_to_world(safest_grid_idx))

        collision_time = times[collision_idx]
        new_wp = Waypoint(time=collision_time, position=safe_pos)

        new_waypoints = []
        inserted = False
        for wp in current_waypoints:
            if not inserted and wp.time > collision_time:
                new_waypoints.append(new_wp)
                inserted = True
            new_waypoints.append(wp)

        return False, new_waypoints

    def _path_to_timed_waypoints(self, grid_path: List[GridIdx]) -> List[Waypoint]:
        avg_speed = self.get_parameter("avg_speed").value
        raw_points = [self.map_reader.grid_to_world(idx) for idx in grid_path]

        waypoints = []
        current_time = 0.0

        for i, pos_list in enumerate(raw_points):
            pos = np.array(pos_list)

            if i > 0:
                prev_pos = np.array(raw_points[i-1])
                dist = np.linalg.norm(pos - prev_pos)

                segment_time = dist / avg_speed

                if i > 1:
                    prev_prev = np.array(raw_points[i-2])
                    v1 = prev_pos - prev_prev
                    v2 = pos - prev_pos

                    norm_v1 = np.linalg.norm(v1)
                    norm_v2 = np.linalg.norm(v2)

                    if norm_v1 > 1e-6 and norm_v2 > 1e-6:
                        dir1 = v1 / norm_v1
                        dir2 = v2 / norm_v2
                        dot_prod = np.dot(dir1, dir2)
                        dot_prod = max(-1.0, min(1.0, dot_prod))

                        angle = math.acos(dot_prod)

                        segment_time += (angle * 2)  # Facteur 1.5 à ajuster

                segment_time = max(segment_time, 0.5)

                current_time += segment_time

            if i == 0 or i == len(raw_points) - 1:
                wp = Waypoint(time=current_time, position=pos, velocity=[0,0,0], acceleration=[0,0,0])
            else:
                wp = Waypoint(time=current_time, position=pos)

            waypoints.append(wp)

        return waypoints
    def waypoint_to_trajectory(self, waypoints):

        traj_poly = generate_trajectory(waypoints, degree=7, idx_minimized_orders=4)

        t_total = waypoints[-1].time
        t_samples = np.arange(0, t_total, self.get_parameter("sample_rate").value)

        quad_traj = compute_quadrotor_trajectory(
            traj_poly,
            t_samples,
            vehicle_mass=self.get_parameter("drone_mass").value
        )
        #[positions, attitudes, velocities]
        #attitude = [qx, qy, qz, qw]
        #self.get_logger().info(f"Test={quad_traj.velocity}")


        #accelration
        derivatives = compute_trajectory_derivatives(traj_poly, t_samples, 4)
        #[Pos, Vel, Acc, Jerk]
        acceleration = derivatives[2]

        return quad_traj, t_samples, acceleration

    def _publish_grid_path(self, grid_path, path_pub, marker_pub, ns, color):
        world_points = [self.map_reader.grid_to_world(idx) for idx in grid_path]
        self._publish_visuals(world_points, path_pub, marker_pub, ns, color)

    def _publish_trajectory(self, quad_traj, path_pub, marker_pub, ns, color):
        # quad_traj.position est un numpy array (N, 3)
        points = quad_traj.position.tolist()
        self._publish_visuals(points, path_pub, marker_pub, ns, color)

    def _publish_visuals(self, points_xyz, path_pub, marker_pub, ns, color):
        frame_id = self.get_parameter("frame_id").value
        now = self.get_clock().now().to_msg()

        msg_path = Path()
        msg_path.header.frame_id = frame_id
        msg_path.header.stamp = now

        for p in points_xyz:
            ps = PoseStamped()
            ps.header = msg_path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            ps.pose.orientation.w = 1.0
            msg_path.poses.append(ps)

        path_pub.publish(msg_path)

        # 2. Message Marker (Ligne)
        marker = Marker()
        marker.header = msg_path.header
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05 # Epaisseur ligne
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color

        for p in points_xyz:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
            marker.points.append(pt)

        marker_pub.publish(marker)

    def publish_final_trajectory(self,quad_traj, timestamps, acceleration):
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = self.get_parameter("frame_id").value
        traj.joint_names=["base_footprint"] #base_link test

        for i, t in enumerate(timestamps):
            point = MultiDOFJointTrajectoryPoint()
            point.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()

            trans = Transform()
            trans.translation.x = float(quad_traj.position[i][0])
            trans.translation.y = float(quad_traj.position[i][1])
            trans.translation.z = float(quad_traj.position[i][2])

            trans.rotation.x = float(quad_traj.attitude[i][0])
            trans.rotation.y = float(quad_traj.attitude[i][1])
            trans.rotation.z = float(quad_traj.attitude[i][2])
            trans.rotation.w = float(quad_traj.attitude[i][3])

            point.transforms.append(trans)

            vel = Twist()
            vel.linear.x = float(quad_traj.velocity[i][0])
            vel.linear.y = float(quad_traj.velocity[i][1])
            vel.linear.z = float(quad_traj.velocity[i][2])

            vel.angular.x = float(quad_traj.body_rates[i][0])
            vel.angular.y = float(quad_traj.body_rates[i][1])
            vel.angular.z = float(quad_traj.body_rates[i][2])

            point.velocities.append(vel)

            acc = Twist()
            acc.linear.x = float(acceleration[i][0])
            acc.linear.y = float(acceleration[i][1])
            acc.linear.z = float(acceleration[i][2])

            point.accelerations.append(acc)

            traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info("Trajectory published")


def main():
    rclpy.init()
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()