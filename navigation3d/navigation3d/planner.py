import math
import heapq
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rcl_interfaces.msg import SetParametersResult
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion, PoseStamped, Point
from navigation3d.lib.astar import AStar
from navigation3d.lib.octomap_reader import OctomapReader
from navigation3d.lib.minimum_snap import Waypoint, compute_trajectory_derivatives,generate_trajectory, compute_quadrotor_trajectory

# todo voir  Ramer-Douglas-Peucker

GridIdx = Tuple[int, int, int]

class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        # Params
        self.declare_parameter("occupied_cloud_topic", "/octomap_point_cloud_centers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid_res", 0.20)          # meters 0.2
        self.declare_parameter("inflation_astar", 0.30)
        self.declare_parameter("inflation_minsnap", 0.10)
        self.declare_parameter("bbox_margin", 1.0)        # meters
        self.declare_parameter("drone_mass", 0.032 )
        self.declare_parameter("sample_rate", 0.05)
        self.declare_parameter("avg_speed", 0.5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot_frame", "crazyflie/base_footprint")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("force_goal_z", -1.0)
        # self.frame_id = self.get_parameter("frame_id").value
        # self.step = 5
        # qos_sub = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1,
        # )

        self.map_reader = OctomapReader(
            grid_res=self.get_parameter("grid_res").value,
            inflation_astar=self.get_parameter("inflation_astar").value,
            inflation_minsnap=self.get_parameter("inflation_minsnap").value
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
            self._occupied_cloud_callback,
            qos_map,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self._goal_callback,
            10
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.marker_pub_basic = self.create_publisher(Marker, "planned_path_marker", latching_qos)

        self.marker_pub_pruned = self.create_publisher(Marker, "planned_path_marker_pruned", latching_qos)

        self.marker_pub_snaped = self.create_publisher(Marker, "planned_path_marker_snaped", latching_qos)
        self.marker_pub_densify= self.create_publisher(Marker, "planned_path_marker_densify", latching_qos)
        # self.path_verified_basic = self.create_publisher(Path, "planned_path_Snaped_verif", latching_qos)
        self.marker_pub_verif = self.create_publisher(Marker, "planned_path_marker_snaped_verif", latching_qos)

        self.traj_pub = self.create_publisher(MultiDOFJointTrajectory, "planned_trajectory_final", latching_qos)

        self.occupied: Set[GridIdx] = set()
        self.occupied_inflated: Set[GridIdx] = set()
        self.map_ready = False

        self.bbox_margin = self.get_parameter("bbox_margin").value

        self.get_logger().info("Planner started. Waiting for occupied cloud...")

    def _occupied_cloud_callback(self, msg: PointCloud2):
        self.get_logger().info("Map received, updating grid...")
        self.map_reader.update_map(msg)
        self.map_ready = True

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

    def _goal_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        force_z = self.get_parameter("force_goal_z").value

        if force_z > 0.0:
            z = force_z
        elif abs(z) < 0.01:
            default_goal = self.get_parameter("goal").value
            z = default_goal[2]
            self.get_logger().info(f"goal z is <0 remplacement by {z}")

        self.current_goal = [x, y, z]
        self.get_logger().info(f"new goal : {self.current_goal}")

        if self.map_ready:
            self.planner()
        else:
            self.get_logger().warn("map not ready yet")


    # def _prune_path(self, path: List[GridIdx]) -> List[GridIdx]:
    #     if len(path) < 3: return path
    #
    #     pruned = [path[0]]
    #     old_dir = (path[1][0]-path[0][0], path[1][1]-path[0][1], path[1][2]-path[0][2])
    #     last_kept_idx = 0
    #     max_segment_dist = 1.5
    #     grid_res = self.get_parameter("grid_res").value
    #
    #     for i in range(1, len(path) - 1):
    #         cur, nxt = path[i], path[i+1]
    #         new_dir = (nxt[0]-cur[0], nxt[1]-cur[1], nxt[2]-cur[2])
    #
    #         last_kept = path[last_kept_idx]
    #         dist_sq = (cur[0]-last_kept[0])**2 + (cur[1]-last_kept[1])**2 + (cur[2]-last_kept[2])**2
    #         dist_meters = math.sqrt(dist_sq) * grid_res
    #
    #         keep_point = False
    #
    #         if new_dir != old_dir:
    #             keep_point = True
    #
    #         elif dist_meters > max_segment_dist:
    #             keep_point = True
    #
    #         if keep_point:
    #             pruned.append(cur)
    #             old_dir = new_dir
    #             last_kept_idx = i
    #
    #     pruned.append(path[-1])
    #     return pruned

    def _densify_path(self, path_idx: List[GridIdx], max_dist: float = 1.0) -> List[GridIdx]:
        if len(path_idx) < 2:
            return path_idx

        # Conversion en points réels pour calculer les distances
        points = [np.array(self.map_reader.grid_to_world(idx)) for idx in path_idx]

        densified_indices = []
        p0 = points[0]
        p1 = points[1]
        #anchor pour le debut
        dist_start = np.linalg.norm(p1 - p0)
        if dist_start > 0.5:
            direction = (p1 - p0) / dist_start

            anchor_pt = p0 + direction * 0.30
            anchor_idx = self.map_reader.world_to_grid(anchor_pt)
            densified_indices.append(anchor_idx)

        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i+1]

            dist = np.linalg.norm(p2 - p1)

            if dist > max_dist:
                num_segments = math.ceil(dist / max_dist)

                for j in range(1, num_segments):
                    alpha = j / num_segments
                    new_pt_world = p1 + alpha * (p2 - p1)

                    new_idx = self.map_reader.world_to_grid(new_pt_world)
                    densified_indices.append(new_idx)

            densified_indices.append(path_idx[i+1])

        return densified_indices

    def _rdp(self, path: List[GridIdx]) -> List[GridIdx]:
        """
        Ramer-Douglas-Peucker.
        """

        if len(path) < 3: return path

        point_list = np.array([self.map_reader.grid_to_world(idx) for idx in path])

        # Epsilon : Tolérance de simplification. Supprime les points à moins de epsilon m de la ligne droite.
        epsilon = 0.08

        # Appel recursif
        mask = self._rec_rdp(point_list, epsilon)

        pruned_path = [path[i] for i in range(len(path)) if mask[i]]

        return pruned_path

    def _rec_rdp(self, points, epsilon):
        dmax = 0.0
        index = 0
        end = len(points) - 1

        if end < 2:
            return [True] * len(points)

        start_pt = points[0]
        end_pt = points[end]

        line_vec = end_pt - start_pt
        line_len = np.linalg.norm(line_vec)

        if line_len == 0:
            return [True] * len(points)

        line_unit = line_vec / line_len

        vec_start_to_points = points[1:end] - start_pt

        scalar_projections = np.dot(vec_start_to_points, line_unit)

        proj_vecs = np.outer(scalar_projections, line_unit)

        perp_vecs = vec_start_to_points - proj_vecs
        dists = np.linalg.norm(perp_vecs, axis=1)

        if len(dists) > 0:
            dmax = np.max(dists)
            index = np.argmax(dists) + 1

        results = [False] * len(points)

        if dmax > epsilon:
            rec_results1 = self._rec_rdp(points[:index+1], epsilon)
            rec_results2 = self._rec_rdp(points[index:], epsilon)

            results = rec_results1[:-1] + rec_results2
        else:
            results[0] = True
            results[-1] = True

        return results

    def planner(self) -> None:
        real_pose = self.get_drone_position()

        if real_pose:
            start = real_pose
            self.get_logger().info(f"Start set to Drone Position: {start}")
            if start[2] < 0.3:
                self.get_logger().warn(f"Drone au sol ({start[2]:.2f}m).")
                start[2] = 1# 0.5
        else:
            self.get_logger().warn(f"Drone TF not found")
            return
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
        astar_lenght = self._aster_lenght(path_astar)


        self.get_logger().info(f"A* found: {len(path_astar)} voxels")
        self._publish_grid_path(path_astar,  self.marker_pub_basic, ns="basic", color=(1.0, 0.0, 0.0))

        path_pruned = self._rdp(path_astar)
        self.get_logger().info(f"Pruned: {len(path_pruned)} waypoints")
        self._publish_grid_path(path_pruned, self.marker_pub_pruned, ns="pruned", color=(1.0, 1.0, 0.0))

        path_densify = self._densify_path(path_pruned)
        self.get_logger().info(f"Pruned: {len(path_densify)} waypoints")
        self._publish_grid_path(path_densify, self.marker_pub_densify, ns="densify", color=(1.0, 1.0, 0.5))


        waypoints = self._path_to_timed_waypoints(path_densify) #path_pruned
        path_snaped, _, _ = self.waypoint_to_trajectory(waypoints) #tmp

        self.get_logger().info(f"Trajectory generated: {len(path_snaped.position)} samples")
        self._publish_trajectory(path_snaped, self.marker_pub_snaped, ns="snaped", color=(0.0, 1.0, 0.0))


        max_retries = 50
        path_verified, t_samples, acceleration = self.waypoint_to_trajectory(waypoints)
        for attempt in range(max_retries):
            is_repaired, waypoints = self._check_and_repair(path_verified, waypoints, path_astar)

            if is_repaired:
                pass
            else:
                self.get_logger().error("Repair failed (Segment too small). Aborting planner.")
                return

            try:
                res = self.waypoint_to_trajectory(waypoints)
                if res[0] is None: raise ValueError("Solver failed")
                path_verified, t_samples, acceleration = res
            except ValueError:
                self.get_logger().error("MinSnap crashed during repair loop.")
                return

            if self._is_trajectory_safe(path_verified):
                self.get_logger().info(f"Trajectory valid after {attempt} repairs.")
                break

        self.get_logger().info(f"Trajectory generated: {len(path_verified.position)} samples")
        self._publish_trajectory(path_verified,  self.marker_pub_verif, ns="verif", color=(0.0, 1.0, 1.0))
        min_snap_length = self._min_snap_lenght(path_verified)
        if astar_lenght < 0.1:
            ratio = 1.0
        else:
            ratio = min_snap_length / astar_lenght

        self.get_logger().info(f"Check Traj: Len A*={astar_lenght:.2f}m, Len Snap={min_snap_length:.2f}m, Ratio={ratio:.2f}")

        if ratio > 2.0:
            self.get_logger().error(f"Trajectoire rejetée. Ratio: {ratio:.2f}")
            return
        self.publish_final_trajectory(path_verified,t_samples, acceleration)

    def _is_trajectory_safe(self, quad_traj):
        for pos in quad_traj.position:
            grid_idx = self.map_reader.world_to_grid(pos)
            if self.map_reader.is_occupied_minsnap(grid_idx):
                return False
        return True

    def _aster_lenght(self, path):
        astar_length = 0.0
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            astar_length += np.linalg.norm(p2 - p1)
        return astar_length
    def _min_snap_lenght(self, quad_traj):
        positions = quad_traj.position

        diffs = np.diff(positions, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        minsnap_length = np.sum(dists)

        return minsnap_length

    def _check_and_repair(self, quad_traj, current_waypoints: List[Waypoint], raw_astar_path: List[GridIdx]) -> Tuple[bool, List[Waypoint]]:
        positions = quad_traj.position
        times = np.linspace(0, current_waypoints[-1].time, len(positions))

        collision_idx = -1

        #detection de collision
        for i, pos in enumerate(positions):
            grid_idx = self.map_reader.world_to_grid(pos)
            if self.map_reader.is_occupied_minsnap(grid_idx):
                collision_idx = i
                break

        if collision_idx == -1:
            return True, current_waypoints

        #on cherche le point
        collision_time = times[collision_idx]
        collision_pos = positions[collision_idx]

        self.get_logger().warn(f"Collision at t={collision_time:.2f}s. Pos={collision_pos}")

        # cherche le point A* le plus proche
        best_dist = float('inf')
        safest_grid_idx = None
        for idx in raw_astar_path:
            pt_world = np.array(self.map_reader.grid_to_world(idx))
            dist = np.linalg.norm(pt_world - collision_pos)
            if dist < best_dist:
                best_dist = dist
                safest_grid_idx = idx

        if safest_grid_idx is None:
            return False, current_waypoints

        safe_pos = np.array(self.map_reader.grid_to_world(safest_grid_idx))

        # 4. STRATEGIE D'INSERTION INTELLIGENTE (La clé est ici)

        new_waypoints = []
        inserted = False
        min_dt = 0.2
        for i, wp in enumerate(current_waypoints):

            if not inserted and wp.time > collision_time:

                prev_wp = current_waypoints[i-1] if i > 0 else None

                dt_before = collision_time - prev_wp.time if prev_wp else min_dt + 1
                dt_after = wp.time - collision_time

                if dt_before < min_dt or dt_after < min_dt:
                    self.get_logger().warn("Cannot repair: Segment too small for insertion. Skipping repair.")
                    #todo decaler les temps pour l insertion du point
                    return False, current_waypoints

                repair_wp = Waypoint(time=collision_time, position=safe_pos)

                new_waypoints.append(repair_wp)
                inserted = True

            new_waypoints.append(wp)

        if not inserted:
            pass

        return False, new_waypoints

    def _path_to_timed_waypoints(self, grid_path: List[GridIdx]) -> List[Waypoint]:
        # Vitesse max cible (m/s)
        v_max = self.get_parameter("avg_speed").value
        # Si trop faible (<0.5), risque de boucles. Si trop fort (>2.0), risque de décrochage.
        a_max = 1.0

        raw_points = [self.map_reader.grid_to_world(idx) for idx in grid_path]
        waypoints = []
        current_time = 0.0

        for i, pos_list in enumerate(raw_points):
            pos = np.array(pos_list)

            if i > 0:
                prev_pos = np.array(raw_points[i-1])
                dist = np.linalg.norm(pos - prev_pos)


                t_accel = v_max / a_max
                dist_accel = 0.5 * a_max * (t_accel ** 2)

                if dist < 2 * dist_accel:
                    t_kinematic = 2 * math.sqrt(dist / a_max)
                else:

                    t_cruise_part = (dist - 2 * dist_accel) / v_max
                    t_kinematic = 2 * t_accel + t_cruise_part


                t_cruise = dist / max(v_max, 0.1)

                angle_factor = 0.0

                if i > 1:
                    prev_prev = np.array(raw_points[i-2])
                    v1 = prev_pos - prev_prev
                    v2 = pos - prev_pos

                    n1 = np.linalg.norm(v1)
                    n2 = np.linalg.norm(v2)

                    if n1 > 1e-3 and n2 > 1e-3:
                        dot = np.dot(v1/n1, v2/n2)
                        dot = max(-1.0, min(1.0, dot))
                        angle = math.acos(dot)

                        angle_factor = angle / (math.pi / 2.0)
                        angle_factor = max(0.0, min(1.0, angle_factor))

                segment_time = t_cruise + (t_kinematic - t_cruise) * angle_factor

                min_phys_time = math.sqrt(2 * dist / a_max) * 0.8
                segment_time = max(segment_time, min_phys_time)

                segment_time = max(segment_time, 0.1)

                current_time += segment_time

            vel = None
            acc = None

            if i == 0 or i == len(raw_points) - 1:
                vel = [0, 0, 0]
                acc = [0, 0, 0]

            if vel is not None:
                wp = Waypoint(time=current_time, position=pos, velocity=vel, acceleration=acc)
            else:
                wp = Waypoint(time=current_time, position=pos)

            waypoints.append(wp)

        return waypoints

    def waypoint_to_trajectory(self, waypoints):

        try:
            traj_poly = generate_trajectory(waypoints, degree=7, idx_minimized_orders=4)
        except ValueError as e:
            self.get_logger().error(f"MinSnap Solver Failed: {e}")
            return None, None, None

        t_total = waypoints[-1].time
        t_samples = np.arange(0, t_total, self.get_parameter("sample_rate").value)

        quad_traj = compute_quadrotor_trajectory(
            traj_poly,
            t_samples,
            vehicle_mass=self.get_parameter("drone_mass").value
        )

        derivatives = compute_trajectory_derivatives(traj_poly, t_samples, 4)
        acceleration = derivatives[2]

        return quad_traj, t_samples, acceleration

    def _publish_grid_path(self, grid_path, marker_pub, ns, color):
        world_points = [self.map_reader.grid_to_world(idx) for idx in grid_path]
        self._publish_visuals(world_points, marker_pub, ns, color)

    def _publish_trajectory(self, quad_traj, marker_pub, ns, color):
        # quad_traj.position est un numpy array (N, 3)
        points = quad_traj.position.tolist()
        self._publish_visuals(points, marker_pub, ns, color)

    def _publish_visuals(self, points_xyz, marker_pub, ns, color):
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

        # path_pub.publish(msg_path)

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