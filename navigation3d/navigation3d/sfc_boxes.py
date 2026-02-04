# #!/usr/bin/env python3
# import math
# from typing import List, Set, Tuple
#
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
#
# from nav_msgs.msg import Path
# from visualization_msgs.msg import Marker, MarkerArray
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2
#
#
# GridIdx = Tuple[int, int, int]
#
#
# class SFCBoxes(Node):
#     """
#     MVP SFC:
#       - Subscribes to planned_path (nav_msgs/Path)
#       - Builds occupied voxel set from /octomap_point_cloud_centers
#       - For each waypoint, creates an axis-aligned box and expands it until collision
#       - Publishes MarkerArray (CUBEs) for RViz
#     """
#
#     def __init__(self) -> None:
#         super().__init__("sfc_generator")
#
#         self.declare_parameter("frame_id", "map")
#         self.declare_parameter("occupied_cloud_topic", "/octomap_point_cloud_centers")
#         self.declare_parameter("grid_res", 0.20)
#         self.declare_parameter("inflation", 0.30)
#
#         # Box growth params (tweak)
#         self.declare_parameter("min_half", [0.4, 0.4, 0.25])  # initial half extents
#         self.declare_parameter("max_half", [1.5, 1.5, 0.8])   # cap
#         self.declare_parameter("grow_step", 0.10)            # meters
#         self.declare_parameter("path_stride", 3)             # take 1 waypoint out of N
#
#         self.frame_id = self.get_parameter("frame_id").value
#
#         # Ce profil est n�cessaire pour recevoir les messages "Latched" (Carte ET Chemin)
#         qos_latched = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
#         )
#
#         # 1. Abonnement � la carte (Avec le QoS Latched)
#         self.occ_sub = self.create_subscription(
#             PointCloud2,
#             self.get_parameter("occupied_cloud_topic").value,
#             self._on_cloud,
#             qos_latched,  # <--- Utilisation du profil ici
#         )
#
#         # 2. Abonnement au chemin (Avec le QoS Latched aussi, car le planner est maintenant latched)
#         self.path_sub = self.create_subscription(
#             Path,
#             "planned_path",
#             self._on_path,
#             qos_latched  # <--- Remplacement de '10' par 'qos_latched'
#         )
#
#         # Pour le publisher des markers, on peut aussi le mettre en Latched pour que Rviz le voie bien
#         self.pub = self.create_publisher(MarkerArray, "sfc_markers", qos_latched)
#
#
#
#         self.occupied: Set[GridIdx] = set()
#         self.occupied_inflated: Set[GridIdx] = set()
#         self.map_ready = False
#         self.last_path: Path = None
#
#         self.get_logger().info("SFC generator started.")
#
#     def _on_cloud(self, msg: PointCloud2) -> None:
#         grid_res = float(self.get_parameter("grid_res").value)
#         occ = set()
#         for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#             occ.add((math.floor(x / grid_res), math.floor(y / grid_res), math.floor(z / grid_res)))
#
#         self.occupied = occ
#         self.occupied_inflated = self._inflate(occ)
#         self.map_ready = True
#         self.get_logger().info(f"SFC map ready. Voxels={len(self.occupied)} (inflated={len(self.occupied_inflated)})")
#
#         if self.last_path is not None:
#             self._build_and_publish(self.last_path)
#
#     def _inflate(self, occ: Set[GridIdx]) -> Set[GridIdx]:
#         grid_res = float(self.get_parameter("grid_res").value)
#         inflation = float(self.get_parameter("inflation").value)
#         if inflation <= 1e-6:
#             return set(occ)
#
#         r = int(math.ceil(inflation / grid_res))
#         offsets = [(dx, dy, dz)
#                    for dx in range(-r, r+1)
#                    for dy in range(-r, r+1)
#                    for dz in range(-r, r+1)
#                    if (dx*dx + dy*dy + dz*dz) <= (r*r)]
#
#         inflated = set()
#         for (ix, iy, iz) in occ:
#             for dx, dy, dz in offsets:
#                 inflated.add((ix + dx, iy + dy, iz + dz))
#         return inflated
#
#     def _on_path(self, msg: Path) -> None:
#         self.last_path = msg
#         if self.map_ready:
#             self._build_and_publish(msg)
#
#     def _collides_box(self, cx: float, cy: float, cz: float, hx: float, hy: float, hz: float) -> bool:
#         """Checks if any inflated occupied voxel is inside the AABB box."""
#         grid_res = float(self.get_parameter("grid_res").value)
#
#         minx = math.floor((cx - hx) / grid_res)
#         maxx = math.floor((cx + hx) / grid_res)
#         miny = math.floor((cy - hy) / grid_res)
#         maxy = math.floor((cy + hy) / grid_res)
#         minz = math.floor((cz - hz) / grid_res)
#         maxz = math.floor((cz + hz) / grid_res)
#
#         for ix in range(minx, maxx + 1):
#             for iy in range(miny, maxy + 1):
#                 for iz in range(minz, maxz + 1):
#                     if (ix, iy, iz) in self.occupied_inflated:
#                         return True
#         return False
#
#     def _grow_box(self, cx: float, cy: float, cz: float) -> Tuple[float, float, float]:
#         min_half = list(self.get_parameter("min_half").value)
#         max_half = list(self.get_parameter("max_half").value)
#         step = float(self.get_parameter("grow_step").value)
#
#         hx, hy, hz = min_half
#
#         # grow independently per axis (simple but effective MVP)
#         for axis in range(3):
#             while True:
#                 nhx, nhy, nhz = hx, hy, hz
#                 if axis == 0:
#                     nhx = min(hx + step, max_half[0])
#                 elif axis == 1:
#                     nhy = min(hy + step, max_half[1])
#                 else:
#                     nhz = min(hz + step, max_half[2])
#
#                 if (nhx, nhy, nhz) == (hx, hy, hz):
#                     break
#
#                 if self._collides_box(cx, cy, cz, nhx, nhy, nhz):
#                     break
#
#                 hx, hy, hz = nhx, nhy, nhz
#
#         return hx, hy, hz
#
#     def _build_and_publish(self, path: Path) -> None:
#         stride = int(self.get_parameter("path_stride").value)
#         if not path.poses:
#             return
#
#         ma = MarkerArray()
#         now = self.get_clock().now().to_msg()
#
#         # Clear previous markers (optional technique: publish DELETEALL)
#         delete_all = Marker()
#         delete_all.action = Marker.DELETEALL
#         ma.markers.append(delete_all)
#
#         mid = 0
#         for i in range(0, len(path.poses), max(1, stride)):
#             ps = path.poses[i]
#             cx = ps.pose.position.x
#             cy = ps.pose.position.y
#             cz = ps.pose.position.z
#
#             hx, hy, hz = self._grow_box(cx, cy, cz)
#
#             m = Marker()
#             m.header.frame_id = self.frame_id
#             m.header.stamp = now
#             m.ns = "sfc_boxes"
#             m.id = mid
#             mid += 1
#
#             m.type = Marker.CUBE
#             m.action = Marker.ADD
#             m.pose.position.x = float(cx)
#             m.pose.position.y = float(cy)
#             m.pose.position.z = float(cz)
#             m.pose.orientation.w = 1.0
#
#             m.scale.x = float(2.0 * hx)
#             m.scale.y = float(2.0 * hy)
#             m.scale.z = float(2.0 * hz)
#
#             m.color.a = 0.25
#             m.color.g = 1.0  # green translucent
#
#             ma.markers.append(m)
#
#         self.pub.publish(ma)
#         self.get_logger().info(f"Published {mid} SFC boxes.")
#
#
# def main():
#     rclpy.init()
#     node = SFCBoxes()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()
