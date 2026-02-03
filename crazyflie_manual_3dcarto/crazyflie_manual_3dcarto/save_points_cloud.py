#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Bool

class SavePCDNode(Node):
    def __init__(self):
        super().__init__('save_cloud_node')
        self.points = []

        # Subscriber to the PointCloud2 topic
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/my_3d_cloud', self.cloud_callback, 10
        )

        # Subscriber to the save trigger topic
        self.save_sub = self.create_subscription(
            Bool, '/save_cloud', self.save_callback, 10
        )

        self.get_logger().info("Node ready. Press 1 (via /save_cloud) to save as .pcd.")

    def cloud_callback(self, msg):
        for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            self.points.append([p[0], p[1], p[2]])

    def save_callback(self, msg):
        if msg.data:
            self.save_pcd()

    def save_pcd(self):
        if not self.points:
            self.get_logger().warn("No points to save.")
            return

        arr = np.array(self.points, dtype=np.float32)
        n = arr.shape[0]
        filename = "map.pcd"

        header = f"""# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA ascii
"""

        with open(filename, "w") as f:
            f.write(header)
            for p in arr:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")

        self.get_logger().info(f"Cloud saved to {filename} ({n} points).")

def main(args=None):
    rclpy.init(args=args)
    node = SavePCDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
