#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from octomap_msgs.srv import GetOctomap 
import subprocess 
import struct
from std_msgs.msg import Header
from std_msgs.msg import Bool

class SavePCDNode(Node):
    def __init__(self):
        super().__init__('save_cloud_node')
        self.points = []

        # Publisher for the fake floor (to ensure octomap fills the ground)
        self.floor_pub = self.create_publisher(PointCloud2, '/my_3d_cloud', 10)

        # Subscriber to the PointCloud2 topic
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/my_3d_cloud', self.cloud_callback, 10
        )

        # Subscriber to the save trigger topic
        self.save_sub = self.create_subscription(
            Bool, '/save_cloud', self.save_callback, 10
        )

        self.get_logger().info("Node ready. Press 1 (via /save_cloud) to save as .pcd.")

        self.publish_fake_floor()

    def cloud_callback(self, msg):
        for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            self.points.append([p[0], p[1], p[2]])

    def publish_fake_floor(self):
        """
        Create and publish a flat floor point cloud to ensure octomap fills the ground level.
        """
        self.get_logger().info("Generating a fake floor...")
        
        points = []
        # Configure floor size (e.g., from -5m to +5m)
        min_x, max_x = -5.0, 5.0
        min_y, max_y = -5.0, 5.0
        step = 0.1 # Point density (10cm)
        z_floor = 0.0 # Floor height
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                points.append([x, y, z_floor])
                y += step
            x += step

        # Create the PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        pc2_msg = pc2.create_cloud_xyz32(header, points)
        
        # Publish the floor so octomap_server adds it to its map
        self.floor_pub.publish(pc2_msg)
        
        # Add it to our local list for the PCD file
        for p in points:
            self.points.append(p)
            
        self.get_logger().info("Fake floor sent to Octomap and added to PCD buffer")
    def save_callback(self, msg):
        if msg.data:
            self.save_pcd()
            self.save_octomap()

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

    

    def save_octomap(self):
        map_filename = "map.bt"
        self.get_logger().info(f"Attempting to save Octomap to {map_filename}...")
        
        try:
            cmd = [
                "ros2", 
                "run", 
                "octomap_server", 
                "octomap_saver_node", 
                "--ros-args", 
                "-p", 
                f"octomap_path:={map_filename}"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info(f"Success: Octomap saved to {map_filename}")
            else:
                self.get_logger().error(f"Octomap saver error: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Exception during Octomap saving: {e}")
       
def main(args=None):
    rclpy.init(args=args)
    node = SavePCDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
