#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs 
import math

class ScanToPCDNode(Node):
    def __init__(self):
        super().__init__('scan_to_pcd_node')

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber LaserScan
        self.scan_sub = self.create_subscription(LaserScan,'/crazyflie/scan',self.scan_callback,10)

        # Publisher PointCloud2
        self.cloud_pub = self.create_publisher(PointCloud2, '/my_3d_cloud', 10)

        self.map_frame = 'map'
        self.get_logger().info('Node ScanToPCD started.')

    def scan_callback(self, msg: LaserScan):
        points_world = []
        try:
            source_frame_id = 'crazyflie/base_footprint'
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Unable to get TF transform: {e}')
            return

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            point_sensor_frame = PointStamped()
            point_sensor_frame.header = msg.header
            point_sensor_frame.point.x = r * math.cos(angle)
            point_sensor_frame.point.y = r * math.sin(angle)
            point_sensor_frame.point.z = 0.0

            point_world_frame = tf2_geometry_msgs.do_transform_point(point_sensor_frame, transform)
            points_world.append([
                point_world_frame.point.x,
                point_world_frame.point.y,
                point_world_frame.point.z
            ])

        if not points_world:
            return

        cloud_header = msg.header
        cloud_header.frame_id = self.map_frame

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        cloud_msg = pc2.create_cloud(cloud_header, fields, points_world)
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPCDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual stop.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
