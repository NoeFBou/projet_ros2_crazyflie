#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs 
import math
import pcl
import numpy as np
import atexit
#test
class CloudAssemblerNode(Node):
    def __init__(self):
        super().__init__('cloud_assembler_node')

        # transformations TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # topic du multiranger
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/crazyflie/scan',
            self.scan_callback,
            10)

        # publisher pour notre nuage de points
        self.cloud_pub = self.create_publisher(PointCloud2, '/my_3d_cloud', 10)

        # frame de référence ou odom pour avoir la pos
        self.map_frame = 'map'
        self.all_points = []
        atexit.register(self.save_to_pcd)
        self.get_logger().info('Nœud d\'assemblage de nuage de points démarré.')

    def scan_callback(self, msg: LaserScan):
        points_world = [] 
        try:

            source_frame_id = 'crazyflie/base_footprint' 

    # transformation entre la pos de la map  et le la pos du drone
            transform = self.tf_buffer.lookup_transform(
        	self.map_frame,             # la map
        	source_frame_id,            # le drone
        	msg.header.stamp,           
       		timeout=rclpy.duration.Duration(seconds=0.1) 
	    )

        except Exception as e:
            self.get_logger().warn(f'Impossible d\'obtenir la transformation TF: {e}')
            return

        # conversion du point du laserscan en point 3D
        for i, range_reading in enumerate(msg.ranges):
            if math.isinf(range_reading) or math.isnan(range_reading):
                continue


            angle = msg.angle_min + i * msg.angle_increment


            point_sensor_frame = PointStamped()
            point_sensor_frame.header = msg.header
            point_sensor_frame.point.x = range_reading * math.cos(angle)
            point_sensor_frame.point.y = range_reading * math.sin(angle)
            point_sensor_frame.point.z = 0.0 #

            # transformation du point du repère du capteur à la map
            point_world_frame = tf2_geometry_msgs.do_transform_point(point_sensor_frame, transform)


            points_world.append([
                point_world_frame.point.x,
                point_world_frame.point.y,
                point_world_frame.point.z
            ])

        if not points_world:
            return

        self.all_points.extend(points_world)

        # message pour le topic PointCloud2
        cloud_header = msg.header
        cloud_header.frame_id = self.map_frame 

        # Créer le nuage de points à partir de la liste de points
        fields = [pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                  pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                  pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)]

        cloud_msg = pc2.create_cloud(cloud_header, fields, points_world)


        self.cloud_pub.publish(cloud_msg)

    def save_to_pcd(self):
        self.get_logger().info('Fermeture... Sauvegarde du nuage de points accumulé.')

        if not self.all_points:
            self.get_logger().warn('Aucun point accumulé. Aucun fichier .pcd ne sera créé.')
            return

        # Convertir notre liste python en un nuage de points PCL
        cloud_array = np.array(self.all_points, dtype=np.float32)

        pcd_cloud = pcl.PointCloud()
        pcd_cloud.from_array(cloud_array)

        try:
            filename = 'my_dense_cloud.pcd'
            pcl.save(pcd_cloud, filename)
            self.get_logger().info(f'Nuage de points dense sauvegardé dans {filename}!')
            self.get_logger().info(f'Total des points: {len(self.all_points)}')
        except Exception as e:
            self.get_logger().error(f'Impossible de sauvegarder le fichier .pcd: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CloudAssemblerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt manuel demandé.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
