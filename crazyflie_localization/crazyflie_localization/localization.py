import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion, TransformStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import open3d as o3d
import tf_transformations
import tf2_ros
import os

class CrazyflieLocalization3D(Node):
    def __init__(self):
        super().__init__('crazyflie_pf_localization_3d')

        # --- CONFIGURATION VIA PARAMÈTRES ---
        self.declare_parameter('ply_path', '')
        self.ply_path = self.get_parameter('ply_path').get_parameter_value().string_value

        # Paramètres pour la localisation globale 3D
        self.num_particles = 800  # Augmenté pour couvrir X, Y et Z simultanément
        self.sensor_sigma = 0.05   # Très précis pour distinguer les hauteurs
        self.map_ready = False

        # --- INITIALISATION 4D (X, Y, Z, Yaw) ---
        self.particles = np.zeros((self.num_particles, 4))
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.init_particles()
        self.load_fixed_map()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.scan_sub = self.create_subscription(LaserScan, '/crazyflie/scan', self.scan_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.parts_pub = self.create_publisher(PoseArray, '/particles', 10)

        self.get_logger().info("Localisation 3D Globale active (Recherche X, Y, Z).")

    def load_fixed_map(self):
        if not self.ply_path or not os.path.exists(self.ply_path):
            self.get_logger().error(f"Fichier PLY introuvable : {self.ply_path}")
            return
        mesh = o3d.io.read_triangle_mesh(self.ply_path)
        t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(t_mesh)
        self.map_ready = True

    def init_particles(self):
        """Initialisation globale : on cherche partout dans la salle."""
        # X et Y : entre -2m et 2m (ajuste selon ta salle)
        self.particles[:, 0] = np.random.uniform(-2.0, 2.0, self.num_particles)
        self.particles[:, 1] = np.random.uniform(-2.0, 2.0, self.num_particles)
        # Z : du sol au plafond (0m à 2m) pour détecter les cubes/tables
        self.particles[:, 2] = np.random.uniform(0.0, 2.0, self.num_particles)
        # Orientation : 0 à 360 degrés
        self.particles[:, 3] = np.random.uniform(0, 2 * np.pi, self.num_particles)

    def scan_callback(self, msg):
        if not self.map_ready: return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid): return

        meas_dists = ranges[valid]
        sensor_angles = angles[valid]

        # --- 1. ÉTAPE DE PRÉDICTION (SANS ODOMÉTRIE) ---
        # On ajoute un fort bruit d'exploration pour que le nuage "cherche" la position
        self.particles[:, 0:2] += np.random.normal(0, 0.08, (self.num_particles, 2)) # X, Y
        self.particles[:, 2] += np.random.normal(0, 0.15, self.num_particles)       # Z (Recherche de hauteur)
        self.particles[:, 3] += np.random.normal(0, 0.1, self.num_particles)        # Yaw

        # Limites physiques
        self.particles[:, 2] = np.clip(self.particles[:, 2], 0.0, 2.5)

        # --- 2. ÉTAPE DE MISE À JOUR (RAYCASTING) ---
        total_weights = np.ones(self.num_particles)
        for i, angle in enumerate(sensor_angles):
            ray_angles = self.particles[:, 3] + angle
            origins = self.particles[:, 0:3].copy()

            dirs = np.zeros((self.num_particles, 3))
            dirs[:, 0], dirs[:, 1] = np.cos(ray_angles), np.sin(ray_angles)

            rays = np.concatenate([origins, dirs], axis=1).astype(np.float32)
            ans = self.scene.cast_rays(o3d.core.Tensor(rays))
            sim_dists = ans['t_hit'].numpy()

            diff = sim_dists - meas_dists[i]
            # Les particules à la mauvaise hauteur (Z) auront un poids très faible
            total_weights *= np.exp(-(diff**2) / (2 * self.sensor_sigma**2))

        self.weights *= total_weights
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

        # --- 3. RÉÉCHANTILLONNAGE SYSTÉMATIQUE ---
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

        self.publish_results()

    def publish_results(self):
        # Calcul de la position estimée (Moyenne pondérée)
        est_x = np.sum(self.particles[:, 0] * self.weights)
        est_y = np.sum(self.particles[:, 1] * self.weights)
        est_z = np.sum(self.particles[:, 2] * self.weights)
        est_yaw = np.arctan2(np.sum(np.sin(self.particles[:, 3]) * self.weights),
                             np.sum(np.cos(self.particles[:, 3]) * self.weights))

        timestamp = self.get_clock().now().to_msg()

        # Pose unique
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = est_x, est_y, est_z
        q = tf_transformations.quaternion_from_euler(0, 0, est_yaw)
        pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pose_pub.publish(pose_msg)

        # TF map -> base_link (Mise à jour directe de la position 3D)
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'crazyflie/base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = est_x, est_y, est_z
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Particules 3D pour RViz
        parts_msg = PoseArray()
        parts_msg.header = pose_msg.header
        for p in self.particles:
            pp = Pose()
            pp.position.x, pp.position.y, pp.position.z = p[0], p[1], p[2]
            pq = tf_transformations.quaternion_from_euler(0, 0, p[3])
            pp.orientation = Quaternion(x=pq[0], y=pq[1], z=pq[2], w=pq[3])
            parts_msg.poses.append(pp)
        self.parts_pub.publish(parts_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieLocalization3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()