#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml
import time
import math
import sys
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path              
from geometry_msgs.msg import PoseStamped

class PositionController(Node):
    def __init__(self, waypoint_file):
        super().__init__('position_controller')
        self.get_logger().info("Initialisation du contrôleur de position...")



        # --- Paramètres du Contrôleur (Gains P) ---
        self.P_GAIN_XY = 0.5
        self.P_GAIN_Z = 0.8
        self.P_GAIN_YAW = 0.6

        # --- Limites de Vitesse  ---
        self.MAX_SPEED_XY = 0.3
        self.MAX_SPEED_Z = 0.6
        self.MAX_SPEED_YAW = 0.5

        # --- Seuil de Précision ---
        self.WAYPOINT_THRESHOLD_POS = 0.3 # 30cm
        self.WAYPOINT_THRESHOLD_YAW = 0.2 #  ~12 deg

        self.PATH_DISTANCE_THRESHOLD = 0.1
        # --- Variables d'état ---
        self.current_pos = None     
        self.current_yaw = 0.0      
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_waypoint_index = 0
        self.waypoint_reached_time = None
        self.odom_frame_id = None
        self.path_msg = Path()
        # --- ROS 2 Publishers & Subscribers ---
        self.get_logger().info("Configuration des topics...")
        
        # S'abonne à l'odométrie pour savoir où est le drone
        self.odom_sub = self.create_subscription(
            Odometry,
            '/crazyflie/odom',  
            self.odom_callback,
            10)
        
        # Publie les commandes de vélocité
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/crazyflie/cmd_vel', # Le topic de contrôle
            10)
            
        self.marker_pub = self.create_publisher(
            Marker,
            '/crazyflie/visualization_markers',  # Topic pour les marqueurs
            10)
        self.path_pub = self.create_publisher(
            Path,
            '/crazyflie/path',
            10)
        # Boucle de contrôle principale (tourne à 20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Contrôleur en attente de la première odométrie...")
        

    def load_waypoints(self, file_path):
        self.get_logger().info(f"Chargement des waypoints depuis {file_path}")
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                return data['waypoints']
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement du fichier YAML: {e}")
            rclpy.shutdown()
            return []

    def odom_callback(self, msg):
        # Met à jour la position à chaque message d'odométrie
        self.current_pos = msg.pose.pose.position
        if self.odom_frame_id is None:
            self.odom_frame_id = msg.header.frame_id
            self.path_msg.header.frame_id = self.odom_frame_id

        
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        marker = Marker()
        marker.header = msg.header  # Utilise le header de l'odométrie
        marker.ns = "crazyflie_viz"
        marker.id = 0  # ID 0 pour la position actuelle
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = self.current_pos
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.8  
        marker.color.r = 0.0
        marker.color.g = 1.0  
        marker.color.b = 0.0
        marker.lifetime = Duration(seconds=1.0).to_msg() 
        add_to_path = False
        if not self.path_msg.poses:

            add_to_path = True
        else:

            last_pos = self.path_msg.poses[-1].pose.position
            dist = math.sqrt(
                (self.current_pos.x - last_pos.x)**2 +
                (self.current_pos.y - last_pos.y)**2 +
                (self.current_pos.z - last_pos.z)**2
            )

            if dist > self.PATH_DISTANCE_THRESHOLD:
                add_to_path = True
        
        if add_to_path:

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose 
            

            self.path_msg.poses.append(pose_stamped)
            

            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publie la trace complète mise à jour
            self.path_pub.publish(self.path_msg)
        self.marker_pub.publish(marker)
        
        
    def control_loop(self):
        # N'exécute rien tant que nous n'avons pas reçu de données d'odométrie
        if self.current_pos is None or not self.waypoints or self.odom_frame_id is None:
            return

        # Vérifie si la mission est terminée
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Mission terminée. Maintien de la position et atterrissage lent.")
            self.stop_and_hover(land=True)
            return

        # --- 1. Obtenir la cible actuelle ---
        target_wp = self.waypoints[self.current_waypoint_index]
        target_pos = {'x': target_wp['x'], 'y': target_wp['y'], 'z': target_wp['z']}
        target_yaw = target_wp['yaw']
        hold_time = target_wp['hold_time']
        
        target_marker = Marker()
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.header.frame_id = self.odom_frame_id
        target_marker.ns = "crazyflie_viz"
        target_marker.id = 1  # ID 1 pour la destination
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(target_pos['x'])
        target_marker.pose.position.y = float(target_pos['y'])
        target_marker.pose.position.z = float(target_pos['z'])
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.2
        target_marker.scale.y = 0.2
        target_marker.scale.z = 0.2
        target_marker.color.a = 0.8
        target_marker.color.r = 1.0  
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.lifetime = Duration(seconds=1.0).to_msg()
        
        self.marker_pub.publish(target_marker)

        # --- 2. Calculer l'erreur ---
        error_pos_x = target_pos['x'] - self.current_pos.x
        error_pos_y = target_pos['y'] - self.current_pos.y
        error_pos_z = target_pos['z'] - self.current_pos.z
        error_yaw = self.normalize_angle(target_yaw - self.current_yaw)
        
        dist_error = math.sqrt(error_pos_x**2 + error_pos_y**2 + error_pos_z**2)
        yaw_error_abs = abs(error_yaw)
        
        cmd_vel_msg = Twist() 



        # ÉTAT 1 : Déplacement (Si on est trop loin)
        if dist_error > self.WAYPOINT_THRESHOLD_POS:
            self.waypoint_reached_time = None 
            
            # Log verbeux
            if self.get_clock().now().nanoseconds % 1e9 < 50e6:
                self.get_logger().info(f"Déplacement vers WP {self.current_waypoint_index}: [ErrDist: {dist_error:.2f}m]")

            # Contrôle P pour la position (identique)
            cmd_vel_msg.linear.x = self.P_GAIN_XY * error_pos_x
            cmd_vel_msg.linear.y = self.P_GAIN_XY * error_pos_y
            cmd_vel_msg.linear.z = self.P_GAIN_Z * error_pos_z
            
            # Rotation continue (scan)
            cmd_vel_msg.angular.z = self.MAX_SPEED_YAW * 0.5
            


        # ÉTAT 2 : Rotation (Si on est à la bonne position, mais mal orienté)
        elif yaw_error_abs > self.WAYPOINT_THRESHOLD_YAW:
            self.waypoint_reached_time = None # On n'est pas prêt, reset le timer

            if self.get_clock().now().nanoseconds % 1e9 < 50e6: # Log 1x par seconde
                self.get_logger().info(f"Ajustement du cap pour WP {self.current_waypoint_index}: [ErrYaw: {yaw_error_abs:.2f}rad]")
            
            # On ne bouge pas (linear.x/y/z = 0 par défaut)
            # On pivote SEULEMENT
            cmd_vel_msg.angular.z = self.P_GAIN_YAW * error_yaw

        # ÉTAT 3 : Maintien (On est à la bonne position ET au bon cap)
        else:
            # On arrête tout mouvement (la commande est déjà vide)
            # Démarrer le chrono si ce n'est pas fait
            if self.waypoint_reached_time is None:
                self.get_logger().info(f"Waypoint {self.current_waypoint_index} atteint. Maintien pendant {hold_time}s...")
                self.waypoint_reached_time = self.get_clock().now()
            
            # Vérifier si le temps d'attente est écoulé
            elapsed_time = (self.get_clock().now() - self.waypoint_reached_time).nanoseconds / 1e9
            
            if elapsed_time >= hold_time:
                self.get_logger().info(f"Temps d'attente terminé. Passage au waypoint {self.current_waypoint_index + 1}.")
                self.current_waypoint_index += 1
                self.waypoint_reached_time = None

        # --- 4. Saturation (Limiter les vitesses) ---
        speed_xy = math.sqrt(cmd_vel_msg.linear.x**2 + cmd_vel_msg.linear.y**2)
        if speed_xy > self.MAX_SPEED_XY:
            scale = self.MAX_SPEED_XY / speed_xy
            cmd_vel_msg.linear.x *= scale
            cmd_vel_msg.linear.y *= scale

        cmd_vel_msg.linear.z = max(min(cmd_vel_msg.linear.z, self.MAX_SPEED_Z), -self.MAX_SPEED_Z)
        cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, self.MAX_SPEED_YAW), -self.MAX_SPEED_YAW)

        # --- 5. Publier la commande ---
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def stop_and_hover(self, land=False):
        # Envoie une commande Twist vide pour arrêter le mouvement
        cmd_vel_msg = Twist()
        if land:
            # Commande d'atterrissage (descendre doucement)
            cmd_vel_msg.linear.z = -0.2
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def normalize_angle(self, angle):
        # Normalise un angle entre -pi et pi
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: python3 position_controller.py <chemin_vers_yaml>")
        return

    waypoint_file_path = sys.argv[1]
    
    controller_node = PositionController(waypoint_file_path)
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info("Arrêt du contrôleur (Ctrl+C).")
    finally:
        controller_node.stop_and_hover(land=True)
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
