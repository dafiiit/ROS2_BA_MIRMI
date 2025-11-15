#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped, Pose, Transform
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, translation_matrix, quaternion_matrix, inverse_matrix

def transform_to_matrix(transform: Transform):
    trans = [transform.translation.x, transform.translation.y, transform.translation.z]
    rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    T = translation_matrix(trans)
    R = quaternion_matrix(rot)
    return T @ R
    
# Hilfsfunktion, um eine geometry_msgs/Pose in eine 4x4-Matrix umzuwandeln
def pose_to_matrix(pose: Pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    T = translation_matrix(trans)
    R = quaternion_matrix(rot)
    return T @ R

# Definiere die Zustände
class DockingState(Enum):
    SEARCHING = 1          # 1. Dreht sich, bis Tag gesehen wird
    LOCALIZING = 2         # 3. Bestimmt Welt-Position
    GOTO_WAYPOINT_1 = 3    # 4. Fährt auf 5m Abstand
    GOTO_WAYPOINT_2 = 4    # 5. Fährt Radius zur Öffnung
    GOTO_WAYPOINT_3 = 5    # 6. Dreht sich zur Hütte
    DOCKING = 6            # 7. Fährt in die Hütte
    FINAL_STOP = 7         # Ziel erreicht

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Zustand der State Machine
        self.state = DockingState.SEARCHING
        
        # Konstanten für die Navigation
        self.HUT_CENTER = np.array([5.0, 2.0]) #
        self.TARGET_RADIUS = 5.0
        # Zielkoordinate 1: Vor der Öffnung (Hütte ist 3m lang, Zentrum bei 5, Öffnung bei 5+1.5=6.5)
        self.WAYPOINT_OPENING = np.array([6.5 + 0.5, 2.0]) # 0.5m Puffer
        # Zielkoordinate 2: In der Hütte
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])
        
        # Aktuelle Roboter-Pose (x, y, theta) im 'world'-Frame
        self.robot_world_pose = None # (x, y, theta)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Statische TF für die Kamera publizieren
        self.publish_static_camera_tf()
        
        # Haupt-Kontrollschleife
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Docking Controller gestartet. Status: SEARCHING")

    def publish_static_camera_tf(self):
        # Publiziert die statische Transformation von chassis -> camera_sensor
        # basierend auf der model.sdf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot/chassis'
        t.child_frame_id = 'robot/chassis/camera_sensor' 
        
        # Pose aus SDF: 1.0 0 0.25 0 0 0
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.25
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info("Statische TF (chassis -> camera_sensor) publiziert.")

    def detection_callback(self, msg: AprilTagDetectionArray):
        # Schritt 1 & 2: Tag gefunden, stoppen
        if self.state == DockingState.SEARCHING and len(msg.detections) > 0:
            self.get_logger().info("Tag(s) gefunden. Stoppe Rotation.")
            self.publish_twist(0.0, 0.0)
            
            # Schritt 3: Position bestimmen
            self.state = DockingState.LOCALIZING
            self.run_localization(msg)

    def run_localization(self, msg: AprilTagDetectionArray):
        # Versucht, die Roboter-Welt-Pose zu berechnen
        try:
            detection = msg.detections[0] # Nimm die erste Detektion
            tag_id = detection.id
            tag_frame = f"tag36_11_{tag_id:05d}"
            
            # --- NEU: Definiere ein Timeout ---
            timeout_duration = rclpy.time.Duration(seconds=0.5)
            now = rclpy.time.Time()
            
            # 1. Hole T(world, tag)
            tf_world_tag = self.tf_buffer.lookup_transform(
                'world', 
                tag_frame, 
                now,  # <--- GEÄNDERT
                timeout=timeout_duration # <--- NEU
            )
            T_world_tag = transform_to_matrix(tf_world_tag.transform)

            # 2. Hole T(camera, tag)
            tf_cam_tag = self.tf_buffer.lookup_transform(
                'robot/chassis/camera_sensor', # Parent Frame
                tag_frame,                     # Child Frame
                now,  # <--- GEÄNDERT
                timeout=timeout_duration # <--- NEU
            )
            T_cam_tag = transform_to_matrix(tf_cam_tag.transform)
            
            # 3. Berechne T(tag, camera)
            T_tag_cam = inverse_matrix(T_cam_tag)

            # 4. Hole T(chassis, camera)
            tf_chassis_cam = self.tf_buffer.lookup_transform(
                'robot/chassis', 
                'robot/chassis/camera_sensor', 
                now,  # <--- GEÄNDERT
                timeout=timeout_duration # <--- NEU
            )
            T_chassis_cam = transform_to_matrix(tf_chassis_cam.transform)

            # 5. Berechne T(camera, chassis)
            T_cam_chassis = inverse_matrix(T_chassis_cam)

            # 6. Berechne T(world, chassis)
            # T(world, chassis) = T(world, tag) * T(tag, camera) * T(camera, chassis)
            T_world_chassis = T_world_tag @ T_tag_cam @ T_cam_chassis
            
            # Extrahiere (x, y, theta)
            x = T_world_chassis[0, 3]
            y = T_world_chassis[1, 3]
            quat = tf_transformations.quaternion_from_matrix(T_world_chassis)
            _, _, theta = euler_from_quaternion(quat)
            
            self.robot_world_pose = (x, y, theta)
            
            self.get_logger().info(f"Schritt 3: Roboter lokalisiert bei (x={x:.2f}, y={y:.2f}, th={math.degrees(theta):.1f})")
            
            # Nächster Zustand
            self.state = DockingState.GOTO_WAYPOINT_1

        except TransformException as ex:
            self.get_logger().error(f"Lokalisierung fehlgeschlagen: {ex}. Kehre zur Suche zurück.")
            self.state = DockingState.SEARCHING
        except Exception as e:
            self.get_logger().error(f"Fehler bei Lokalisierung: {e}")
            self.state = DockingState.SEARCHING

    def control_loop(self):
        # Haupt-Steuerungslogik
        
        # Wenn wir den Zustand > LOCALIZING haben, aber keine Pose, -> zurück zur Suche
        if self.state.value > DockingState.LOCALIZING.value and self.robot_world_pose is None:
            self.get_logger().warn("Pose verloren. Kehre zu SEARCHING zurück.")
            self.state = DockingState.SEARCHING

        # --- Schritt 1: Suchen ---
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.3) # Langsam drehen
            return

        # --- Die folgenden Schritte benötigen eine Pfadplanung/Regelung ---
        # Dies ist ein Platzhalter-Implementierung.
        # Du benötigst hier einen "Go-to-Goal"-Regler (z.B. PID).

        current_pose = self.robot_world_pose
        if current_pose is None: return
        
        # --- Schritt 4: Auf 5m Abstand ---
        if self.state == DockingState.GOTO_WAYPOINT_1:
            # Ziel: Ein Punkt auf dem 5m-Radius.
            # Wir fahren zu einem Punkt, der 5m vom Zentrum ENTFERNT ist,
            # auf der Linie zwischen Roboter und Zentrum.
            current_pos = np.array(current_pose[:2])
            vec_to_center = self.HUT_CENTER - current_pos
            dist_to_center = np.linalg.norm(vec_to_center)
            
            # Wir berechnen einen Zielpunkt auf dem 5m-Radius
            if dist_to_center > 0:
                target_point = self.HUT_CENTER - (vec_to_center / dist_to_center) * self.TARGET_RADIUS
                
                # HIER: "Go-to-Goal"-Logik für target_point einfügen
                if self.simple_go_to(target_point, 0.15):
                    self.get_logger().info("Schritt 4: 5m Radius erreicht.")
                    self.state = DockingState.GOTO_WAYPOINT_2

        # --- Schritt 5: Radius zur Öffnung fahren ---
        elif self.state == DockingState.GOTO_WAYPOINT_2:
            # Ziel: WAYPOINT_OPENING
            # HIER: "Go-to-Goal"-Logik für WAYPOINT_OPENING einfügen
            # Für eine "Radiusfahrt" müsstest du einen Arc-Controller implementieren.
            # Der Einfachheit halber fahren wir direkt zum Wegpunkt.
            if self.simple_go_to(self.WAYPOINT_OPENING, 0.1):
                self.get_logger().info("Schritt 5: Öffnung erreicht.")
                self.state = DockingState.GOTO_WAYPOINT_3

        # --- Schritt 6: Zur Hütte drehen ---
        elif self.state == DockingState.GOTO_WAYPOINT_3:
            # Ziel: WAYPOINT_INSIDE (Das ist Schritt 7)
            # Wir kombinieren Drehen und Reinfahren
            if self.simple_go_to(self.WAYPOINT_INSIDE, 0.1):
                self.get_logger().info("Schritt 7: Erfolgreich gedockt.")
                self.state = DockingState.FINAL_STOP
                
        # --- Ziel erreicht ---
        elif self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info("Docking-Sequenz abgeschlossen.")
            # Timer stoppen, um CPU zu sparen
            self.control_timer.cancel()

    def simple_go_to(self, target_pos: np.array, tolerance: float) -> bool:
        """
        Ein sehr einfacher P-Regler, um zu einem Ziel zu fahren.
        Gibt True zurück, wenn das Ziel erreicht ist.
        """
        if self.robot_world_pose is None:
            return False

        K_linear = 0.3
        K_angular = 1.0
        max_linear = 0.4
        max_angular = 0.5

        current_pos = np.array(self.robot_world_pose[:2])
        current_theta = self.robot_world_pose[2]
        
        dist_err = np.linalg.norm(target_pos - current_pos)
        
        if dist_err < tolerance:
            self.publish_twist(0.0, 0.0)
            return True

        # Winkel zum Ziel berechnen
        angle_to_target = math.atan2(
            target_pos[1] - current_pos[1],
            target_pos[0] - current_pos[0]
        )
        
        angle_err = angle_to_target - current_theta
        # Winkelfehler normalisieren (-pi, pi)
        angle_err = (angle_err + math.pi) % (2 * math.pi) - math.pi
        
        cmd = Twist()
        
        # Wenn wir falsch ausgerichtet sind, nur drehen
        if abs(angle_err) > 0.2: # 0.2 rad ~ 11 Grad
            cmd.linear.x = 0.0
            cmd.angular.z = np.clip(K_angular * angle_err, -max_angular, max_angular)
        else:
            # Sonst fahren und drehen
            cmd.linear.x = np.clip(K_linear * dist_err, 0.0, max_linear)
            cmd.angular.z = np.clip(K_angular * angle_err, -max_angular, max_angular)

        self.publish_twist_msg(cmd)
        return False

    def publish_twist(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
        
    def publish_twist_msg(self, msg: Twist):
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0) # Sicherstellen, dass der Roboter stoppt
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
