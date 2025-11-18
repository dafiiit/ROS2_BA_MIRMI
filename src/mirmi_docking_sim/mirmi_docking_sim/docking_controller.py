#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
# qos_profile_sensor_data ist der Goldstandard für Sensordaten in ROS 2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class DockingState(Enum):
    SEARCHING = 1
    LOCALIZING = 2
    GOTO_RADIUS_POINT = 3
    FOLLOW_ARC = 4
    ALIGN_TO_HUT = 5
    FINAL_ALIGNMENT = 6
    DOCKING = 7
    FINAL_STOP = 8


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_theta = 0.0
        self.raw_odom_pose = None # Zum Speichern der "rohen" Odom-Werte
        
        # 1. Parameter Logik (funktioniert jetzt sicher)
        self.declare_parameter('use_ground_truth', False)
        gt_param = self.get_parameter('use_ground_truth').value
        # Sicherheits-Check für String "True"/"False"
        self.use_ground_truth = str(gt_param).lower() == 'true'
        
        # 2. Subscriber Setup mit qos_profile_sensor_data
        if self.use_ground_truth:
            self.get_logger().warn("!!! ACHTUNG: Nutze GROUND TRUTH (/model/robot/pose) !!!")
            self.gt_sub = self.create_subscription(
                PoseArray,
                '/model/robot/pose',
                self.ground_truth_callback,
                qos_profile_sensor_data # Passt zu Best Effort der Bridge
            )
        else:
            self.get_logger().info("Modus: Nutze Standard-Odometrie (/odom)")
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                qos_profile_sensor_data
            )
        
        # State Machine & Navigation
        self.state = DockingState.SEARCHING
        self.state_entry_time = self.get_clock().now()
        
        self.HUT_CENTER = np.array([5.0, 2.0])
        self.TARGET_RADIUS = 5.5
        self.RADIUS_TOLERANCE = 0.15
        self.WAYPOINT_OPENING = np.array([6.5, 2.0])
        self.WAYPOINT_INSIDE = np.array([5.5, 2.0])
        
        self.DOCKING_TARGET_FRAME = 'tag36_11_00001'
        self.DOCKING_TARGET_ID = 1
        self.DOCKING_TARGET_DISTANCE = 1.9
        self.DOCKING_SPEED_ANGULAR = 1.0
        
        self.IMAGE_WIDTH = 640.0
        self.IMAGE_CENTER_X = self.IMAGE_WIDTH / 2.0
        self.PIXEL_TOLERANCE = 10.0
        self.K_P_PIXEL = 0.005
        self.NUDGE_SPEED = 0.15
        self.NUDGE_DURATION = 2.0
        
        self.current_odom_pose = None
        self.has_localized = False
        self.odom_received_once = False
        
        self.last_target_detection = None
        self.last_detection_time = self.get_clock().now()
        
        self.arc_direction = 1.0
        self.current_arc_goal = None
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS für State Publisher (Latching)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/docking_controller/state', state_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)
       
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.debug_timer = self.create_timer(2.0, self.debug_status)
        
        self._align_nudge_active = False
        self._align_nudge_start_time = None
        self._docking_drive_active = False
        self._docking_drive_start_time = None
        
        self.get_logger().info("DOCKING CONTROLLER GESTARTET")
        initial_state_msg = String()
        initial_state_msg.data = self.state.name
        self.state_pub.publish(initial_state_msg)
        
        # Subscriber für manuellen Reset (Teleport)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.set_initial_pose_callback,
            10
        )

    def debug_status(self):
        if self.current_odom_pose is None:
            topic = "/model/robot/pose" if self.use_ground_truth else "/odom"
            self.get_logger().warn(f"State: {self.state.name} | WARTE AUF DATEN ({topic}) ❌")
        # Sonst keine Ausgabe um Log sauber zu halten

    def ground_truth_callback(self, msg: PoseArray):
        if len(msg.poses) == 0: return
        robot_pose = msg.poses[0]
        pos = robot_pose.position
        orient = robot_pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        self.current_odom_pose = (pos.x, pos.y, yaw)
        if not self.odom_received_once:
            self.get_logger().info(f"✓ Ground Truth empfangen! ({pos.x:.2f}, {pos.y:.2f})")
            self.odom_received_once = True
    
    def set_initial_pose_callback(self, msg):
        if self.raw_odom_pose is None:
            self.get_logger().warn("Kann Pose nicht setzen - noch keine Odom empfangen!")
            return

        # 1. Wo sollen wir sein? (Vom Test-Runner geschickt)
        target_x = msg.pose.pose.position.x
        target_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, target_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2. Wo sind wir laut Encoder gerade?
        raw_x, raw_y, raw_theta = self.raw_odom_pose

        # 3. Offset berechnen: Offset = Soll - Ist
        self.odom_offset_x = target_x - raw_x
        self.odom_offset_y = target_y - raw_y
        self.odom_offset_theta = target_theta - raw_theta
        
        self.get_logger().info(f"⚠️ TELEPORT ERKANNT! Neuer Offset gesetzt: x={self.odom_offset_x:.2f}, y={self.odom_offset_y:.2f}")

    def odom_callback(self, msg: Odometry):
        # Rohdaten extrahieren
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, raw_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # Speichern für die Offset-Berechnung
        self.raw_odom_pose = (pos.x, pos.y, raw_yaw)

        # --- HIER IST DER MATHEMATISCHE TRICK ---
        # Wir addieren den Offset auf die Rohdaten
        eff_x = pos.x + self.odom_offset_x
        eff_y = pos.y + self.odom_offset_y
        eff_yaw = raw_yaw + self.odom_offset_theta
        
        # Winkel normalisieren (-pi bis pi), damit der Controller nicht verwirrt wird
        eff_yaw = (eff_yaw + math.pi) % (2 * math.pi) - math.pi

        # Das ist jetzt die Pose, mit der der Controller arbeitet
        self.current_odom_pose = (eff_x, eff_y, eff_yaw)

        if not self.odom_received_once:
            self.get_logger().info(f"✓ ODOMETRIE EMPFANGEN (mit Offset)!")
            self.odom_received_once = True

    def change_state(self, new_state: DockingState):
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        self._align_nudge_active = False
        self._docking_drive_active = False
        
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"STATE CHANGE: {old_state.name} -> {new_state.name}")

    def detection_callback(self, msg: AprilTagDetectionArray):
        num_detections = len(msg.detections)
        if self.state == DockingState.SEARCHING and num_detections > 0:
            self.get_logger().info(f"✓ Tag gefunden! Stoppe Suche.")
            self.publish_twist(0.0, 0.0)
            self.change_state(DockingState.LOCALIZING)
            
        if self.state in [DockingState.SEARCHING, DockingState.LOCALIZING]:
            if num_detections > 0:
                self.run_localization(msg)
        
        if num_detections > 0:
            for detection in msg.detections:
                if detection.id == self.DOCKING_TARGET_ID:
                    self.last_target_detection = detection
                    self.last_detection_time = self.get_clock().now()

    def run_localization(self, msg: AprilTagDetectionArray):
        if self.current_odom_pose is None: return
        if not self.has_localized:
            self.has_localized = True
            self.get_logger().info("✓ Lokalisierung initialisiert -> GOTO_RADIUS_POINT")
            self.change_state(DockingState.GOTO_RADIUS_POINT)

    def control_loop(self):
        if self.current_odom_pose is None: return
        
        current_pos_vec = np.array(self.current_odom_pose[:2])
        current_theta = self.current_odom_pose[2]
        
        if self.state == DockingState.SEARCHING:
            self.publish_twist(0.0, 0.3)
            return
        
        if self.state == DockingState.LOCALIZING:
            self.publish_twist(0.0, 0.0)
            return
        
        if self.state == DockingState.GOTO_RADIUS_POINT:
            vec_to_center = self.HUT_CENTER - current_pos_vec
            dist_to_center = np.linalg.norm(vec_to_center)
            if dist_to_center > 0.01:
                target_point = self.HUT_CENTER - (vec_to_center / dist_to_center) * self.TARGET_RADIUS
                if self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE):
                    opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
                    current_angle = math.atan2(current_pos_vec[1] - self.HUT_CENTER[1], current_pos_vec[0] - self.HUT_CENTER[0])
                    angle_diff = (opening_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
                    self.arc_direction = 1.0 if angle_diff > 0 else -1.0
                    self.change_state(DockingState.FOLLOW_ARC)
            else:
                target_point = current_pos_vec - vec_to_center * (self.TARGET_RADIUS + 1.0)
                self.simple_go_to(target_point, tolerance=self.RADIUS_TOLERANCE)
            return
            
        if self.state == DockingState.FOLLOW_ARC:
            current_angle = math.atan2(current_pos_vec[1] - self.HUT_CENTER[1], current_pos_vec[0] - self.HUT_CENTER[0])
            opening_angle = math.atan2(self.WAYPOINT_OPENING[1] - self.HUT_CENTER[1], self.WAYPOINT_OPENING[0] - self.HUT_CENTER[0])
            angle_err = (opening_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_err) < 0.05:
                self.change_state(DockingState.ALIGN_TO_HUT)
                self.publish_twist(0.0, 0.0)
                self.current_arc_goal = None
                return

            if self.current_arc_goal is None:
                step_angle = 0.5 / self.TARGET_RADIUS
                next_angle = current_angle + (self.arc_direction * step_angle)
                gx = self.HUT_CENTER[0] + self.TARGET_RADIUS * math.cos(next_angle)
                gy = self.HUT_CENTER[1] + self.TARGET_RADIUS * math.sin(next_angle)
                self.current_arc_goal = np.array([gx, gy])

            if self.simple_go_to(self.current_arc_goal, tolerance=0.15):
                self.current_arc_goal = None
            return
            
        if self.state == DockingState.ALIGN_TO_HUT:
            target_theta = math.atan2(self.WAYPOINT_INSIDE[1] - current_pos_vec[1], self.WAYPOINT_INSIDE[0] - current_pos_vec[0])
            angle_err = (target_theta - current_theta + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_err) < 0.05:
                self.change_state(DockingState.FINAL_ALIGNMENT)
                self.publish_twist(0.0, 0.0)
            else:
                ang_vel = np.clip(1.5 * angle_err, -0.5, 0.5)
                self.publish_twist(0.0, ang_vel)
            return
            
        if self.state == DockingState.FINAL_ALIGNMENT:
            if self._align_nudge_active:
                if (self.get_clock().now() - self._align_nudge_start_time).nanoseconds / 1e9 < self.NUDGE_DURATION:
                    self.publish_twist(self.NUDGE_SPEED, 0.0)
                else:
                    self.publish_twist(0.0, 0.0)
                    self._align_nudge_active = False
                return

            tag_visible = self.last_target_detection and (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9 < 1.0
            if tag_visible:
                pixel_err = self.IMAGE_CENTER_X - self.last_target_detection.centre.x
                if abs(pixel_err) <= self.PIXEL_TOLERANCE:
                    self.change_state(DockingState.DOCKING)
                    self.publish_twist(0.0, 0.0)
                else:
                    ang_vel = np.clip(self.K_P_PIXEL * pixel_err, -self.DOCKING_SPEED_ANGULAR, self.DOCKING_SPEED_ANGULAR)
                    self.publish_twist(0.0, ang_vel)
            else:
                if not self._align_nudge_active:
                    self._align_nudge_active = True
                    self._align_nudge_start_time = self.get_clock().now()
            return
            
        if self.state == DockingState.DOCKING:
            if self._docking_drive_active:
                if (self.get_clock().now() - self._docking_drive_start_time).nanoseconds / 1e9 < self.NUDGE_DURATION:
                    self.publish_twist(self.NUDGE_SPEED, 0.0)
                else:
                    self.publish_twist(0.0, 0.0)
                    self._docking_drive_active = False
                    self.change_state(DockingState.FINAL_ALIGNMENT)
                return
            try:
                t = self.tf_buffer.lookup_transform('robot/chassis', self.DOCKING_TARGET_FRAME, rclpy.time.Time())
                dist_err = t.transform.translation.x - self.DOCKING_TARGET_DISTANCE
                if abs(dist_err) < 0.1:
                    self.change_state(DockingState.FINAL_STOP)
                    self.publish_twist(0.0, 0.0)
                else:
                    self._docking_drive_active = True
                    self._docking_drive_start_time = self.get_clock().now()
            except TransformException:
                self.change_state(DockingState.FINAL_ALIGNMENT)
            return

        if self.state == DockingState.FINAL_STOP:
            self.publish_twist(0.0, 0.0)
            if np.linalg.norm(current_pos_vec - self.HUT_CENTER) > 3.0:
                self.change_state(DockingState.SEARCHING)

    def simple_go_to(self, target_pos, tolerance):
        if self.current_odom_pose is None: return False
        current_pos = np.array(self.current_odom_pose[:2])
        theta = self.current_odom_pose[2]
        dist = np.linalg.norm(target_pos - current_pos)
        if dist < tolerance:
            self.publish_twist(0.0, 0.0)
            return True
        target_angle = math.atan2(target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])
        angle_diff = (target_angle - theta + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_diff) > 0.3:
            self.publish_twist(0.0, np.clip(2.0 * angle_diff, -1.0, 1.0))
        else:
            lin = np.clip(1.0 * dist, 0.0, 1.5) * (1.0 - abs(angle_diff)/math.pi)
            self.publish_twist(lin, np.clip(2.0 * angle_diff, -1.0, 1.0))
        return False

    def publish_twist(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
