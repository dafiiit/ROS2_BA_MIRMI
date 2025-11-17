#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import time
import csv
import math
import random
import os
from datetime import datetime
from tf_transformations import quaternion_from_euler

class DockingTestRunner(Node):
    def __init__(self):
        super().__init__('docking_test_runner')

        # --- KONFIGURATION ---
        self.declare_parameter('test_attempts', 5)
        self.TEST_ATTEMPTS = self.get_parameter('test_attempts').value
        self.TIMEOUT_SEC = 300        # 5 min
        self.HUT_POSITION = np.array([5.0, 2.0])
        # Kollisionsbox der Hütte (etwas größer als das visuelle Modell)
        self.HUT_COLLISION_BOX = {
            'x_min': 3.4, 'x_max': 6.6, # 3m lang + Puffer
            'y_min': 0.8, 'y_max': 3.2  # 2.2m breit + Puffer
        }
        
        # --- LOGGING SETUP ---
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = f"docking_results_{timestamp}.csv"
        self.setup_csv()

        # --- ROS SCHNITTSTELLEN ---
        # Zum Teleportieren
        self.pose_pub = self.create_publisher(Pose, '/model/robot/set_pose', 1)
        # Zum Stoppen des Roboters beim Reset
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Überwachung
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/docking_controller/state', self.state_callback, 10)

        self.current_pose = None
        self.current_controller_state = "UNKNOWN"
        
        # Test Status
        self.test_active = False
        self.start_time = 0
        self.current_attempt = 0
        self.results = []

        self.get_logger().info(f"Test Runner bereit. Ergebnisse in: {self.csv_filename}")
        
        # Starte den Loop
        self.timer = self.create_timer(1.0, self.test_loop)

    def setup_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Versuch_ID", 
                "Ergebnis",           # SUCCESS, COLLISION, TIMEOUT
                "Dauer_Sekunden", 
                "Start_X", "Start_Y", "Start_Theta_Deg", "Start_Distanz",
                "End_X", "End_Y", 
                "Letzter_State", 
                "Fehlergrund"
            ])

    def log_result(self, result, reason):
        duration = time.time() - self.start_time
        
        start_x, start_y, start_theta = self.start_pose_data
        start_dist = np.linalg.norm(np.array([start_x, start_y]) - self.HUT_POSITION)
        
        end_x = self.current_pose.position.x if self.current_pose else 0.0
        end_y = self.current_pose.position.y if self.current_pose else 0.0

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.current_attempt,
                result,
                f"{duration:.2f}",
                f"{start_x:.2f}", f"{start_y:.2f}", f"{math.degrees(start_theta):.2f}", f"{start_dist:.2f}",
                f"{end_x:.2f}", f"{end_y:.2f}",
                self.current_controller_state,
                reason
            ])
        
        self.get_logger().info(f"VERSUCH {self.current_attempt}: {result} ({reason}) nach {duration:.2f}s")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def state_callback(self, msg):
        self.current_controller_state = msg.data

    def get_random_start_pose(self):
        """Generiert eine Position zwischen 6m (außerhalb Sichtweite Hütte) und 10m"""
        min_dist = 6.0 
        max_dist = 10.0
        
        dist = random.uniform(min_dist, max_dist)
        angle = random.uniform(0, 2 * math.pi)
        
        x = self.HUT_POSITION[0] + dist * math.cos(angle)
        y = self.HUT_POSITION[1] + dist * math.sin(angle)
        
        # Zufällige Rotation des Roboters (0 bis 360 Grad)
        theta = random.uniform(0, 2 * math.pi)
        
        return x, y, theta

    def check_collision(self):
        if not self.current_pose: return False
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Simple Box-Check
        if (self.HUT_COLLISION_BOX['x_min'] < x < self.HUT_COLLISION_BOX['x_max'] and
            self.HUT_COLLISION_BOX['y_min'] < y < self.HUT_COLLISION_BOX['y_max']):
            return True
        return False

    def reset_robot(self):
        """Setzt den Roboter an eine neue Position"""
        x, y, theta = self.get_random_start_pose()
        self.start_pose_data = (x, y, theta)
        
        # 1. Motor stoppen
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        # 2. Teleportieren
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = 0.1 # Leicht über Boden
        
        q = quaternion_from_euler(0, 0, theta)
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
        
        # Mehrfach senden um sicherzugehen (Gazebo Bridge UDP kann Pakete droppen)
        for _ in range(5):
            self.pose_pub.publish(pose_msg)
            time.sleep(0.05)
            
        self.get_logger().info(f"Reset auf: x={x:.1f}, y={y:.1f}, theta={math.degrees(theta):.1f}°")
        time.sleep(2.0) # Warte kurz, bis Simulation sich beruhigt

    def test_loop(self):
        # Wenn Limit erreicht
        if self.current_attempt >= self.TEST_ATTEMPTS:
            self.get_logger().info("ALLE TESTDURCHLÄUFE BEENDET.")
            self.destroy_node()
            return

        # Starte neuen Versuch
        if not self.test_active:
            self.current_attempt += 1
            self.reset_robot()
            self.start_time = time.time()
            self.test_active = True
            return

        # --- WÄHREND DES VERSUCHS ---
        
        # 1. Check Timeout
        if (time.time() - self.start_time) > self.TIMEOUT_SEC:
            self.log_result("FAILURE", "TIMEOUT")
            self.test_active = False
            return

        # 2. Check Kollision
        if self.check_collision():
            self.log_result("FAILURE", "COLLISION_WITH_HUT")
            self.test_active = False
            return

        # 3. Check Success (State == FINAL_STOP)
        if self.current_controller_state == "FINAL_STOP":
            self.log_result("SUCCESS", "REACHED_FINAL_STOP")
            self.test_active = False
            return

def main(args=None):
    rclpy.init(args=args)
    runner = DockingTestRunner()
    try:
        rclpy.spin(runner)
    except KeyboardInterrupt:
        pass
    finally:
        runner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
