#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs_py import point_cloud2
from enum import Enum
import math
from scipy.spatial.transform import Rotation as R
import time

class DockingState(Enum):
    WAITING_FOR_PC = 0
    REGISTERING = 1
    APPROACHING = 2
    DOCKING = 3
    DONE = 4

class PCLDockingController(Node):
    def __init__(self):
        super().__init__('pcl_docking_controller')
        
        # Parameters
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth/points',
            self.pc_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Debug publisher for the transformed reference cloud (optional, for visualization)
        self.debug_pc_pub = self.create_publisher(PointCloud2, '/debug/aligned_cloud', 10)
        
        # State
        self.state = DockingState.WAITING_FOR_PC
        self.latest_pc_msg = None
        self.reference_cloud = self.generate_reference_cloud()
        self.current_pose_estimate = np.eye(4) # 4x4 Transformation matrix (Robot in Hut frame or Hut in Robot frame?)
        # Let's assume we want to find T_robot_hut (Hut pose in Robot frame)
        
        self.get_logger().info('PCL Docking Controller started. Waiting for PointCloud...')
        self.get_logger().info(f'Generated Reference Cloud with {len(self.reference_cloud)} points.')

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

    def generate_reference_cloud(self):
        """
        Generates a synthetic point cloud of the hut based on model.sdf dimensions.
        The hut frame is at the center of the floor.
        """
        points = []
        density = 20 # points per meter
        
        # Helper to add points for a box
        def add_box(center, size):
            # Simple surface sampling
            half_size = size / 2.0
            x_range = np.linspace(center[0] - half_size[0], center[0] + half_size[0], int(size[0] * density))
            y_range = np.linspace(center[1] - half_size[1], center[1] + half_size[1], int(size[1] * density))
            z_range = np.linspace(center[2] - half_size[2], center[2] + half_size[2], int(size[2] * density))
            
            # Add points for faces (simplified: just fill the volume or surface? Surface is better)
            # For now, let's just fill it sparsely or do surface. Let's do surface.
            # Actually, just random sampling or grid on surface is enough.
            # Let's just do a grid on the main visible faces (inside walls)
            
            # Back Wall (facing +x direction from inside? No, back wall is at x=-1.45)
            # Inner surface of back wall is at x = -1.45 + 0.05 = -1.4
            if size[0] < 0.5: # It's a wall
                 # It's the back wall
                 y_grid, z_grid = np.meshgrid(y_range, z_range)
                 x_val = center[0] + half_size[0] # Inner face
                 for y, z in zip(y_grid.flatten(), z_grid.flatten()):
                     points.append([x_val, y, z])
            elif size[1] < 0.5: # Side walls
                # Left/Right walls
                x_grid, z_grid = np.meshgrid(x_range, z_range)
                y_val = center[1] - half_size[1] if center[1] > 0 else center[1] + half_size[1] # Inner face
                for x, z in zip(x_grid.flatten(), z_grid.flatten()):
                    points.append([x, y_val, z])
            
        # Dimensions from SDF
        # Back wall: -1.45, 0, 0.75; size 0.1, 2.2, 1.5
        add_box(np.array([-1.45, 0, 0.75]), np.array([0.1, 2.2, 1.5]))
        
        # Left wall: 0, 1.05, 0.75; size 3.0, 0.1, 1.5
        add_box(np.array([0, 1.05, 0.75]), np.array([3.0, 0.1, 1.5]))
        
        # Right wall: 0, -1.05, 0.75; size 3.0, 0.1, 1.5
        add_box(np.array([0, -1.05, 0.75]), np.array([3.0, 0.1, 1.5]))
        
        return np.array(points, dtype=np.float32)

    def pc_callback(self, msg):
        self.latest_pc_msg = msg
        if self.state == DockingState.WAITING_FOR_PC:
            self.state = DockingState.REGISTERING
            self.get_logger().info("PointCloud received. Switching to REGISTERING.")

    def control_loop(self):
        if self.state == DockingState.WAITING_FOR_PC:
            return
            
        if self.state == DockingState.REGISTERING:
            self.perform_registration()
            return
            
        if self.state == DockingState.APPROACHING:
            self.perform_approach()
            return

    def perform_registration(self):
        if self.latest_pc_msg is None: return
        
        # Convert ROS PointCloud2 to numpy
        # We only care about x, y, z
        gen = point_cloud2.read_points(self.latest_pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        scene_points = np.array(list(gen))
        
        if len(scene_points) < 100:
            self.get_logger().warn("Not enough points in cloud.")
            return

        # Downsample scene cloud for speed (simple random sampling)
        if len(scene_points) > 1000:
            idx = np.random.choice(len(scene_points), 1000, replace=False)
            scene_points = scene_points[idx]

        # Run ICP
        # We want to align scene_points (Source) to self.reference_cloud (Target)
        # T * Source = Target
        # T is the transform from Camera Frame to Hut Frame
        
        T, distances, iterations = self.icp(scene_points, self.reference_cloud, max_iterations=20, tolerance=0.001)
        
        # Calculate fitness (mean squared error)
        fitness = np.mean(distances)
        self.get_logger().info(f"ICP converged in {iterations} iterations. Fitness: {fitness:.4f}")
        
        if fitness < 0.05: # Threshold for good match
            self.current_pose_estimate = T
            self.state = DockingState.APPROACHING
            self.get_logger().info("Registration successful! Switching to APPROACHING.")
        else:
            self.get_logger().warn(f"Registration poor (fitness {fitness:.4f}). Retrying...")

    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
          A: Nxm numpy array of corresponding points
          B: Nxm numpy array of corresponding points
        Returns:
          T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
          R: mxm rotation matrix
          t: mx1 translation vector
        '''
        assert A.shape == B.shape

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[2,:] *= -1
            R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R, centroid_A.T)

        # homogeneous transformation
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T, R, t

    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: Indices of the nearest neighbor in dst
        '''
        # Using Scipy's KDTree for fast nearest neighbor search
        # Note: We should build the tree once if dst is static, but here we do it every time for simplicity
        # Optimization: Build tree in __init__
        if not hasattr(self, 'reference_tree'):
            from scipy.spatial import KDTree
            self.reference_tree = KDTree(dst)
            
        distances, indices = self.reference_tree.query(src)
        return distances, indices

    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001):
        '''
        The Iterative Closest Point method: aligns source A to target B
        '''
        # Ensure A and B are numpy arrays
        src = np.copy(A)
        dst = np.copy(B)

        if init_pose is not None:
            src = np.ones((src.shape[0], 4))
            src[:,0:3] = A
            src = np.dot(init_pose, src.T).T
            src = src[:,0:3]

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(src, dst)

            # compute the transformation between the current source and nearest destination points
            T, _, _ = self.best_fit_transform(src, dst[indices])

            # update the current source
            src = np.ones((src.shape[0], 4))
            src[:,0:3] = src[:,0:3] # restore just xyz
            # Actually we need to apply T to the current src
            # But best_fit_transform returns T that maps src -> dst[indices]
            # So we apply T to src
            src[:,0:3] = src[:,0:3] # Wait, src was modified in previous loop? Yes.
            # Let's re-do the homogeneous coord stuff properly
            # src is Nx3
            
            # Apply T
            src_h = np.ones((src.shape[0], 4))
            src_h[:,0:3] = src[:,0:3] # previous src
            src_h = np.dot(T, src_h.T).T
            src = src_h[:,0:3]

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculate final transformation
        # T_final * A = src
        # We need to find the total transformation. 
        # Instead of accumulating T, we can just compute best fit from A to final src?
        # No, because point correspondence changed.
        # Actually, it's better to accumulate T.
        
        # Let's re-calculate T from original A to final src? 
        # No, ICP usually returns the transformation.
        # Let's just run best_fit from A to src (final aligned) - this works if the correspondence is 1-to-1 which it is not.
        # Correct way: Accumulate T.
        
        # Simplified: Just return the T that maps A to the final position? 
        # We can compute T from A to src using best_fit_transform? 
        # No, because src points moved independently? No, they moved rigidly.
        # So yes, we can compute T from A to src.
        
        T, _, _ = self.best_fit_transform(A, src)
        
        return T, distances, i

    def perform_approach(self):
        if self.current_pose_estimate is None: return
        
        # T is Camera pose in Hut Frame (T_hut_cam)
        # We want to navigate to a target point in Hut Frame.
        
        # Extract rotation and translation
        R_mat = self.current_pose_estimate[:3, :3]
        t_vec = self.current_pose_estimate[:3, 3]
        
        # Define targets (in Hut Frame)
        # Hut is centered at 0,0. Opening is at +x direction (based on walls at +/- y and back at -x)
        # Approach point: 2.0m in front of center
        approach_target = np.array([2.0, 0.0, 0.0])
        docking_target = np.array([0.0, 0.0, 0.0])
        
        target = approach_target if self.state == DockingState.APPROACHING else docking_target
        
        # Calculate vector to target in Hut Frame
        v_hut = target - t_vec
        
        # Transform vector to Camera/Robot Frame
        # v_robot = R^T * v_hut
        v_robot = np.dot(R_mat.T, v_hut)
        
        # Calculate distance and angle
        dist = np.linalg.norm(v_robot[:2]) # 2D distance
        angle = math.atan2(v_robot[1], v_robot[0])
        
        self.get_logger().info(f"Target Dist: {dist:.2f}, Angle: {angle:.2f}")
        
        # Control Logic
        if self.state == DockingState.APPROACHING:
            if dist < 0.2:
                self.state = DockingState.DOCKING
                self.get_logger().info("Reached Approach Point. Switching to DOCKING.")
                self.publish_twist(0.0, 0.0)
                return
                
            # Simple P-Controller
            lin_vel = np.clip(0.5 * dist, 0.0, 0.3)
            ang_vel = np.clip(2.0 * angle, -0.5, 0.5)
            
            # Slow down if turning too much
            if abs(angle) > 0.5:
                lin_vel = 0.0
                
            self.publish_twist(lin_vel, ang_vel)
            
        elif self.state == DockingState.DOCKING:
            if dist < 0.1:
                self.state = DockingState.DONE
                self.get_logger().info("DOCKING COMPLETED!")
                self.publish_twist(0.0, 0.0)
                return
                
            lin_vel = np.clip(0.3 * dist, 0.0, 0.15)
            ang_vel = np.clip(2.0 * angle, -0.5, 0.5)
            self.publish_twist(lin_vel, ang_vel)
            
        elif self.state == DockingState.DONE:
            self.publish_twist(0.0, 0.0)

    def publish_twist(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PCLDockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
