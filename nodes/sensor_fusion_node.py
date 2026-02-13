#!/usr/bin/env python3
"""
Sensor Fusion Node for Leo Rover (EKF Monitor Mode).
Subscribes to standard EKF output for logging and comparison.
Calculates error metrics against Ground Truth.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
import csv
import os


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # ============== CONFIGURATION ==============
        # Leo Rover parameters
        self.wheel_radius = 0.0625  # meters
        self.wheel_joints = [
            'wheel_FL_joint', 'wheel_FR_joint',
            'wheel_RL_joint', 'wheel_RR_joint',
        ]
        
        # ============== STATE VARIABLES ==============
        # Wheel encoder state
        self.prev_positions = {}
        self.prev_wheel_time = None
        self.wheel_velocities = {}
        self.v_wheel = 0.0
        
        # IMU state
        self.prev_imu_time = None
        self.v_imu = 0.0  # Velocity from integrated acceleration
        self.imu_accel_bias = 0.0
        self.imu_samples = []
        self.imu_calibrated = False
        
        # Ground truth (for validation)
        self.v_ground_truth = 0.0
        self.prev_gt_x = None
        self.prev_gt_y = None
        self.prev_gt_time = None
        
        # EKF (Robot Localization) output
        self.v_fused = 0.0
        self.is_on_ground = False
        
        # ============== SUBSCRIBERS ==============
        # Wheel encoders (joint states)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10
        )
        
        # IMU data
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw',
            self.imu_callback, 10
        )
        
        # Ground truth from Gazebo (True Physics State)
        self.gt_sub = self.create_subscription(
            Odometry, '/model/leo_rover/odometry',
            self.ground_truth_callback, 10
        )

        # Standard EKF Output
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.ekf_callback, 10
        )
        
        # ============== PUBLISHER ==============
        # Republish for plotting if needed
        self.fused_pub = self.create_publisher(
            Float64MultiArray, '/velocity/fused', 10
        )
        
        # ============== CSV LOGGING ==============
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        csv_path = os.path.join(project_root, 'data', 'sensor_fusion_data.csv')
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'v_wheel', 'v_imu', 'v_fused', 'v_ground_truth',
            'error_wheel', 'error_imu', 'error_fused',
            'state_bias', 'covariance'
        ])
        
        # Timer for logging and error calc
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        self.get_logger().info('Sensor Fusion Node started (EKF Monitor Mode)')
    
    # ============== WHEEL ENCODER PROCESSING ==============
    def joint_callback(self, msg: JointState):
        """Process wheel encoder data to get wheel-based velocity."""
        current_time = time.time()
        
        if self.prev_wheel_time is None:
            for i, name in enumerate(msg.name):
                if name in self.wheel_joints and i < len(msg.position):
                    self.prev_positions[name] = msg.position[i]
            self.prev_wheel_time = current_time
            return
        
        dt = current_time - self.prev_wheel_time
        if dt < 0.01:
            return
        
        wheel_vels = []
        for i, name in enumerate(msg.name):
            if name in self.wheel_joints and i < len(msg.position):
                if name in self.prev_positions:
                    dpos = msg.position[i] - self.prev_positions[name]
                    omega = dpos / dt
                    v = abs(omega) * self.wheel_radius
                    self.wheel_velocities[name] = v
                    wheel_vels.append(v)
                self.prev_positions[name] = msg.position[i]
        
        if wheel_vels:
            self.v_wheel = sum(wheel_vels) / len(wheel_vels)
        
        self.prev_wheel_time = current_time
    
    # ============== IMU PROCESSING ==============
    def imu_callback(self, msg: Imu):
        """Process IMU data to get acceleration-based velocity."""
        current_time = time.time()
        
        # Get forward acceleration (x-axis in robot frame)
        accel_x = msg.linear_acceleration.x
        
        # Calibration: collect samples when stationary
        if not self.imu_calibrated:
            self.imu_samples.append(accel_x)
            if len(self.imu_samples) >= 50:
                self.imu_accel_bias = sum(self.imu_samples) / len(self.imu_samples)
                self.imu_calibrated = True
                self.get_logger().info(f'IMU calibrated. Bias: {self.imu_accel_bias:.4f}')
            return
        
        # Remove bias
        accel_corrected = accel_x - self.imu_accel_bias
        
        # Integrate acceleration to get velocity
        if self.prev_imu_time is not None:
            dt = current_time - self.prev_imu_time
            if dt > 0 and dt < 0.1:
                self.v_imu += accel_corrected * dt
                # Prevent drift when acceleration is near zero
                if abs(accel_corrected) < 0.05 and abs(self.v_wheel) < 0.01:
                    self.v_imu *= 0.95  # Decay towards zero
                # Keep velocity positive (magnitude)
                self.v_imu = max(0, self.v_imu)
        
        self.prev_imu_time = current_time
    
    # ============== GROUND TRUTH (VALIDATION) ==============
    def ground_truth_callback(self, msg: Odometry):
        """Get ground truth velocity directly from Gazebo's physics engine."""
        z = msg.pose.pose.position.z
        
        # Check if rover is on the ground
        if z < 0.2:
            self.is_on_ground = True
        
        # Use Gazebo's internal velocity (noise-free ground truth)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v_ground_truth = math.sqrt(vx**2 + vy**2)

    # ============== EKF CALLBACK (ROBOT LOCALIZATION) ==============
    def ekf_callback(self, msg: Odometry):
        """Receive filtered odometry from the EKF node."""
        # Use the linear x velocity from the EKF
        # The EKF usually outputs in the `odom` frame, but the Twist is in the child_frame_id (base_link)
        self.v_fused = msg.twist.twist.linear.x

    # ============== MAIN LOOP ==============
    def timer_callback(self):
        """Logging loop."""
        
        # Compute errors (for tuning)
        error_wheel = abs(self.v_wheel - self.v_ground_truth)
        error_imu = abs(self.v_imu - self.v_ground_truth) if self.imu_calibrated else 0
        error_fused = abs(self.v_fused - self.v_ground_truth)
        
        # Publish fused velocity (optional, for visualization consistency)
        msg = Float64MultiArray()
        msg.data = [self.v_fused, self.v_wheel, self.v_imu, self.v_ground_truth]
        self.fused_pub.publish(msg)
        
        # Log to CSV
        # Using 0.0 placeholders for state bias/covariance since we aren't extracting them from EKF directly here
        if self.is_on_ground:
            self.csv_writer.writerow([
                time.time(),
                self.v_wheel, self.v_imu, self.v_fused, self.v_ground_truth,
                error_wheel, error_imu, error_fused,
                0.0, 0.0
            ])
            self.csv_file.flush()
        
        # Log to console only when moving
        if self.v_ground_truth > 0.001 or self.v_wheel > 0.001:
            self.get_logger().info(
                f'EKF: {self.v_fused:.3f} | '
                f'Wheel: {self.v_wheel:.3f} | '
                f'IMU: {self.v_imu:.3f} | '
                f'GT: {self.v_ground_truth:.3f} | '
                f'Err: {error_fused:.4f}'
            )
    
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
