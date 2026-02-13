#!/usr/bin/env python3
"""
Slip Ratio Measurement Node for Leo Rover.
Computes and publishes tire slip ratio based on wheel velocities and odometry.

Slip Ratio = (wheel_velocity - linear_velocity) / max(wheel_velocity, linear_velocity)
Where wheel_velocity = angular_velocity * wheel_radius
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
import csv
import os

class SlipRatioNode(Node):
    def __init__(self):
        super().__init__('slip_ratio_node')
        
        # Leo Rover wheel parameters
        self.wheel_radius = 0.0625  # meters
        
        # Wheel joint names
        self.wheel_joints = [
            'wheel_FL_joint',
            'wheel_FR_joint', 
            'wheel_RL_joint',
            'wheel_RR_joint',
        ]
        
        # State for velocity computation from position diff
        self.prev_positions = {}
        self.prev_time = None
        # Initialize velocities to 0 so we can publish immediately
        self.wheel_velocities = {j: 0.0 for j in self.wheel_joints}
        
        # Odometry data
        self.v_rover_pos = 0.0  # From ground truth /model/...
        self.v_rover_wheel = 0.0 # From /wheel_odom (DiffDrive)
        self.v_rover_ekf = 0.0
        
        # Ground Truth state
        self.prev_model_x = None
        self.prev_model_y = None
        self.prev_model_time = None
        
        # Wheel Odom state
        self.prev_odom_x = None
        self.prev_odom_y = None
        self.prev_odom_time = None
        
        self.is_on_ground = False
        
        # Output
        self.slip_ratios_pos = [0.0] * 4
        self.slip_ratios_ekf = [0.0] * 4
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Subscribe to raw Gazebo odometry (DiffDrive mode)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to Gazebo model pose odometry (works in both modes)
        self.model_odom_sub = self.create_subscription(
            Odometry,
            '/model/leo_rover/odometry',
            self.model_odom_callback,
            10
        )
        
        # Subscribe to EKF filtered odometry
        self.ekf_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.ekf_callback,
            10
        )
        
        # Publisher
        self.slip_pub = self.create_publisher(
            Float64MultiArray,
            '/slip_ratio',
            10
        )
        
        # CSV Logging setup
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.csv_file_path = os.path.join(project_root, 'data', 'traction_data.csv')
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 
            'v_wheel_avg', 
            'v_rover_pos', 'v_rover_ekf', 
            'avg_slip_pos', 'avg_slip_ekf'
        ])
        
        # Timer for periodic computation and logging
        self.timer = self.create_timer(0.1, self.compute_slip_ratio)
        
        self.get_logger().info('Slip Ratio Node started (Dual Mode: Position vs EKF)')
        self.last_log_time = time.time()
    
    def joint_callback(self, msg: JointState):
        """Get wheel velocities directly from Gazebo's JointState velocity field."""
        # Debug logging on first message
        if self.prev_time is None:
            self.get_logger().info(f"Received first joint state message with {len(msg.name)} joints")
            if len(msg.velocity) > 0:
                self.get_logger().info("Using msg.velocity directly from Gazebo (encoder-like)")
            else:
                self.get_logger().warn("msg.velocity is empty, falling back to position differentiation")
            self.prev_time = time.time()
        
        # Use Gazebo's internal velocity if available (preferred - cleaner data)
        if len(msg.velocity) >= len(msg.name):
            for i, name in enumerate(msg.name):
                if name in self.wheel_joints and i < len(msg.velocity):
                    # Direct velocity from physics engine - like encoder data
                    self.wheel_velocities[name] = abs(msg.velocity[i])
        else:
            # Fallback: compute from position change (noisier)
            current_time = self.get_clock().now().nanoseconds / 1e9
            dt = current_time - self.prev_time
            if dt < 0.01:
                return
            
            for i, name in enumerate(msg.name):
                if name in self.wheel_joints and i < len(msg.position):
                    if name in self.prev_positions:
                        dpos = msg.position[i] - self.prev_positions[name]
                        omega = dpos / dt
                        self.wheel_velocities[name] = abs(omega)
                    self.prev_positions[name] = msg.position[i]
            
            self.prev_time = current_time
    
    def odom_callback(self, msg: Odometry):
        """Process local wheel odometry (from DiffDrive plugin)."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.prev_odom_time is None:
            self.prev_odom_x = x
            self.prev_odom_y = y
            self.prev_odom_time = current_time
            return
            
        dt = current_time - self.prev_odom_time
        if dt < 0.01:
            return
            
        dx = x - self.prev_odom_x
        dy = y - self.prev_odom_y
        
        self.v_rover_wheel = math.sqrt(dx**2 + dy**2) / dt
        
        self.prev_odom_x = x
        self.prev_odom_y = y
        self.prev_odom_time = current_time

    def model_odom_callback(self, msg: Odometry):
        """Get ground truth velocity directly from Gazebo's physics engine."""
        z = msg.pose.pose.position.z
        
        if z < 0.2:
            self.is_on_ground = True
        
        # Use Gazebo's internal velocity (ground truth, noise-free)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v_rover_pos = math.sqrt(vx**2 + vy**2)

    def ekf_callback(self, msg: Odometry):
        """Receive filtered velocity from EKF."""
        self.v_rover_ekf = msg.twist.twist.linear.x
    
    def compute_slip_ratio(self):
        """Compute slip ratio for both methods and publish/log."""
        if len(self.wheel_velocities) < 4:
            return
        
        wheel_vels = []
        for i, joint in enumerate(self.wheel_joints):
            if joint in self.wheel_velocities:
                omega = self.wheel_velocities[joint]
                wheel_vel = omega * self.wheel_radius  # m/s
                wheel_vels.append(wheel_vel)
        
        # Pad if necessary
        while len(wheel_vels) < 4:
            wheel_vels.append(0.0)
            
        avg_wheel_vel = sum(wheel_vels) / 4.0

        # --- Method 1: Position Based ---
        for i in range(4):
            wheel_v = wheel_vels[i]
            rover_v = abs(self.v_rover_pos)
            max_v = max(wheel_v, rover_v)
            if max_v > 0.01:
                slip = (wheel_v - rover_v) / max_v
                self.slip_ratios_pos[i] = max(-1.0, min(1.0, slip))
            else:
                self.slip_ratios_pos[i] = 0.0

        # --- Method 2: EKF Based ---
        for i in range(4):
            wheel_v = wheel_vels[i]
            rover_v = abs(self.v_rover_ekf)
            max_v = max(wheel_v, rover_v)
            if max_v > 0.01:
                slip = (wheel_v - rover_v) / max_v
                self.slip_ratios_ekf[i] = max(-1.0, min(1.0, slip))
            else:
                self.slip_ratios_ekf[i] = 0.0
        
        # Publish (Legacy: sending pos-based as default for now, or maybe just avg?)
        # Keeping existing behavior of sending 4 values, using POS based as it's the 'manual' one
        msg = Float64MultiArray()
        msg.data = self.slip_ratios_pos
        self.slip_pub.publish(msg)
        
        if self.is_on_ground:
            # Log averages
            avg_slip_pos = sum(self.slip_ratios_pos) / 4
            avg_slip_ekf = sum(self.slip_ratios_ekf) / 4
            
            self.csv_writer.writerow([
                self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
                avg_wheel_vel,
                self.v_rover_pos,
                self.v_rover_ekf,
                avg_slip_pos,
                avg_slip_ekf
            ])
            self.csv_file.flush()
        
        # Log to console
        if abs(self.v_rover_pos) > 0.01 or avg_wheel_vel > 0.01:
            self.get_logger().info(
                f'Slip(Pos): {sum(self.slip_ratios_pos)/4:.2f} | '
                f'Slip(EKF): {sum(self.slip_ratios_ekf)/4:.2f} | '
                f'V(Pos): {self.v_rover_pos:.2f} | V(EKF): {self.v_rover_ekf:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SlipRatioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
