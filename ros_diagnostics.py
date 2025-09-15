#!/usr/bin/env python3
"""
ROS2 diagnostic script to monitor G1 robot state and debug issues.
Run this inside the docker container while the simulation is running.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import sys
import time

class G1Diagnostics(Node):
    def __init__(self):
        super().__init__('g1_diagnostics')
        
        # Subscribers for monitoring
        self.joint_sub = self.create_subscription(
            JointState, '/unitree_g1/joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/unitree_g1/odom', self.odom_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/unitree_g1/pose', self.pose_callback, 10)
            
        # Publisher for sending test commands
        self.cmd_pub = self.create_publisher(Twist, '/unitree_g1/cmd_vel', 10)
        
        # Data storage
        self.last_joint_state = None
        self.last_odom = None
        self.last_pose = None
        self.start_time = time.time()
        
        print("G1 Diagnostics Node Started")
        print("=" * 50)
        print("Available commands:")
        print("  'w' - Forward")
        print("  's' - Backward") 
        print("  'a' - Left strafe")
        print("  'd' - Right strafe")
        print("  'z' - Turn left")
        print("  'c' - Turn right")
        print("  'x' - Stop")
        print("  'q' - Quit")
        print("=" * 50)
    
    def joint_callback(self, msg):
        self.last_joint_state = msg
        
    def odom_callback(self, msg):
        self.last_odom = msg
        
    def pose_callback(self, msg):
        self.last_pose = msg
    
    def print_status(self):
        """Print current robot status."""
        current_time = time.time() - self.start_time
        print(f"\n--- G1 Status at {current_time:.1f}s ---")
        
        # Joint state info
        if self.last_joint_state:
            print(f"Joints: {len(self.last_joint_state.position)} positions received")
            # Show key joint positions
            joint_names = self.last_joint_state.name
            positions = self.last_joint_state.position
            
            key_joints = ['left_hip_pitch_joint', 'right_hip_pitch_joint', 
                         'left_knee_joint', 'right_knee_joint',
                         'left_ankle_pitch_joint', 'right_ankle_pitch_joint']
            
            for joint_name in key_joints:
                if joint_name in joint_names:
                    idx = joint_names.index(joint_name)
                    print(f"  {joint_name}: {positions[idx]:.3f}")
        else:
            print("No joint state data received")
        
        # Pose info  
        if self.last_pose:
            pos = self.last_pose.pose.position
            print(f"Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            
            # Check if robot has fallen
            if pos.z < 0.5:
                print("⚠️  WARNING: Robot height is low - possible fall!")
            elif pos.z > 1.0:
                print("⚠️  WARNING: Robot height is high - possible instability!")
        else:
            print("No pose data received")
            
        # Odometry info
        if self.last_odom:
            vel = self.last_odom.twist.twist.linear
            ang_vel = self.last_odom.twist.twist.angular
            print(f"Velocity: vx={vel.x:.3f}, vy={vel.y:.3f}, vz={vel.z:.3f}")
            print(f"Angular vel: wx={ang_vel.x:.3f}, wy={ang_vel.y:.3f}, wz={ang_vel.z:.3f}")
        else:
            print("No odometry data received")
    
    def send_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Send velocity command to robot."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        print(f"Sent command: vx={linear_x:.1f}, vy={linear_y:.1f}, wz={angular_z:.1f}")

def main():
    rclpy.init()
    diagnostics = G1Diagnostics()
    
    # Start a timer for periodic status updates
    timer = diagnostics.create_timer(2.0, diagnostics.print_status)
    
    print("Monitoring G1 robot... Press Enter for command mode or Ctrl+C to exit")
    
    try:
        # Interactive command mode
        while True:
            try:
                # Spin once to process callbacks
                rclpy.spin_once(diagnostics, timeout_sec=0.1)
                
                # Check for keyboard input (non-blocking)
                import select
                import sys
                
                if select.select([sys.stdin], [], [], 0)[0]:
                    command = input().lower().strip()
                    
                    if command == 'q':
                        break
                    elif command == 'w':
                        diagnostics.send_command(linear_x=0.5)
                    elif command == 's':
                        diagnostics.send_command(linear_x=-0.5)
                    elif command == 'a':
                        diagnostics.send_command(linear_y=0.5)
                    elif command == 'd':
                        diagnostics.send_command(linear_y=-0.5)
                    elif command == 'z':
                        diagnostics.send_command(angular_z=0.8)
                    elif command == 'c':
                        diagnostics.send_command(angular_z=-0.8)
                    elif command == 'x':
                        diagnostics.send_command()
                    else:
                        print("Unknown command. Use w/s/a/d/z/c/x/q")
                        
            except KeyboardInterrupt:
                break
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Send stop command
        diagnostics.send_command()
        diagnostics.destroy_node()
        rclpy.shutdown()
        print("Diagnostics node shut down.")

if __name__ == '__main__':
    main()