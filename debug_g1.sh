#!/bin/bash
# G1 Robot Debugging Script
# Run this inside the docker container to debug G1 issues

echo "=== G1 Robot Debugging Tools ==="
echo ""

# Function to check if topics are publishing
check_topics() {
    echo "1. Checking ROS topics..."
    ros2 topic list | grep unitree_g1
    echo ""
}

# Function to check joint states
check_joints() {
    echo "2. Checking joint states (press Ctrl+C to stop)..."
    ros2 topic echo /unitree_g1/joint_states --once
    echo ""
}

# Function to check robot position
check_position() {
    echo "3. Checking robot position..."
    ros2 topic echo /unitree_g1/pose --once
    echo ""
}

# Function to check if robot is moving
check_movement() {
    echo "4. Checking robot movement..."
    ros2 topic echo /unitree_g1/odom --once
    echo ""
}

# Function to send test commands
send_test_command() {
    echo "5. Sending test forward command..."
    ros2 topic pub --once /unitree_g1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    echo "Command sent! Check if robot responds."
    sleep 2
    echo "Sending stop command..."
    ros2 topic pub --once /unitree_g1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    echo ""
}

# Function for continuous monitoring
continuous_monitor() {
    echo "6. Starting continuous position monitoring (press Ctrl+C to stop)..."
    while true; do
        echo -n "$(date '+%H:%M:%S') - Robot height: "
        ros2 topic echo /unitree_g1/pose --once | grep -A1 "position:" | grep "z:" | awk '{print $2}'
        sleep 1
    done
}

# Main menu
while true; do
    echo "Choose debugging option:"
    echo "  1) Check topics"
    echo "  2) Check joint states" 
    echo "  3) Check position"
    echo "  4) Check movement"
    echo "  5) Send test command"
    echo "  6) Continuous monitoring"
    echo "  7) Run interactive diagnostics"
    echo "  0) Exit"
    echo ""
    read -p "Enter choice (0-7): " choice
    
    case $choice in
        1) check_topics ;;
        2) check_joints ;;
        3) check_position ;;
        4) check_movement ;;
        5) send_test_command ;;
        6) continuous_monitor ;;
        7) python3 ros_diagnostics.py ;;
        0) echo "Goodbye!"; break ;;
        *) echo "Invalid choice!" ;;
    esac
    
    if [ "$choice" != "6" ] && [ "$choice" != "7" ]; then
        read -p "Press Enter to continue..."
        clear
    fi
done