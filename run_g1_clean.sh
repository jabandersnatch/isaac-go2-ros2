#!/bin/bash

# G1 Clean Runner - Sets up proper environment like the main entrypoint

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/home/ubuntu/isaac-go2-ros2/install/setup.bash" ]; then
    source "/home/ubuntu/isaac-go2-ros2/install/setup.bash"
fi

# Set RMW implementation for Isaac Sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Activate isaaclab conda environment
if [ -f "/home/ubuntu/miniconda3/etc/profile.d/conda.sh" ]; then
    source "/home/ubuntu/miniconda3/etc/profile.d/conda.sh"
    conda activate isaaclab
fi

# Isaac Sim specific environment variables
export ACCEPT_EULA=Y
export PRIVACY_CONSENT=N

# Change to the correct directory
cd /home/ubuntu/isaac-go2-ros2

echo "Running G1 CLEAN simulation using Go2 patterns..."
python isaac_g1_clean.py