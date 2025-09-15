#!/bin/bash

# Get the directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR

# Open Xhost for Isaac Sim GUI
xhost +local: > /dev/null 2>&1

cd ./docker

# Environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:=100}

# Create or overwrite the .env file with static settings
cat <<EOL > .env
# Docker Compose Settings
COMPOSE_PROJECT_NAME=isaac-go2-ros2
COMPOSE_BAKE=true

# ROS Settings
ROS_DISTRO=humble

# Dynamic UID and GID
# LOCAL_USER_ID=$(id -u)
# LOCAL_GROUP_ID=$(id -g)
LOCAL_USER_ID=1000
LOCAL_GROUP_ID=1000
EOL

# Ensure cleanup on script exit
cleanup() {
    if [[ "$ACTION" != "close" && "$ACTION" != "build" && "$ACTION" != "run" && "$ACTION" != "sim" ]]; then
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo "Container is still running. Use 'close' to stop it."
        fi
    elif [[ "$ACTION" == "close" ]]; then
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            docker-compose down
        fi
    fi
    xhost -local:docker > /dev/null 2>&1
    xhost -local: > /dev/null 2>&1
}
trap cleanup EXIT

# Argument handling
ACTION=${1:-}
CUSTOM_COMMAND=${2:-}

print_usage() {
    echo -e "\033[1;32m----- [ isaac-launch Usage ] ----------------\033[0m"
    echo -e "\033[1;32m|\033[0m   build           - Build the ROS workspace"
    echo -e "\033[1;32m|\033[0m   run             - Run Isaac simulation (Go2 or G1)"
    echo -e "\033[1;32m|\033[0m   g1-simple       - Run G1 simplified simulation"
    echo -e "\033[1;32m|\033[0m   g1-clean        - Run G1 CLEAN simulation (uses Go2 patterns)"
    echo -e "\033[1;32m|\033[0m   sim             - Launch Isaac Sim GUI"
    echo -e "\033[1;32m|\033[0m   enter           - Enter the container"
    echo -e "\033[1;32m|\033[0m   close           - Stop the container"
    echo -e "\033[1;32m|\033[0m   --command <cmd> - Run a custom command"
    echo -e "\033[1;32m|\033[0m"
    echo -e "\033[1;32m|\033[0m   Robot Selection (set in cfg/sim.yaml):"
    echo -e "\033[1;32m|\033[0m     robot_type: go2 - Unitree Go2 quadruped"
    echo -e "\033[1;32m|\033[0m     robot_type: g1  - Unitree G1 humanoid"
    echo -e "\033[1;32m---------------------------------------------\033[0m"
}

if [[ "$ACTION" == "--command" ]]; then
    if [[ -z "$CUSTOM_COMMAND" ]]; then
        echo "Error: No command specified for --command."
        exit 1
    fi
    if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
        echo "Container 'isaac-go2-ros2' is not running. Starting it..."
        docker-compose up -d
    fi
    docker-compose exec isaac-go2-ros2 bash -c "$CUSTOM_COMMAND"
    exit 0
fi

case $ACTION in
    build)
        export AUTO_BUILD=true AUTO_RUN=false
        echo "Building ROS workspace..."
        docker-compose up
        ;;
    run)
        export AUTO_BUILD=false AUTO_RUN=true
        echo "Running Isaac simulation (robot type configured in cfg/sim.yaml)..."
        docker-compose up
        docker-compose down
        ;;
    g1-simple)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Running G1 simplified simulation with proper environment..."
        if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo "Starting container..."
            docker-compose up -d
        fi
        docker-compose exec isaac-go2-ros2 bash /home/ubuntu/isaac-go2-ros2/run_g1_simple.sh
        echo "G1 simple simulation finished."
        ;;
    g1-clean)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Running G1 CLEAN simulation using Go2 patterns..."
        if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo "Starting container..."
            docker-compose up -d
        fi
        docker-compose exec isaac-go2-ros2 bash /home/ubuntu/isaac-go2-ros2/run_g1_clean.sh
        echo "G1 clean simulation finished."
        ;;
    sim)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Launching Isaac Sim GUI..."
        docker-compose up -d
        docker-compose exec isaac-go2-ros2 bash -c "/isaac-sim/isaac-sim.sh"
        echo "Isaac Sim process exited. Stopping the container..."
        docker-compose down
        ;;
    enter)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Entering the container..."
        if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo -e "Container 'isaac-go2-ros2' is not running.\nStarting without build or run..."
            docker-compose up -d
        fi
        docker exec -it --user ubuntu isaac-go2-ros2 bash || echo "Exited container without shutting it down."
        ;;
    close)
        export AUTO_BUILD=false AUTO_RUN=false
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2" || \
           docker ps --filter "name=isaac-go2-ros2" --filter "status=exited" | grep -q "isaac-go2-ros2"; then
            echo "Stopping the container..."
            docker-compose down
        else
            echo "Container is already stopped."
        fi
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
