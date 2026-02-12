#!/bin/bash
# =============================================================================
# start_robot.sh — One-command robot bringup for Orange Pi 5
# =============================================================================
# Usage:
#   ./start_robot.sh slam       # Start robot + SLAM mapping
#   ./start_robot.sh nav        # Start robot + Nav2 navigation
#   ./start_robot.sh bringup    # Start robot hardware only (no SLAM/Nav2)
# =============================================================================

set -e

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# Network
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Device ports (using udev symlinks)
LIDAR_PORT="${LIDAR_PORT:-/dev/rplidar}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/arduino}"
MAP_FILE="${MAP_FILE:-$HOME/robot_ws/src/my_bot/config/my_real_map.yaml}"

MODE="${1:-bringup}"

echo "=============================================="
echo " Robot Startup — Mode: $MODE"
echo " Lidar:   $LIDAR_PORT"
echo " Arduino: $ARDUINO_PORT"
echo "=============================================="

# Check devices exist
if [ ! -e "$LIDAR_PORT" ]; then
    echo "ERROR: Lidar not found at $LIDAR_PORT"
    echo "  Check: ls /dev/ttyUSB*"
    echo "  Install udev rules: sudo cp ~/robot_ws/src/my_bot/config/99-robot-devices.rules /etc/udev/rules.d/"
    exit 1
fi

if [ ! -e "$ARDUINO_PORT" ]; then
    echo "ERROR: Arduino not found at $ARDUINO_PORT"
    echo "  Check: ls /dev/ttyUSB*"
    exit 1
fi

case "$MODE" in
    bringup)
        echo "Starting robot hardware bringup only..."
        ros2 launch my_bot bringup_hardware.launch.py \
            lidar_port:="$LIDAR_PORT" \
            arduino_port:="$ARDUINO_PORT"
        ;;

    slam)
        echo "Starting robot + SLAM mapping..."
        echo "Launch bringup in background, then SLAM..."
        ros2 launch my_bot bringup_hardware.launch.py \
            lidar_port:="$LIDAR_PORT" \
            arduino_port:="$ARDUINO_PORT" &
        BRINGUP_PID=$!
        sleep 5
        ros2 launch my_bot slam_hardware.launch.py &
        SLAM_PID=$!
        echo "Bringup PID: $BRINGUP_PID, SLAM PID: $SLAM_PID"
        echo "Press Ctrl+C to stop..."
        wait
        ;;

    nav)
        echo "Starting robot + Navigation..."
        if [ ! -f "$MAP_FILE" ]; then
            echo "ERROR: Map file not found: $MAP_FILE"
            echo "  Run SLAM first to create a map."
            exit 1
        fi
        ros2 launch my_bot bringup_hardware.launch.py \
            lidar_port:="$LIDAR_PORT" \
            arduino_port:="$ARDUINO_PORT" &
        BRINGUP_PID=$!
        sleep 5
        ros2 launch my_bot navigation_hardware.launch.py \
            map:="$MAP_FILE" &
        NAV_PID=$!
        echo "Bringup PID: $BRINGUP_PID, Nav PID: $NAV_PID"
        echo "Press Ctrl+C to stop..."
        wait
        ;;

    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 {bringup|slam|nav}"
        exit 1
        ;;
esac
