#!/bin/bash
# =============================================================================
# setup_network.sh — ROS2 Multi-Machine Networking Setup
# =============================================================================
# Run this on BOTH machines (Orange Pi 5 and Desktop PC).
# Sets up ROS2 DDS networking for cross-machine communication.
#
# Usage:
#   source setup_network.sh
# Or add to ~/.bashrc for permanent setup.
# =============================================================================

# --- Configuration ---
# Both machines MUST use the same ROS_DOMAIN_ID (0-232)
export ROS_DOMAIN_ID=42

# Use CycloneDDS for better multi-machine support (recommended)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "=============================================="
echo " ROS2 Network Configuration"
echo "=============================================="
echo " ROS_DOMAIN_ID:      $ROS_DOMAIN_ID"
echo " RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

# Detect IP address on the active WiFi/ethernet interface
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo " Local IP:           $LOCAL_IP"
echo ""
echo " Both machines must be on the same WiFi network"
echo " and use the same ROS_DOMAIN_ID."
echo "=============================================="
