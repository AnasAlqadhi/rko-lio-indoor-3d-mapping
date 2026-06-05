#!/bin/bash
# Setup robot network configuration - FIXED

echo "Setting up robot network..."

# Find actual ethernet interface
ETH_INTERFACE=$(ip link show | grep -E "^[0-9]+: (eth|enp|ens|eno)" | head -1 | cut -d: -f2 | tr -d ' ')

if [ -z "$ETH_INTERFACE" ]; then
    echo "Warning: No ethernet interface found, using existing network"
    ETH_INTERFACE="auto"
fi

echo "Using interface: $ETH_INTERFACE"

# Set robot Velodyne IP
ROBOT_VELODYNE_IP="192.168.4.201"  
ROBOT_HOST_IP="192.168.4.100"

# Only configure if interface exists and is not already configured
if [ "$ETH_INTERFACE" != "auto" ]; then
    # Check if IP is already configured
    if ! ip addr show $ETH_INTERFACE | grep -q "$ROBOT_HOST_IP"; then
        sudo ip addr replace $ROBOT_HOST_IP/24 dev $ETH_INTERFACE 2>/dev/null || echo "IP already configured or interface busy"
    fi
fi

# Test robot Velodyne connection
ping -c 3 $ROBOT_VELODYNE_IP

if [ $? -eq 0 ]; then
    echo "✓ Robot Velodyne connected successfully"
    export ROBOT_VELODYNE_IP=$ROBOT_VELODYNE_IP
else
    echo "✗ Robot Velodyne connection failed"
    echo "Check: 1) Velodyne power 2) Network cable 3) IP configuration"
    exit 1
fi
