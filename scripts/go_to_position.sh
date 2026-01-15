#!/bin/bash
# Simple script to send robot to position

cd /home/boris/ros2_ws
source install/setup.bash

X=$1
Y=$2
THETA=$3

if [ -z "$X" ] || [ -z "$Y" ] || [ -z "$THETA" ]; then
    echo "Usage: go_to_position.sh <x> <y> <theta_radians>"
    echo "Example: go_to_position.sh 0.51 -0.24 -0.418879"
    exit 1
fi

echo "Sending robot to position: x=$X, y=$Y, theta=$THETA"
python3 scripts/send_robot_to_position.py $X $Y $THETA --frame_id odom

