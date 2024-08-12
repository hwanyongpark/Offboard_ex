#!/bin/bash

gnome-terminal --tab --title="Gazebo Simulator" -- bash -c "cd ~/PX4-Autopilot; make px4_sitl gz_x500; bash"

sleep 10

gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "MicroXRCEAgent udp4 -p 8888; bash"

sleep 30

gnome-terminal --tab --title="Offboard Node" -- bash -c "cd ~/ws_kari; source ./install/setup.bash; ros2 run offboard_ex offboard_node; bash"

echo "All processes have been launched!"
