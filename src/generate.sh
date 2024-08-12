cd
cd PX4-Autopilot

gnome-terminal --tab -- sh -c "make px4_sitl gz_x500; bash"
sleep 10

gnome-terminal --tab -- sh -c "MicroXRCEAgent udp4 -p 8888; bash"