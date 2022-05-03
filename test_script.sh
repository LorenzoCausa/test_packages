#!/bin/bash

xterm -e bash -c "roslaunch test_pkg test.launch ; exec bash" &
sleep 2
xterm -e bash -c "rosrun test_pkg img_listener_UDP.py ; exec bash" &
xterm -e bash -c "rosrun test_pkg UDP_server.py ; exec bash" &
xterm -e bash -c "rosrun test_pkg command_publisher.py ; exec bash" &
xterm -e bash -c "rosrun test_pkg test_publisher.py ; exec bash" &
echo "script finished"
