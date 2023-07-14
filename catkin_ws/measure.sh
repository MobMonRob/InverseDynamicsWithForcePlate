#!/bin/bash

#catkin_make

# Must be done in each terminal window.
source ./devel/setup.bash

konsole --new-tab -e "roscore" &

sleep 3s

# rosrun [packageName] [nodeName] 

konsole --new-tab -e "rosrun vicon_data_publisher Force_plate_data_publisher" &

sleep 1s

konsole --new-tab -e "rosrun vicon_data_publisher Marker_data_publisher" &
