#!/bin/bash

#catkin_make

# Must be done in each terminal window.
source ./devel/setup.bash

konsole --new-tab -e "roscore" &

sleep 3s

# rosrun [packageName] [nodeName] 

# rosbag play -l <bagfile>

konsole --new-tab -e "rosrun data_transformation Force_plate_data_1euro_filter.py" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Inverse_dynamics.py" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Id_normed.py" &

sleep 1s

konsole --new-tab -e "rosrun plotjuggler plotjuggler" &

