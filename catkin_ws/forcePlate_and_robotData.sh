#!/bin/bash

#catkin_make

# Must be done in each terminal window.
source ./devel/setup.bash

konsole --new-tab -e "roscore" &

sleep 3s

# rosrun [packageName] [nodeName] 

konsole --new-tab -e "rosrun vicon_data_publisher Force_plate_data_publisher" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Force_plate_data_sma_transceiver.py" &

sleep 1s

konsole --new-tab -e "rosrun ur_robot_data_acquisition Ur_robot_data_publisher.py" &

sleep 1s

konsole --new-tab -e "rosrun plotjuggler plotjuggler" &

