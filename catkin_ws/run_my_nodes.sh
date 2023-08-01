#!/bin/bash

#catkin_make

# Must be done in each terminal window.
source ./devel/setup.bash

konsole --new-tab -e "roscore" &

sleep 3s

# rosrun [packageName] [nodeName] 

# Caution: mock!!
# konsole --new-tab -e "rosrun vicon_data_publisher Force_plate_data_publisher_mock" &

konsole --new-tab -e "rosrun vicon_data_publisher Force_plate_data_publisher" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Force_plate_data_sma_transceiver.py" &

sleep 1s

konsole --new-tab -e "rosrun vicon_data_publisher Marker_data_publisher" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Marker_data_filter.py" &

sleep 1s

konsole --new-tab -e "rosrun plotjuggler plotjuggler" &

