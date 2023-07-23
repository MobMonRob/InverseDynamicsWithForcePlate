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

sleep 1s

konsole --new-tab -e "rosrun force_plate_data_transceiver Force_plate_data_sma_transceiver.py" &

sleep 1s

konsole --new-tab -e "rosrun force_plate_data_transceiver CoP_force_plate_sma.py" &

sleep 1s

#konsole --new-tab -e "rosrun force_plate_data_transceiver CoP_marker.py" &

