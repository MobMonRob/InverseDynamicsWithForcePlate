 #!/bin/bash

 #catkin_make

# Must be done in each terminal window.
source ./devel/setup.bash

konsole --new-tab -e "roscore" &

sleep 3s

konsole --new-tab -e "rosbag record --output-name=2023-07-14-16-45-49_filtered.bag CoP_force_plate_filter CoP_marker_filter" &

sleep 1s

konsole --new-tab -e "rosbag play --wait-for-subscribers --loop ./rosbags/2023-07-14-16-45-49.bag --topics /Force_plate_data " &

sleep 1s

konsole --new-tab -e "rosbag play --wait-for-subscribers --loop ./rosbags/2023-07-14-16-45-49.bag --topics /Marker_global_translation " &

sleep 1s

konsole --new-tab -e "rosrun data_transformation CoP_data_filter.py" &

sleep 1s

konsole --new-tab -e "rosrun data_transformation Force_plate_data_sma_transceiver.py" &

