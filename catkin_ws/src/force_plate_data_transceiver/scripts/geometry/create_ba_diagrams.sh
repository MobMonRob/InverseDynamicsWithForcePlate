#!/bin/bash

# Set the paths to the directories containing .csv files
dirPath1="/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/20Punkte_24.07.2023_1/folded_CoP_FP_SMA/"
dirPath2="/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/20Punkte_24.07.2023_1/CSV_MARKERS_ALL/folded_TWO_POINTS/Intersection_TWO_POINTS/"

# Get the list of .csv files in dirPath1 and dirPath2
filesDir1="$dirPath1"/*.csv
filesDir2="$dirPath2"/*.csv

# Loop through the files in dirPath1 and dirPath2
for file1 in $filesDir1; do
    for file2 in $filesDir2; do
        # Get the file names without the extension
        file1Name=$(basename "$file1" .csv)
        file2Name=$(basename "$file2" .csv)

        # Extract the desired parts from the file names
        pointNumber=$(echo "$file1Name" | cut -d '_' -f 2)   # Extract characters between first two _
        intersectionPoint=$(echo "$file2Name" | cut -d '_' -f 2)   # Extract characters after the last _

        # Execute the bland-altman.py script for the current pair of files
        python3 bland-altman.py "$file1" "CoP Kraftmessplatte $pointNumber" "field.x_m" "$file2" "CoP Überschneidung $intersectionPoint" "field.x_m" "" "[mm]" 1000
    done
done
