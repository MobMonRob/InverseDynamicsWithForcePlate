#!/bin/bash

# Set the path to the directory containing .bag files
bagsPath="/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/20Punkte_24.07.2023_1/"

# Set the path to the directory containing fold_force_plate_data.py script
scriptDir=$(realpath .)
csvOutputDir="CSV_MARKERS_ALL"
# # Task 0: Process .bag files and create CSVs
# cd "$bagsPath" || exit
# count=1
# csvOutputDir="CSV_MARKERS_ALL"
# if [ -d "$csvOutputDir" ]; then
#     rm -r "$csvOutputDir"/* # Clear the contents of the folder
# else
#     mkdir "$csvOutputDir" # Create the CSV folder if it doesn't exist
# fi

# for file in *.bag; do
#     outputCSV="${csvOutputDir}/P${count}_MARKERS_ALL.csv"
#     rostopic echo -b "$file" -p /Marker_global_translation > "$outputCSV"
#     ((count++))
# done

# # Task 1: Execute fold_marker_data.py for each .csv file in ${bagsPath}/CSV/
# cd "$scriptDir" || exit
# cd "$bagsPath/$csvOutputDir" || exit
# find . -type f -name "*.csv" -print0 | while IFS= read -r -d $'\0' csvFile; do
#     python3 $scriptDir/fold_marker_data.py "$csvFile"
# done

# Task 2: Execute find_line_plane_intersection.py for each .csv file in ${bagsPath}/CSV_MARKERS_ALL/
cd "$scriptDir" || exit
cd "$bagsPath/$csvOutputDir/folded_TWO_POINTS" || exit
find . -type f -name "*.csv" -print0 | while IFS= read -r -d $'\0' csvFile; do
    python3 "$scriptDir/find_line_plane_intersection.py" "$csvFile"
done
