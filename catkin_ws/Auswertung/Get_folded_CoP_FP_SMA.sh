#!/bin/bash

# Set the path to the directory containing .bag files
bagsPath="/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/20Punkte_24.07.2023_1/"

# Set the path to the directory containing fold_force_plate_data.py script
scriptDir=$(realpath .)

# Task 0: Process .bag files and create CSVs
cd "$bagsPath" || exit
count=1
csvOutputDir="CSV_CoP_FP_SMA"
if [ -d "$csvOutputDir" ]; then
    rm -r "$csvOutputDir"/* # Clear the contents of the CSV folder
else
    mkdir "$csvOutputDir" # Create the CSV folder if it doesn't exist
fi

for file in *.bag; do
    outputCSV="${csvOutputDir}/P${count}_FP_SMA.csv"
    rostopic echo -b "$file" -p /CoP_force_plate_sma > "$outputCSV"
    ((count++))
done

# Task 1: Execute fold_force_plate_data.py for each .csv file in ${bagsPath}/CSV/
cd "$scriptDir" || exit
cd "$bagsPath/$csvOutputDir" || exit
find . -type f -name "*.csv" -print0 | while IFS= read -r -d $'\0' csvFile; do
    python3 $scriptDir/fold_force_plate_data.py "$csvFile"
done
