#!/bin/bash

# Set the paths to the directories containing .csv files
dirPath1="/path/to/dir1"
dirPath2="/path/to/dir2"

# Get the list of .csv files in dirPath1 and dirPath2
filesDir1="$dirPath1"/*.csv
filesDir2="$dirPath2"/*.csv

# Loop through the files in dirPath1 and dirPath2
for file1 in $filesDir1; do
    for file2 in $filesDir2; do
        # Get the file names without the extension
        file1Name=$(basename "$file1" .csv)
        file2Name=$(basename "$file2" .csv)

        # Execute the bland-altman.py script for the current pair of files
        python bland-altman.py "$file1" "CoP Kraftmessplatte" "field.x_m" "$file2" "CoP Überschneidung" "field.x_m" "" "[mm]" 1000
    done
done
 
