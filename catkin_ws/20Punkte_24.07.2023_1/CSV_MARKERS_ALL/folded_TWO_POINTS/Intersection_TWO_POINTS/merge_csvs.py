import os
import pandas as pd


def merge_csv_files(directory_path):
    merged_data = pd.DataFrame()  # Initialize an empty DataFrame to store the merged data

    csv_files = [file for file in os.listdir(directory_path) if file.endswith('.csv')]

    for idx, csv_file in enumerate(csv_files):
        if (csv_file.startswith("merged")):
            continue
        print(csv_file)
        csv_path = os.path.join(directory_path, csv_file)
        data = pd.read_csv(csv_path)
        data.to_csv('merged_Intersection.csv', mode='a', index=False, header=False)


if __name__ == "__main__":
    # Replace with the path to your directory containing CSV files
    directory_path = os.path.dirname(__file__)
    merge_csv_files(directory_path)
