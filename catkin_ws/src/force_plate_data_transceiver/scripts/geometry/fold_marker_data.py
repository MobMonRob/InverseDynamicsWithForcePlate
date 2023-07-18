import pandas as pd
import sys

def calculate_averages(data):
    avg_x = data['field.x_mm'].mean()
    avg_y = data['field.y_mm'].mean()
    avg_z = data['field.z_mm'].mean()
    return avg_x, avg_y, avg_z

def main():
    # Read the original CSV file
    file_path = 'marker_data.csv'
    data = pd.read_csv(file_path)

    # Filter the data based on marker numbers 0 and 1
    marker_numbers_01 = [0, 1]
    filtered_data_01 = data[data['field.markerNumber'].isin(marker_numbers_01)]

    # Calculate averages for marker numbers 0 and 1 grouped by frame number
    grouped_data_01 = filtered_data_01.groupby('field.frameNumber')
    folded_data_01 = pd.DataFrame(columns=['field.frameNumber', 'field.x_mm', 'field.y_mm', 'field.z_mm'])
    for frame_number, frame_data in grouped_data_01:
        avg_x, avg_y, avg_z = calculate_averages(frame_data)

        # Create a new row with the averaged values for marker numbers 0 and 1
        avg_frame_data_01 = pd.DataFrame({
            'field.frameNumber': [frame_number],
            'field.x_mm': [avg_x],
            'field.y_mm': [avg_y],
            'field.z_mm': [avg_z]
        })

        folded_data_01 = pd.concat([folded_data_01, avg_frame_data_01], ignore_index=True)

    # Save the folded data for marker numbers 0 and 1 to a new CSV file
    folded_file_path_01 = 'folded_marker_data_two_points_01.csv'
    folded_data_01.to_csv(folded_file_path_01, index=False)

    # Filter the data based on marker numbers 2 and 3
    marker_numbers_23 = [2, 3]
    filtered_data_23 = data[data['field.markerNumber'].isin(marker_numbers_23)]

    # Calculate averages for marker numbers 2 and 3 grouped by frame number
    grouped_data_23 = filtered_data_23.groupby('field.frameNumber')
    folded_data_23 = pd.DataFrame(columns=['field.frameNumber', 'field.x_mm', 'field.y_mm', 'field.z_mm'])
    for frame_number, frame_data in grouped_data_23:
        avg_x, avg_y, avg_z = calculate_averages(frame_data)

        # Create a new row with the averaged values for marker numbers 2 and 3
        avg_frame_data_23 = pd.DataFrame({
            'field.frameNumber': [frame_number],
            'field.x_mm': [avg_x],
            'field.y_mm': [avg_y],
            'field.z_mm': [avg_z]
        })

        folded_data_23 = pd.concat([folded_data_23, avg_frame_data_23], ignore_index=True)

    # Save the folded data for marker numbers 2 and 3 to a new CSV file
    folded_file_path_23 = 'folded_marker_data_two_points_23.csv'
    folded_data_23.to_csv(folded_file_path_23, index=False)

    # Read the CSV files
    file_path_01 = 'folded_marker_data_two_points_01.csv'
    file_path_23 = 'folded_marker_data_two_points_23.csv'
    folded_data_01 = pd.read_csv(file_path_01)
    folded_data_23 = pd.read_csv(file_path_23)

    # Merge the data into a single DataFrame
    merged_data = pd.concat([folded_data_01, pd.DataFrame(columns=['', '']), folded_data_23], axis=1)

    # Save the merged data to a new CSV file
    folded_file_path = 'folded_marker_data_two_points.csv'
    merged_data.to_csv(folded_file_path, index=False)

    print("Data saved to", folded_file_path)
    sys.stdout.flush()  # Flush the output buffer

if __name__ == "__main__":
    main()
