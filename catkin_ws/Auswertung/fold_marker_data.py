import pandas as pd
import sys
import os


def calculate_averages(data):
    avg_x = data['field.x_m'].mean()
    avg_y = data['field.y_m'].mean()
    avg_z = data['field.z_m'].mean()
    return avg_x, avg_y, avg_z


def main(file_path):
    # Read the CSV file
    data = pd.read_csv(file_path)

    # Filter the data based on marker numbers 0 and 1
    first_pair = [4, 5]
    filtered_first_pair = data[data['field.markerNumber'].isin(first_pair)]

    # Calculate averages for marker numbers 0 and 1 grouped by frame number
    grouped_first_pair = filtered_first_pair.groupby('field.frameNumber')
    folded_data_first_pair = pd.DataFrame(columns=['field.frameNumber', 'field.x_m', 'field.y_m', 'field.z_m'])
    for frame_number, frame_data in grouped_first_pair:
        avg_x, avg_y, avg_z = calculate_averages(frame_data)

        # Create a new row with the averaged values for marker numbers 0 and 1
        avg_frame_data_first_pair = pd.DataFrame({
            'field.frameNumber': [frame_number],
            'field.x_m': [avg_x],
            'field.y_m': [avg_y],
            'field.z_m': [avg_z]
        })

        folded_data_first_pair = pd.concat([folded_data_first_pair, avg_frame_data_first_pair], ignore_index=True)

    # Filter the data based on marker numbers 2 and 3
    second_pair = [6, 7]
    filtered_data_second_pair = data[data['field.markerNumber'].isin(second_pair)]

    # Calculate averages for marker numbers 2 and 3 grouped by frame number
    grouped_data_second_pair = filtered_data_second_pair.groupby('field.frameNumber')
    folded_data_second_pair = pd.DataFrame(columns=['field.frameNumber', 'field.x_m', 'field.y_m', 'field.z_m'])
    for frame_number, frame_data in grouped_data_second_pair:
        avg_x, avg_y, avg_z = calculate_averages(frame_data)

        # Create a new row with the averaged values for marker numbers 2 and 3
        avg_frame_data_second_pair = pd.DataFrame({
            'field.frameNumber': [frame_number],
            'field.x_m': [avg_x],
            'field.y_m': [avg_y],
            'field.z_m': [avg_z]
        })

        folded_data_second_pair = pd.concat([folded_data_second_pair, avg_frame_data_second_pair], ignore_index=True)

    # Merge the data into a single DataFrame
    merged_data = pd.concat([folded_data_first_pair, pd.DataFrame(columns=['', '']), folded_data_second_pair], axis=1)

    file_name = os.path.basename(file_path).split('_')[0]

    # Save the folded data to a new CSV file under the "folded_CoP_FP_SMA" folder
    folder_name = "folded_TWO_POINTS"
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Save the merged data to a new CSV file
    folded_file_path = os.path.join(folder_name, f'folded_{file_name}_TWO_POINTS.csv')
    merged_data.to_csv(folded_file_path, index=False)

    print("Data saved to", folded_file_path)
    sys.stdout.flush()  # Flush the output buffer


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py <file_path>")
        sys.exit(1)

    file_path = sys.argv[1]
    main(file_path)
