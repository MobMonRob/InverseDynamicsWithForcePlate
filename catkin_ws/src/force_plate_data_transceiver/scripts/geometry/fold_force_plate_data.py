import pandas as pd

def calculate_averages(data):
    avg_x_m = data['field.x_m'].mean()
    avg_y_m = data['field.y_m'].mean()
    avg_z_m = data['field.z_m'].mean()
    return avg_x_m, avg_y_m, avg_z_m

def main():
    filename = 'CoP_force_plate_sma'
    file_path = filename + ".csv"
    data = pd.read_csv(file_path)

    # Calculate averages for each frame number
    grouped_data = data.groupby('field.frameNumber')
    folded_data = pd.DataFrame(columns=data.columns)
    for frame_number, frame_data in grouped_data:
        avg_x_m, avg_y_m, avg_z_m = calculate_averages(frame_data)

        # Create a new row with the averaged values for each frame number
        avg_frame_data = pd.DataFrame({
            '%time': "",
            'field.frameNumber': [frame_number],
            'field.x_m': [avg_x_m],
            'field.y_m': [avg_y_m],
            'field.z_m': [avg_z_m]
        })

        folded_data = pd.concat([folded_data, avg_frame_data], ignore_index=True)

    # Save the folded data to a new CSV file
    folded_file_path = "folded_" + filename + ".csv"
    folded_data.to_csv(folded_file_path, index=False)
    print("Data saved to", folded_file_path)

if __name__ == "__main__":
    main()
