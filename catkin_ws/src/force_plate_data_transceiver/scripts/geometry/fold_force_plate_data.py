import pandas as pd

def calculate_averages(data):
    avg_fx = data['field.fx_N'].mean()
    avg_fy = data['field.fy_N'].mean()
    avg_fz = data['field.fz_N'].mean()
    avg_mx = data['field.mx_Nm'].mean()
    avg_my = data['field.my_Nm'].mean()
    avg_mz = data['field.mz_Nm'].mean()
    return avg_fx, avg_fy, avg_fz, avg_mx, avg_my, avg_mz

def main():
    file_path = 'force_plate_data.csv'
    data = pd.read_csv(file_path)

    # Calculate averages for each frame number
    grouped_data = data.groupby('field.frameNumber')
    folded_data = pd.DataFrame(columns=data.columns)
    for frame_number, frame_data in grouped_data:
        avg_fx, avg_fy, avg_fz, avg_mx, avg_my, avg_mz = calculate_averages(frame_data)

        # Create a new row with the averaged values for each frame number
        avg_frame_data = pd.DataFrame({
            'field.frameNumber': [frame_number],
            'field.fx_N': [avg_fx],
            'field.fy_N': [avg_fy],
            'field.fz_N': [avg_fz],
            'field.mx_Nm': [avg_mx],
            'field.my_Nm': [avg_my],
            'field.mz_Nm': [avg_mz]
        })

        folded_data = pd.concat([folded_data, avg_frame_data], ignore_index=True)

    # Save the folded data to a new CSV file
    folded_file_path = 'folded_force_plate_data.csv'
    folded_data.to_csv(folded_file_path, index=False)
    print("Data saved to", folded_file_path)

if __name__ == "__main__":
    main()
