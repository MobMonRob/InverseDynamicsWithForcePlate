# Based on the thoughts of Alan Carman (allan.carman@otago.ac.nz, Centre for Health, Activity, and Rehabilitation Research Support Unit)
# https://drive.google.com/drive/folders/1bfN8hC_wq9LQO2sB6vjtVNFYpZkxI474

from varname import nameof
from geometry.define_line import Point3D
import pandas as pd


def read_csv_data(file_path):
    data = pd.read_csv(file_path)
    return data


def get_user_input():
    # Get input values from the user
    # For AMTI force plates bms400600 these values are all 0
    a, b, z0 = 0, 0, 0
    # a, b, z0 = map(float, input(f"Enter the values of {nameof(a)}, {nameof(b)} and {nameof(z0)} (separated by a space): ").split())

    # Enter thickness of any additional flooring material
    h = 0.045  # Meters
    # h = float(input(f"Enter the value of {nameof(h)}: "))

    # Enter forces in [Newtons]
    fx, fy, fz = 0, 0, 0
    fx, fy, fz = map(float, input(f"Enter the values of {nameof(fx)}, {nameof(fy)} and {nameof(fz)} (separated by a space): ").split())

    # Enter moments in [Newtons*Meters]
    mx, my, mz = 0, 0, 0
    mx, my, mz = map(float, input(f"Enter the values of {nameof(mx)}, {nameof(my)}, and {nameof(mz)} (separated by a space): ").split())

    # Get force plate width and length (in my convention width is the smaller number, length is the larger number):
    width, length = 0.4, 0.6  # [Meters], For AMTI BP400600 model in labor
    # width, length = map(float, input(f"Enter the values of {nameof(width)} and {nameof(length)} (separated by a space): ").split())

    return a, b, z0, h, fx, fy, fz, mx, my, mz, width, length


def perform_calculation(a, b, z0, h, fx, fy, fz, mx, my, mz, width, length):
    # Calculate CoPx in [Meters]
    CoPx = ((-my + fx * (z0 + h)) / fz) - a

    # Calculate CoPy in [Meters]
    CoPy = ((mx + fy * (z0 + h)) / fz) - b

    # Calculate torsional (or frictional) moment in [Newtons*Meters]
    # This moment is assumed to have no components along the X and Y plate axes
    # Tz = mz + fx * (CoPy + b) - fy * (CoPx + a)

    # Print the result
    # print(f"The value of {nameof(CoPx)} is: {CoPx}\n")
    # print(f"The value of {nameof(CoPy)} is: {CoPy}\n")

    # For fixed rotational transformation matrix from local force plate frame to global frame in the plate corner:
    #     (0 1  0)
    # R = (1 0  0)
    #     (0 0 -1)

    CoPx_g = length / 2 + CoPy
    CoPy_g = width / 2 + CoPx
    CoPz_g = 0
    # Tz_g = -Tz

    # Print the result
    # print(f"The value of {nameof(CoPx_g)} is: {CoPx_g}\n")
    # print(f"The value of {nameof(CoPy_g)} is: {CoPy_g}\n")
    # print(f"The value of {nameof(Tz_g)} is: {Tz_g}\n")

    return CoPx_g, CoPy_g, CoPz_g  # , Tz_g


def get_cop_as_arguments(fx, fy, fz, mx, my, mz) -> Point3D:
    a = 0
    b = 0
    z0 = 0
    h = 0.045 - 15.59 / 1000  # [Meter]
    width = 0.4  # [Meter]
    length = 0.6  # [Meter]

    # Perform calculation
    CoPx_g, CoPy_g, CoPz_g = perform_calculation(a, b, z0, h, fx, fy, fz, mx, my, mz, width, length)
    point = Point3D(CoPx_g, CoPy_g, CoPz_g)
    return point


def get_cop_prompt():
    # Get user input
    a, b, z0, h, fx, fy, fz, mx, my, mz, width, length = get_user_input()

    # Perform calculation
    CoPx_g, CoPy_g, CoPz_g = perform_calculation(a, b, z0, h, fx, fy, fz, mx, my, mz, width, length)
    point = Point3D(CoPx_g, CoPy_g, CoPz_g)
    return point


def main():
    point = get_cop_prompt()
    print(f"x = {point.x}; y = {point.y}")

# def main():
#     file_path = 'folded_force_plate_data.csv'  # Provide the correct file path
#     data = read_csv_data(file_path)

#     # Get other input values
#     a, b, z0, h = 0, 0, 0, 0.045  # Provide the desired values
#     width, length = 0.4, 0.6  # [Meters], For AMTI BP400600 model in labor

#     # Perform calculation for each row
#     results = []
#     for _, row in data.iterrows():
#         fx = row['field.fx_N']
#         fy = row['field.fy_N']
#         fz = row['field.fz_N']
#         mx = row['field.mx_Nm']
#         my = row['field.my_Nm']
#         mz = row['field.mz_Nm']

#         CoPx_g, CoPy_g, CoPz_g = perform_calculation(a, b, z0, h, fx, fy, fz, mx, my, mz, width, length)
#         point = Point3D(CoPx_g, CoPy_g, CoPz_g)
#         point.frame_number = row['field.frameNumber']
#         results.append(point)

#     # Create a DataFrame from the results
#     result_df = pd.DataFrame({'frameNumber': [point.frame_number for point in results],
#                               'x': [point.x for point in results],
#                               'y': [point.y for point in results],
#                               'z': [point.z for point in results]})

#     # Save the DataFrame to a new CSV file
#     result_df.to_csv('CoP_force_plate.csv', index=False)


if __name__ == "__main__":
    main()
