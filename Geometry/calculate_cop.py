# Based on the thoughts of Alan Carman (allan.carman@otago.ac.nz, Centre for Health, Activity, and Rehabilitation Research Support Unit)
# https://drive.google.com/drive/folders/1bfN8hC_wq9LQO2sB6vjtVNFYpZkxI474

from varname import nameof
from define_line import Point3D

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
    #Tz_g = -Tz

    # Print the result
    # print(f"The value of {nameof(CoPx_g)} is: {CoPx_g}\n")
    # print(f"The value of {nameof(CoPy_g)} is: {CoPy_g}\n")
    # print(f"The value of {nameof(Tz_g)} is: {Tz_g}\n")

    return CoPx_g, CoPy_g, CoPz_g#, Tz_g

def main():
    # Get user input
    a, b, z0, h, fx, fy, fz, mx, my, mz, width, length = get_user_input()

    # Perform calculation
    CoPx_g, CoPy_g, CoPz_g = perform_calculation(a, b, z0, h, fx, fy, fz, mx, my, mz, width, length)
    point = Point3D(CoPx_g, CoPy_g, CoPz_g)
    return point#, Tz_g

if __name__ == "__main__":
    main()