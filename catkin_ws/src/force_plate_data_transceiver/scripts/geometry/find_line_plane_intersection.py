import sys  # nopep8
import os  # nopep8
sys.path.append(
    "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/catkin_ws/src/force_plate_data_transceiver/scripts/")  # nopep8

import pandas as pd
from sympy.solvers import solve
from sympy import Symbol
import numpy as np

from geometry.define_line import get_lines_from_csv
from geometry.define_plane import define_plane
from geometry_classes import Point3D

def define_lines_and_plane(pathToCsv):
    lines = get_lines_from_csv(pathToCsv)
    plane = define_plane(np.array([-0.019 / 1000, 0.124 / 1000, -0.002 / 1000]),
                         np.array([0.6, -0.037 / 1000, -0.037 / 1000]),
                         np.array([-0.102 / 1000, 0.434847, 0.1321 / 1000]))
    return lines, plane


def substitute_variables(line, plane):
    substituted_equation = plane.plane_equation.replace('x', f"({line.xTermParametrized})")
    substituted_equation = substituted_equation.replace('y', f"({line.yTermParametrized})")
    substituted_equation = substituted_equation.replace('z', f"({line.zTermParametrized})")

    # Remove the last four symbols which are " = 0"
    substituted_equation = substituted_equation[:-4]
    return substituted_equation


def find_intersection(line, plane):
    substituted_equation = substitute_variables(line, plane)

    # Calculate t parameter that will be used for calculation of intersection point
    t = Symbol('t')
    t_value = solve(substituted_equation, t)[0]

    # print(f"Parameter value is: {t_value}")

    locals_dict = {'t': t_value}
    x = eval(line.xTermParametrized, locals_dict)
    y = eval(line.yTermParametrized, locals_dict)
    z = eval(line.zTermParametrized, locals_dict)

    return Point3D(x, y, z)


def main(filePath):
    lines, plane = define_lines_and_plane(filePath)
    results = []
    for line in lines:
        intersection_point = find_intersection(line, plane)
        results.append([line.frameNumber, intersection_point.x, intersection_point.y, intersection_point.z])

    # Create a DataFrame from the results
    df = pd.DataFrame(results, columns=['field.frameNumber', 'field.x_m', 'field.y_m', 'field.z_m'])

    # Save the folded data to a new CSV file under the "folded_CoP_FP_SMA" folder
    folder_name = "Intersection_TWO_POINTS"
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    file_name = os.path.basename(filePath).split('_')[1]
    intersection_csv = os.path.join(folder_name, f'intersection_by_{file_name}.csv')

    # Save the DataFrame to a CSV file
    df.to_csv(intersection_csv, index=False)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py <filePathToCsv>")
        sys.exit(1)

    filePathToCsv = sys.argv[1]
    main(filePathToCsv)
