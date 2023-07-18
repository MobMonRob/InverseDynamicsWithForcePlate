import pandas as pd
from sympy.solvers import solve
from sympy import Symbol
import numpy as np

from geometry.define_line import get_line_prompt, get_lines_from_csv, get_line_as_arguments, Point3D
from geometry.define_plane import define_plane, Plane3D

def define_lines_and_plane():
    lines = get_lines_from_csv()
    plane = define_plane(np.array([0, 0, 0]), np.array([0.6, 0, 0]), np.array([0, 0.4, 0]))
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

# Console version
def main():
    line = get_line_prompt()
    plane = define_plane(np.array([0, 0, 0]), np.array([0.6, 0, 0]), np.array([0, 0.4, 0]))
    intersection_point = find_intersection(line, plane)
    print(f"Intersection point: ({intersection_point.x}, {intersection_point.y}, {intersection_point.z})")

# Csv version
# def main():
#     lines, plane = define_lines_and_plane()
#     results = []
#     for line in lines:
#         intersection_point = find_intersection(line, plane)
#         results.append([line.frameNumber, intersection_point.x, intersection_point.y, intersection_point.z])

#     # Create a DataFrame from the results
#     df = pd.DataFrame(results, columns=['frameNumber', 'x', 'y', 'z'])

#     # Save the DataFrame to a CSV file
#     df.to_csv('CoP_vicon.csv', index=False)

if __name__ == "__main__":
    main()