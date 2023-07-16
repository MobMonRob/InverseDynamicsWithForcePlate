from sympy.solvers import solve
from sympy import Symbol
import numpy as np

from define_line import main as get_line, Point3D
from define_plane import main as define_plane, Plane3D

print("Define the line:")
line = get_line()

print()

# print("Define the plane:")
plane = define_plane(np.array([0, 0, 0]), np.array([0.6, 0, 0]), np.array([0, 0.4, 0]))
plane = Plane3D()

def substitute_variables(line, plane):
    substituted_equation = plane.plane_equation.replace('x', f"({line.xTermParametrized})")
    substituted_equation = substituted_equation.replace('y', f"({line.yTermParametrized})")
    substituted_equation = substituted_equation.replace('z', f"({line.zTermParametrized})")

    # Remove the last four symbols which are " = 0"
    substituted_equation = substituted_equation[:-4]
    return substituted_equation

def main():
    substituted_equation = substitute_variables(line, plane)

    # Calculate t parameter that will be used for calculation of intersection point
    t = Symbol('t')
    t_value = solve(substituted_equation, t)[0]

    # print(f"Parameter value is: {t_value}")

    locals_dict = {'t': t_value}
    x = eval(line.xTermParametrized, locals_dict)
    y = eval(line.yTermParametrized, locals_dict)
    z = eval(line.zTermParametrized, locals_dict)

    intersection_point = Point3D(x, y, z)
    # print("Intersection Point:")
    # print(intersection_point.pointExpression)

    return intersection_point