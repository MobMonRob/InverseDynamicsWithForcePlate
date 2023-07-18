# This code is inspired by https://kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/

import numpy as np

class Plane3D:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.plane_equation = f"({a}) * x + ({b}) * y + ({c}) * z + ({d}) = 0"

def input_point(prompt):
    values = input(prompt).split()
    if len(values) != 3:
        raise ValueError("Enter three numbers separated by space.")
    x, y, z = map(float, values)
    return np.array([x, y, z])

def define_plane(point1, point2, point3):
    # These two vectors are in the plane
    v1 = point3 - point1
    v2 = point2 - point1

    # The cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a, b, c = cp

    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, point3)

    plane = Plane3D(a, b, c, d)
    #print(plane.plane_equation)

    return plane

def main():
    point1 = input_point("Enter the coordinates for point1 (x y z): ")
    point2 = input_point("Enter the coordinates for point2 (x y z): ")
    point3 = input_point("Enter the coordinates for point3 (x y z): ")

    return define_plane(point1, point2, point3)

if __name__ == "__main__":
    main()