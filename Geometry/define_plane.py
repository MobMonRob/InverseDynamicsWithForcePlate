import numpy as np

class Plane:
    def __init__(self, a, b, c, d, plane_equation):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.plane_equation = plane_equation

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

    plane_equation = 'The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d)
    print(plane_equation)

    return Plane(a, b, c, d, plane_equation)

def main():
    point1 = input_point("Enter the coordinates for point1 (x y z): ")
    point2 = input_point("Enter the coordinates for point2 (x y z): ")
    point3 = input_point("Enter the coordinates for point3 (x y z): ")

    return define_plane(point1, point2, point3)

if __name__ == "__main__":
    main()