from varname import nameof

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Line3D:
    def __init__(self, point, vector, line_equation, parametrized_line_equation):
        self.point = point
        self.vector = vector
        self.line_equation = line_equation
        self.parametrized_line_equation = parametrized_line_equation

def input_point(prompt):
    values = input(prompt).split()
    if len(values) != 3:
        raise ValueError("Enter three numbers separated by space.")
    x, y, z = map(float, values)
    return Point3D(x, y, z)

def define_line(point1, point2):
    xTerm = f"(x - {point1.x}) / {point2.x - point1.x}"
    if point1.x < 0:
        xTerm = f"x + {-point1.x} / {point2.x - point1.x}"

    yTerm = f"(y - {point1.y}) / {point2.y - point1.y}"
    if point1.y < 0:
        yTerm = f"(y + {-point1.y}) / {point2.y - point1.y}"

    zTerm = f"(z - {point1.z}) / {point2.z - point1.z}"
    if point1.z < 0:
        zTerm = f"(z + {-point1.z}) / {point2.z - point1.z}"

    line_equation = f"Line equation: {xTerm} = {yTerm} = {zTerm}"
    return line_equation

def define_line_parametrized(point1, point2):
    xTerm = f"{point2.x - point1.x} * t + {point1.x}"
    if point1.x < 0:
        xTerm = f"{point2.x - point1.x} * t - {-point1.x}"

    yTerm = f"{point2.y - point1.y} * t + {point1.y}"
    if point1.y < 0:
        yTerm = f"{point2.y - point1.y} * t - {-point1.y}"

    zTerm = f"{point2.z - point1.z} * t + {point1.z}"
    if point1.z < 0:
        zTerm = f"{point2.z - point1.z} * t - {-point1.z}"

    parametrized_line_equation = f"Parametrized line equation: x = {xTerm}, y = {yTerm}, z = {zTerm}"
    return parametrized_line_equation

def main():
    point1 = input_point("Enter the coordinates for point1 (x y z): ")
    point2 = input_point("Enter the coordinates for point2 (x y z): ")

    vector = Point3D(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z)

    line_equation = define_line(point1, point2)
    parametrized_line_equation = define_line_parametrized(point1, point2)

    line = Line3D(point1, vector, line_equation, parametrized_line_equation)

    print(f"The line is defined by point {nameof(line.point)}: ({line.point.x}, {line.point.y}, {line.point.z})")
    print(f"and vector: ({line.vector.x}, {line.vector.y}, {line.vector.z})")
    print(line.line_equation)
    print(line.parametrized_line_equation)

    return line

if __name__ == "__main__":
    main()