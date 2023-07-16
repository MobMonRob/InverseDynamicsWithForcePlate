from varname import nameof

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.pointExpression = f"({x}, {y}, {z})"

class Line3D:
    def __init__(self, point, vector, xTerm, yTerm, zTerm, xTermParametrized, yTermParametrized, zTermParametrized):
        self.point = point
        self.vector = vector
        self.line_equation = f"{xTerm} = {yTerm} = {zTerm}"
        self.xTermParametrized = xTermParametrized
        self.yTermParametrized = yTermParametrized
        self.zTermParametrized = zTermParametrized
        self.parametrized_line_equation = f"x = {xTermParametrized}, y = {yTermParametrized}, z = {zTermParametrized}"

def input_point(prompt):
    values = input(prompt).split()
    if len(values) != 3:
        raise ValueError("Enter three numbers separated by space.")
    x, y, z = map(float, values)
    return Point3D(x, y, z)

def define_line(point1, point2):
    terms = ['x', 'y', 'z']
    line_terms = []

    for term in terms:
        p1 = getattr(point1, term)
        p2 = getattr(point2, term)

        term_line = f"({term} - {p1}) / {p2 - p1}"
        if p1 < 0:
            term_line = f"({term} + {-p1}) / {p2 - p1}"

        line_terms.append(term_line)

    return tuple(line_terms)

def define_line_parametrized(point1, point2):
    terms = ['x', 'y', 'z']
    parametrized = []

    for term in terms:
        p1 = getattr(point1, term)
        p2 = getattr(point2, term)

        term_parametrized = f"{p2 - p1} * t"
        if p1 < 0:
            term_parametrized += f" - {-p1}"
        elif p1 == 0:
            term_parametrized = f"{p2} * t"
        else:
            term_parametrized += f" + {p1}"

        parametrized.append(term_parametrized)

    return tuple(parametrized)

def main():
    point1 = input_point("Enter the coordinates for point1 (x y z): ")
    point2 = input_point("Enter the coordinates for point2 (x y z): ")

    vector = Point3D(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z)

    xTerm, yTerm, zTerm = define_line(point1, point2)
    xTermParametrized, yTermParametrized, zTermParametrized = define_line_parametrized(point1, point2)

    line = Line3D(point1, vector, xTerm, yTerm, zTerm, xTermParametrized, yTermParametrized, zTermParametrized)

    # print(f"The line is defined by point {nameof(line.point)}: ({line.point.x}, {line.point.y}, {line.point.z})")
    # print(f"and vector: ({line.vector.x}, {line.vector.y}, {line.vector.z})")
    
    #print(f"Canonical equation: {line.line_equation}")
    #print(f"Parametrized equation: {line.parametrized_line_equation}")

    return line

if __name__ == "__main__":
    main()