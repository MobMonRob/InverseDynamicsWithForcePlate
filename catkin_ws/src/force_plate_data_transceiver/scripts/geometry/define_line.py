import pandas as pd


class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.pointExpression = f"({x}, {y}, {z})"


class Line3D:
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
        self.vector = Point3D(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z)
        self.xTermParametrized = f"{self.point2.x - self.point1.x} * t + {self.point1.x}"
        self.yTermParametrized = f"{self.point2.y - self.point1.y} * t + {self.point1.y}"
        self.zTermParametrized = f"{self.point2.z - self.point1.z} * t + {self.point1.z}"
        self.line_equation = self._define_line_equation()
        self.parametrized_line_equation = self._define_parametrized_line_equation()

    def _define_line_equation(self):
        xTerm = f"({self.point1.x} - x) / ({self.point2.x - self.point1.x})"
        yTerm = f"({self.point1.y} - y) / ({self.point2.y - self.point1.y})"
        zTerm = f"({self.point1.z} - z) / ({self.point2.z - self.point1.z})"
        return f"{xTerm} = {yTerm} = {zTerm}"

    def _define_parametrized_line_equation(self):
        xTermParametrized = f"{self.point2.x - self.point1.x} * t + {self.point1.x}"
        yTermParametrized = f"{self.point2.y - self.point1.y} * t + {self.point1.y}"
        zTermParametrized = f"{self.point2.z - self.point1.z} * t + {self.point1.z}"
        return f"x = {xTermParametrized}, y = {yTermParametrized}, z = {zTermParametrized}"


def input_point(prompt):
    values = input(prompt).split()
    if len(values) != 3:
        raise ValueError("Enter three numbers separated by space.")
    x, y, z = map(float, values)
    return Point3D(x, y, z)


def input_point_from_csv(value1, value2, value3):
    return Point3D(value1, value2, value3)


def get_line_as_arguments(point1, point2):
    return Line3D(point1, point2)


def get_line_prompt():
    point1 = input_point("Enter the coordinates for point1 (x y z): ")
    point2 = input_point("Enter the coordinates for point2 (x y z): ")
    return Line3D(point1, point2)


def get_lines_from_csv(file_path):
    if (file_path == ""):
        file_path = 'folded_marker_data_two_points.csv'
    data = pd.read_csv(file_path)
    lines = []

    for _, row in data.iterrows():
        point1 = Point3D(row[1], row[2], row[3])
        point2 = Point3D(row[7], row[8], row[9])
        line = Line3D(point1, point2)
        line.frameNumber = row[0]
        lines.append(line)

    return lines


def main():
    lines = get_lines_from_csv("")

    # Print the lines for verification
    for i, line in enumerate(lines):
        print(f"Line {i+1}:")
        print(f"Point1: X: {line.point1.x}, Y: {line.point1.y}, Z: {line.point1.z}")
        print(f"Point2: X: {line.point2.x}, Y: {line.point2.y}, Z: {line.point2.z}")
        print(f"Vector: X: {line.vector.x}, Y: {line.vector.y}, Z: {line.vector.z}")
        print(f"Canonical equation: {line.line_equation}")
        print(f"Parametrized equation: {line.parametrized_line_equation}")


if __name__ == "__main__":
    main()
