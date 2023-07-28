import pandas as pd
from geometry_classes import Point3D, Line3D

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
