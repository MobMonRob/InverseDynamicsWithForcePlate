import numpy as np
from Common.geometry_classes import Point3D


def normalize_vector(vec):
    magnitude = np.linalg.norm(vec)
    return vec / magnitude


def get_base_vectors(o, a, b):
    e1 = normalize_vector(np.array([a.x - o.x, a.y - o.y, a.z - o.z]))
    e3 = normalize_vector(np.cross(e1, [b.x - o.x, b.y - o.y, b.z - o.z]))
    e2 = normalize_vector(np.cross(e3, e1))
    return e1, e2, e3


def calculate_new_coordinates(o, a, b, s):
    e1, e2, e3 = get_base_vectors(o, a, b)

    a = np.array([[e1[0], e2[0], e3[0]],
                  [e1[1], e2[1], e3[1]],
                  [e1[2], e2[2], e3[2]]])

    s_old_Minus_o_old = np.array([s.x - o.x, s.y - o.y, s.z - o.z])
    s_new = np.dot(a, s_old_Minus_o_old)

    return Point3D(s_new[0], s_new[1], s_new[2])


def calculate_old_coordinates(o, a, b, s):
    e1, e2, e3 = get_base_vectors(o, a, b)

    a = np.linalg.inv(np.array([[e1[0], e2[0], e3[0]],
                                [e1[1], e2[1], e3[1]],
                                [e1[2], e2[2], e3[2]]]))

    A_mult_S_new = np.dot(a, np.array([s.x, s.y, s.z]))
    s_old = np.array([A_mult_S_new[0] + o.x, A_mult_S_new[1] + o.y, A_mult_S_new[2] + o.z])
    return Point3D(s_old[0], s_old[1], s_old[2])


def define_middle_point(p1: Point3D, p2: Point3D, p3: Point3D, p4: Point3D) -> Point3D:
    middlePoint: Point3D = Point3D(
        (p1.x + p2.x + p3.x + p4.x) / 4,
        (p1.y + p2.y + p3.y + p4.y) / 4,
        (p1.z + p2.z + p3.z + p4.z) / 4 - (7 + 2.02) / 1000
    )

    return middlePoint


if __name__ == "__main__":
    marker1_circle: Point3D = Point3D(92.8505 / 1000, 199.1685 / 1000, 69.1085 / 1000)
    marker2_circle: Point3D = Point3D(125.3575 / 1000, 169.0835 / 1000, 69.0475 / 1000)
    marker3_circle: Point3D = Point3D(155.8285 / 1000, 199.5767 / 1000, 69.153 / 1000)
    marker4_circle: Point3D = Point3D(124.7485 / 1000, 229.9881 / 1000, 69.4205 / 1000)
    middlePoint_circle: Point3D = define_middle_point(marker1_circle, marker2_circle, marker3_circle, marker4_circle)
    print(f"Midle point circrle: ({middlePoint_circle.x}; {middlePoint_circle.y}; {middlePoint_circle.z})")

    marker1_rectangle: Point3D = Point3D(462.4623 / 1000, 137.7025 / 1000, 69.1475 / 1000)
    marker2_rectangle: Point3D = Point3D(557.2875 / 1000, 137.5661 / 1000, 69.4235 / 1000)
    marker3_rectangle: Point3D = Point3D(557.0805 / 1000, 266.6615 / 1000, 70.3195 / 1000)
    marker4_rectangle: Point3D = Point3D(463.0825 / 1000, 267.0355 / 1000, 70.0995 / 1000)
    middlePoint_rectangle: Point3D = define_middle_point(marker1_rectangle, marker2_rectangle, marker3_rectangle, marker4_rectangle)
    print(f"Midle point rectangle: ({middlePoint_rectangle.x}; {middlePoint_rectangle.y}; {middlePoint_rectangle.z})")

    direction = input(
        "Enter the direction of conversion (otn or nto): ").strip().lower()

    # from 2023-07-26-16-18-49_Kreis.bag, coordinates of three markers
    # point_O = Point(0.0702837, 0.116806, 0.480852)
    # point_A = Point(0.1916, 0.107385, 0.476841)
    # point_B = Point(0.0819681, 0.288036, 0.480673)
    # point_S = middlePoint_circle

    # from 2023-07-26-16-16-28_Rechteck.bag, coordinates of three markers
    # point_A = Point(0.549109, 0.110133, 0.480863)
    # point_O = Point(0.428517, 0.126745, 0.478832)
    # point_B = Point(0.450365, 0.296974, 0.477975)
    # point_S = middlePoint_rectangle

    # from 2023-07-26-15-59-24_duenne_meissel_mitte.bag, coordinates of three markers
    point_A = Point3D(0.591102, 0.132062, 0.479058)
    point_O = Point3D(0.469526, 0.129647, 0.485062)
    point_B = Point3D(0.464195, 0.301068, 0.478553)
    point_S = Point3D(0.066528819491609, 0.0605050867550259, -0.41765939501656)

    if direction == "otn":
        new_coordinates_S = calculate_new_coordinates(point_O, point_A, point_B, point_S)
        print("Coordinates of S in the new coordinate system:")
        print(f"x: {new_coordinates_S.x}, y: {new_coordinates_S.y}, z: {new_coordinates_S.z}")
    elif direction == "nto":
        old_coordinates_S = calculate_old_coordinates(point_O, point_A, point_B, point_S)
        print("Coordinates of S in the old coordinate system:")
        print(f"x: {old_coordinates_S.x}, y: {old_coordinates_S.y}, z: {old_coordinates_S.z}")
    else:
        print("Invalid direction. Please choose 'otn' or 'nto'.")
