import numpy as np


class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x: float = x
        self.y: float = y
        self.z: float = z


def normalize_vector(vec):
    magnitude = np.linalg.norm(vec)
    return vec / magnitude


def calculate_new_coordinates(o, a, b, s):
    e1 = np.array([a.x - o.x, a.y - o.y, a.z - o.z])
    e2 = np.array([b.x - o.x, b.y - o.y, b.z - o.z])
    e3 = np.cross(e1, e2)

    e1_normalized = normalize_vector(e1)
    e2_normalized = normalize_vector(e2)
    e3_normalized = normalize_vector(e3)

    a_inv = np.array([[e1_normalized[0], e2_normalized[0], e3_normalized[0]],
                      [e1_normalized[1], e2_normalized[1], e3_normalized[1]],
                      [e1_normalized[2], e2_normalized[2], e3_normalized[2]]])

    s_old_Minus_o_old = np.array([s.x - o.x, s.y - o.y, s.z - o.z])
    s_new = np.dot(a_inv, s_old_Minus_o_old)

    return Point(s_new[0], s_new[1], s_new[2])


def calculate_old_coordinates(o, a, b, s):
    e1 = np.array([a.x - o.x, a.y - o.y, a.z - o.z])
    e2 = np.array([b.x - o.x, b.y - o.y, b.z - o.z])
    e3 = np.cross(e1, e2)

    e1_normalized = normalize_vector(e1)
    e2_normalized = normalize_vector(e2)
    e3_normalized = normalize_vector(e3)

    a = np.linalg.inv(np.array([[e1_normalized[0], e2_normalized[0], e3_normalized[0]],
                                [e1_normalized[1], e2_normalized[1], e3_normalized[1]],
                                [e1_normalized[2], e2_normalized[2], e3_normalized[2]]]))

    A_mult_S_new = np.dot(a, np.array([s.x, s.y, s.z]))
    s_old = np.array([A_mult_S_new[0] + o.x, A_mult_S_new[1] + o.y, A_mult_S_new[2] + o.z])
    return Point(s_old[0], s_old[1], s_old[2])


if __name__ == "__main__":
    marker1_circle: Point = Point(155.7785 / 1000, 184.1291 / 1000, 69.1871 / 1000)
    marker2_circle: Point = Point(125.3213 / 1000, 153.6431 / 1000, 69.0495 / 1000)
    marker3_circle: Point = Point(92.8185 / 1000, 183.7491 / 1000, 69.0835 / 1000)
    marker4_circle: Point = Point(124.7145 / 1000, 214.5565 / 1000, 69.3995 / 1000)

    middlePoint_circle: Point = Point(
        (marker1_circle.x + marker2_circle.x + marker3_circle.x + marker4_circle.x) / 4,
        (marker1_circle.y + marker2_circle.y + marker3_circle.y + marker4_circle.y) / 4,
        (marker1_circle.z + marker2_circle.z + marker3_circle.z + marker4_circle.z) / 4 - (7 + 2.02) / 1000
    )

    print(f"Midle point: x: {middlePoint_circle.x}, y: {middlePoint_circle.y}, z: {middlePoint_circle.z}")

    direction = input(
        "Enter the direction of conversion (otn or nto): ").strip().lower()

    point_O = Point(0.221414, 0.101355, 0.462577)
    point_A = Point(0.343369, 0.102263, 0.465886)
    point_B = Point(0.21844, 0.272956, 0.460893)
    point_S = middlePoint_circle

    # Result old -> new:
    point_S = Point(0.059669819491609016, 0.09288508675502588, -0.41765939501656)

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
