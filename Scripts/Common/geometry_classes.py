from dataclasses import dataclass
import numpy as np


@dataclass
class Point2D:
    x: float
    y: float


@dataclass
class Point3D:
    x_m: float
    y_m: float
    z_m: float


@dataclass
class Line3D:
    point1: Point3D
    point2: Point3D


@dataclass
class Plane3D:
    a: float
    b: float
    c: float
    d: float

    def vicon_main_plane() -> "Plane3D":
        #* Entlang der z-Achse soll man den Abstand von den Mitten der Marker zu der Oberfläche der auf der Kraftmessplate befestigten Metallplatte beachten.
        #* Dieser Abstand wird wie folgt berechnet: z + (24 - 25/2) + 44.78
        #* 24 ist die Höhe der Kraftmessplatte
        #* 25/2 ist die Hälfte des Durchmessers des großen Markers
        #* 44,78 ist die Höhe der Metallplatte
        z_correction: int = (24 - 25/2) + 44.78
        point1 = np.array([-0.019 / 1000, 0.124 / 1000, (-0.002 + z_correction) / 1000])
        point2 = np.array([0.6, -0.037 / 1000, (-0.037 + z_correction) / 1000])
        point3 = np.array([-0.102 / 1000, 0.434847, (0.1321 + z_correction) / 1000])
        
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
