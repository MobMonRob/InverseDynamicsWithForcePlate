from dataclasses import dataclass


@dataclass
class Point2D:
    x: float
    y: float


@dataclass
class Point3D:
    x_m: float
    y_m: float
    z_m: float


class Line3D:
    def __init__(self, point1: Point3D, point2: Point3D):
        self.point1: Point3D = point1
        self.point2: Point3D = point2
        self.vector = Point3D(point2.x_m - point1.x_m, point2.y_m - point1.y_m, point2.z_m - point1.z_m)
        self.xTermParametrized: str = f"{self.point2.x_m - self.point1.x_m} * t + {self.point1.x_m}"
        self.yTermParametrized: str = f"{self.point2.y_m - self.point1.y_m} * t + {self.point1.y_m}"
        self.zTermParametrized: str = f"{self.point2.z_m - self.point1.z_m} * t + {self.point1.z_m}"
        self.line_equation: str = self.__define_line_equation()
        self.parametrized_line_equation: str = self.__define_parametrized_line_equation()

    def __define_line_equation(self) -> str:
        xTerm = f"({self.point1.x_m} - x) / ({self.point2.x_m - self.point1.x_m})"
        yTerm = f"({self.point1.y_m} - y) / ({self.point2.y_m - self.point1.y_m})"
        zTerm = f"({self.point1.z_m} - z) / ({self.point2.z_m - self.point1.z_m})"
        return f"{xTerm} = {yTerm} = {zTerm}"

    def __define_parametrized_line_equation(self) -> str:
        xTermParametrized: str = f"{self.point2.x_m - self.point1.x_m} * t + {self.point1.x_m}"
        yTermParametrized: str = f"{self.point2.y_m - self.point1.y_m} * t + {self.point1.y_m}"
        zTermParametrized: str = f"{self.point2.z_m - self.point1.z_m} * t + {self.point1.z_m}"
        return f"x = {xTermParametrized}, y = {yTermParametrized}, z = {zTermParametrized}"


@dataclass
class Plane3D:
    def __init__(self, a: float, b: float, c: float, d: float):
        self.a: float = a
        self.b: float = b
        self.c: float = c
        self.d: float = d
        self.plane_equation: str = f"({a}) * x + ({b}) * y + ({c}) * z + ({d}) = 0"
