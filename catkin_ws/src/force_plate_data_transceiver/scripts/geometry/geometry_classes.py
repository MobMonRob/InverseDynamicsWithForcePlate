class Point3D:
    def __init__(self, x: float, y: float, z: float):
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.pointExpression = f"({x}, {y}, {z})"


class Line3D:
    def __init__(self, point1: Point3D, point2: Point3D):
        self.point1: Point3D = point1
        self.point2: Point3D = point2
        self.vector = Point3D(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z)
        self.xTermParametrized: str = f"{self.point2.x - self.point1.x} * t + {self.point1.x}"
        self.yTermParametrized: str = f"{self.point2.y - self.point1.y} * t + {self.point1.y}"
        self.zTermParametrized: str = f"{self.point2.z - self.point1.z} * t + {self.point1.z}"
        self.line_equation: str = self.__define_line_equation()
        self.parametrized_line_equation: str = self.__define_parametrized_line_equation()

    def __define_line_equation(self) -> str:
        xTerm = f"({self.point1.x} - x) / ({self.point2.x - self.point1.x})"
        yTerm = f"({self.point1.y} - y) / ({self.point2.y - self.point1.y})"
        zTerm = f"({self.point1.z} - z) / ({self.point2.z - self.point1.z})"
        return f"{xTerm} = {yTerm} = {zTerm}"

    def __define_parametrized_line_equation(self) -> str:
        xTermParametrized: str = f"{self.point2.x - self.point1.x} * t + {self.point1.x}"
        yTermParametrized: str = f"{self.point2.y - self.point1.y} * t + {self.point1.y}"
        zTermParametrized: str = f"{self.point2.z - self.point1.z} * t + {self.point1.z}"
        return f"x = {xTermParametrized}, y = {yTermParametrized}, z = {zTermParametrized}"
    
class Plane3D:
    def __init__(self, a: float, b: float, c: float, d: float):
        self.a: float = a
        self.b: float = b
        self.c: float = c
        self.d: float = d
        self.plane_equation: str = f"({a}) * x + ({b}) * y + ({c}) * z + ({d}) = 0"
