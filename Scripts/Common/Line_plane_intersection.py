from sympy.solvers import solve
from sympy import Symbol
from Common.geometry_classes import Point3D, Line3D, Plane3D
from varname import nameof

class LinePlaneIntersection():
    def __init__(self):
        # self.t = Symbol('t')
        # self.x1 = Symbol("line.point1.x_m")
        # self.x2 = Symbol("line.point2.x_m")
        # self.y1 = Symbol("line.point1.y_m")
        # self.y2 = Symbol("line.point2.y_m")
        # self.z1 = Symbol("line.point1.z_m")
        # self.z2 = Symbol("line.point2.z_m")
        # self.x = ((self.x2 - self.x1)* self.t + self.x1)
        # self.y = ((self.y2 - self.y1)* self.t + self.y1)
        # self.z = ((self.z2 - self.z1)* self.t + self.z1)
        # self.a = Symbol("plane.a")
        # self.b = Symbol("plane.b")
        # self.c = Symbol("plane.c")
        # self.d = Symbol("plane.d")
        # self.formula = self.a*self.x + self.b*self.y + self.c*self.z + self.d
        # self.t_expr = solve(self.formula, self.t)[0]
        # self.x_t = self.x.replace(self.t, self.t_expr)
        # self.y_t = self.y.replace(self.t, self.t_expr)
        # self.z_t = self.z.replace(self.t, self.t_expr)
        # print(self.x_t)
        # print(self.y_t)
        # print(self.z_t)
        return


    def intersect(self, line: Line3D, plane: Plane3D) -> Point3D:
        # locals_dict = {"line": line, "plane": plane}
        # x_value = eval(str(self.x_t), None, locals_dict)
        # y_value = eval(str(self.y_t), None, locals_dict)
        # z_value = eval(str(self.z_t), None, locals_dict)

        x_value = line.point1.x_m + (-line.point1.x_m + line.point2.x_m)*(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c + plane.d)/(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c - line.point2.x_m*plane.a - line.point2.y_m*plane.b - line.point2.z_m*plane.c)

        y_value = line.point1.y_m + (-line.point1.y_m + line.point2.y_m)*(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c + plane.d)/(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c - line.point2.x_m*plane.a - line.point2.y_m*plane.b - line.point2.z_m*plane.c)

        z_value = line.point1.z_m + (-line.point1.z_m + line.point2.z_m)*(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c + plane.d)/(line.point1.x_m*plane.a + line.point1.y_m*plane.b + line.point1.z_m*plane.c - line.point2.x_m*plane.a - line.point2.y_m*plane.b - line.point2.z_m*plane.c)

        point = Point3D(x_m=x_value, y_m=y_value, z_m=z_value)
        return point

