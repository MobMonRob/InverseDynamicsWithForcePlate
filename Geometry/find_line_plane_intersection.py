import numpy as np

# Import functions and objects from define_line.py
from define_line import main as get_line, Line3D

# Import functions and objects from define_plane.py
from define_plane import main as get_plane, Plane

print("Define the line:")
line = get_line()
print()
print("Define the plane:")
plane = get_plane()