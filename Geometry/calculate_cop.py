from varname import nameof
import numpy as np

# Get input values from the user
a, b, z0 = 0, 0, 0
a, b, z0 = map(float, input(f"Enter the values of {nameof(a)}, {nameof(b)} and {nameof(z0)} (separated by a space): ").split())

h = 0
h = float(input(f"Enter the value of {nameof(h)}: "))

fx, fy, fz = 0, 0, 0
fx, fy, fz = map(float, input(f"Enter the values of {nameof(fx)}, {nameof(fy)} and {nameof(fz)} (separated by a space): ").split())

mx, my, mz = 0, 0, 0
mx, my, mz = map(float, input(f"Enter the values of {nameof(mx)}, {nameof(my)}, and {nameof(mz)} (separated by a space): ").split())

# Calculate CoPx
CoPx = ((-my - fx * (z0 + h)) / fz) - a

# Calculate CoPy
CoPy = ((mx - fy * (z0 + h)) / fz) - b

# Print the result
print(f"The value of {nameof(CoPx)} is: {CoPx}\n")
print(f"The value of {nameof(CoPy)} is: {CoPy}\n")

# For fixed rotational transformation matrix from local force plate frame to global frame in the plate corner:
#     (0 1  0)
# R = (1 0  0)
#     (0 0 -1)

# Get force plate width and length (in my convention width is the smaller number, length is the larger number):
width, length = 400, 600 # For AMTI BP400600 model in labor
width, length = map(float, input(f"Enter the values of {nameof(width)} and {nameof(length)} (separated by a space): ").split())

CoPx_g = length / 2 + CoPy
CoPy_g = width / 2 + CoPx

# Print the result
print(f"The value of {nameof(CoPx_g)} is: {CoPx_g}\n")
print(f"The value of {nameof(CoPy_g)} is: {CoPy_g}\n")