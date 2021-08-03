from simple_cartographer import look_at_the_point
import numpy as np

trans0 = np.eye(4)
org = np.array([0, 0, 0])
point1 = np.array([0, 0, 5])
point2 = np.array([0, 3, 5])
point3 = np.array([0, 0, -5])
a1 = look_at_the_point(trans0, org, point1)
a2 = look_at_the_point(trans0, org, point2)
a3 = look_at_the_point(trans0, org, point3)
print(a1)
print(a2)
print(a3)
print(np.sum((1 - np.diag(a1 @ a2)[:-1]))/6)
print(np.sum((1 - np.diag(a1 @ a3)[:-1]))/6)
