import numpy as np

# Joint variables & lengths
l0 = 1
l1 = 1
l2 = 1
d = 0.5
theta1 = np.pi/4
theta2 = np.pi/4

# transformation matrices
P01 = np.array([[0], [0], [l0]])
R01 = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])

P12 = np.array([[l1], [0], [0]])
R12 = np.array([[1, 0, 0], [0, np.cos(theta2), -np.sin(theta2)], [0, np.sin(theta2), np.cos(theta2)]])

P23 = np.array([[0], [l2+d], [0]])

# enf-effector

p0 = P01 + np.matmul(R01,P12) + np.matmul(np.matmul(R01,R12), P23)
print(p0)