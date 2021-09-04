import numpy as np

l1 = 1
l2 = 1
l3 = 1
l4 = 1
d = 1.5
theta1 = np.pi/2
theta2 = 0

p01 = np.array([[0], [0], [l1]])
p12 = np.array([[l2], [0], [0]])
p23 = np.array([[l3], [0], [0]])
p34 = np.array([[0], [0], [l3+d]])

r01 = np.array([[np.cos(theta1), -np.sin(theta1), 0], [np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])
r12 = np.array([[np.cos(theta2), -np.sin(theta2), 0], [np.sin(theta2), np.cos(theta2), 0], [0, 0, 1]])
r23 = np.array([[1, -0, 0], [0, 1, 0], [0, 0, 1]])

p0 = p01 + np.matmul(r01,p12) + np.matmul(np.matmul(r01,r12), p23) + np.matmul(np.matmul(np.matmul(r01,r12),r23),p34)

print(p0)