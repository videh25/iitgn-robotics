import numpy as np

# Joint variables & lengths
l0 = 1
l1 = 1
l2 = 1
l3 = 1
d = 0.5
theta1 = np.pi/4
theta2 = np.pi/4
theta3 = np.pi/4

# Jaocobian calculation
J1 = np.array([[- l2*np.sin(theta1+theta2) - l1*np.sin(theta1) - l3*np.sin(theta1+theta2+theta3)], [l2*np.cos(theta1+theta2) + l1*np.cos(theta1) + l3*np.cos(theta1+theta2+theta3)], [0], [0], [0], [1]])

J2 = np.array([[- l2*np.sin(theta1 + theta2) - l3*np.sin(theta1 + theta2 + theta3)], [l2*np.cos(theta1 + theta2) + l3*np.cos(theta1 + theta2 + theta3)], [0], [0], [0], [1]])

J3 = np.array([[-l3*np.sin(theta1 + theta2 + theta3)], [l3*np.cos(theta1 + theta2 + theta3)], [0], [0], [0], [1]])

# Final
J = np.array([J1, J2, J3])
print("The jacobian matrix is:",J)