import numpy as np

#Joint variables & lengths
l0 = 1
l1 = 1
l2 = 1
l3 = 0.5
d = 0.75
theta1 = np.pi/4
theta2 = np.pi/4

# Jacobian Calculations
J1 = np.array([[- l2*(np.sin(theta1+theta2)) - l1*np.sin(theta1)], [l2*(np.cos(theta1+theta2)) + l1*np.cos(theta1)], [0], [0], [0], [1]])

J2 = np.array([[- l2*(np.sin(theta1+theta2))], [l2*(np.cos(theta1+theta2))], [0], [0], [0], [1]])

J3 = np.array([[0], [0], [1], [0], [0], [0]])
 
# final
J = np.array([J1, J2, J3])
print("The jacobian matrix is:",J)