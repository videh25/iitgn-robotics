#Write a python code implementing the above Jacobian such that the entire
#Jacobian matrix is output for any given values of joint variable

import numpy as np

l1 = 1
l2 = 1
l3 = 1
theta1 = np.pi/2
theta2 = 0
theta3 = 0

J1 = np.array([[- l2*np.sin(theta1 + theta2) - l1*np.sin(theta1) - l3*np.sin(theta1 + theta2 + theta3)],
              [l2*np.cos(theta1 + theta2) + l1*np.cos(theta1) + l3*np.cos(theta1 + theta2 + theta3)], [0], [0], [0], [1]])

J2 = np.array([[- l2*np.sin(theta1 + theta2) - l3*np.sin(theta1 + theta2 + theta3)],
              [l2*np.cos(theta1 + theta2) + l3*np.cos(theta1 + theta2 + theta3)], [0], [0], [0], [1]])

J3 = np.array([[-l3*np.sin(theta1 + theta2 + theta3)],
              [l3*np.cos(theta1 + theta2 + theta3)], [0], [0], [0], [1]])

J = np.array([J1, J2, J3])
print(J)