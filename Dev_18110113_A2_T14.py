# Name - Dev Patel
# Roll No - 18110113

import numpy as np

# define some constants

#length of the links
l1 = 10
l2 = 10
l3 = 10
# relative angles between links
q1 = -np.pi/3
q2 = np.pi/2
q3 = np.pi/3


# Defining position vectors 
O0 = np.array([[0],[0],[0]])
O1 = np.array([[l1*np.cos(q1)],[l1*np.sin(q1)],[0]])
O2 = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2)],[l1*np.sin(q1)+l2*np.sin(q1+q2)],[0]])
O3 = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3)],[l1*np.sin(q1)+l2*np.sin(q1+q2)+l3*np.sin(q1+q2+q3)],[0]])

# defining z vectors

Z0 = np.array([[0],[0],[1]])
Z1 = np.array([[0],[0],[1]])
Z2 = np.array([[0],[0],[1]])

def skew(x):
    return np.array([[0, -x[2,0], x[1,0]],
                     [x[2,0], 0, -x[0,0]],
                     [-x[1,0], x[0,0], 0]])

# Jcobian

J = np.r_[np.c_[np.dot(skew(Z0),(O3-O0)),np.c_[np.dot(skew(Z1),(O3-O1)),np.dot(skew(Z2),(O3-O2))]],np.c_[Z0,np.c_[Z1,Z2]]]
print("J =",J)