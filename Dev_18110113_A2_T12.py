# Name - Dev Patel
# Roll No - 18110113

import numpy as np

# define some constants

#length of links
l1 = 10
l2 = 10
l3 = 10
# relative angles between links
q1 = np.pi/2
q2 = np.pi/4
# extension of prismatic link
d3 = 2


# Defining position vectors 
O0 = np.array([[0],[0],[0]])
O1 = np.array([[l1*np.cos(q1)],[l1*np.sin(q1)],[0]])
O2 = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2)],[l1*np.sin(q1)+l2*np.sin(q1+q2)],[0]])
O3 = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2)],[l1*np.sin(q1)+l2*np.sin(q1+q2)],[-l3-d3]])

# defining z vectors

Z0 = np.array([[0],[0],[1]])
Z1 = np.array([[0],[0],[1]])
Z2 = np.array([[0],[0],[-1]])

def skew(x):
    return np.array([[0, -x[2,0], x[1,0]],
                     [x[2,0], 0, -x[0,0]],
                     [-x[1,0], x[0,0], 0]])

# Jcobian

J = np.r_[np.c_[np.dot(skew(Z0),(O3-O0)),np.c_[np.dot(skew(Z1),(O3-O1)),Z2]],np.c_[Z0,np.c_[Z1,[[0],[0],[0]]]]]
print("J =",J)