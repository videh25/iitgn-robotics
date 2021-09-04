# Name - Dev Patel
# Roll No - 18110113
import numpy as np

# define some constants

#length of links
l1 = 10
l2 = 10
l3 = 10
# relative angles between links
q1 = np.pi/6
q2 = np.pi/3
# extension of prismatic link
d3 = 2


# Defining rotation matrix
R0_1 = np.matrix([[np.cos(q1),-np.sin(q1),0], [np.sin(q1),np.cos(q1),0],[0,0,1]])
R1_2 = np.matrix([[np.cos(q2),-np.sin(q2),0], [np.sin(q2),np.cos(q2),0],[0,0,1]])
R2_3 = np.array([[1,0,0],[0,1,0],[0,0,1]])

# Defining linear matrix

d0_1 = np.matrix([[0],[0],[0]])
d1_2 = np.array([[0],[l1],[0]])
d2_3 = np.array([[0],[l2],[0]])

P =np.array([[0],[0],[-l3-d3]])


H0_1 =np.vstack((np.append(R0_1, d0_1,axis =1),[0,0,0,1]))
H1_2 =np.vstack((np.append(R1_2, d1_2,axis =1),[0,0,0,1]))
H2_3 =np.vstack((np.append(R2_3, d2_3,axis =1),[0,0,0,1]))

# Getting position of end effector

z = np.dot(H0_1,np.dot(H1_2,np.dot(H2_3,np.vstack((P,[1]))))) # z = [[P0],[1]]


print('P0 = ',z[0:3])