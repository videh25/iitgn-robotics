import numpy as np
from numpy.lib.twodim_base import eye

#function to calculate the Homogeneous Transformation matrix 'A' for a given array of Joint Variables
def Transmat(JointVar):
    a = JointVar[0]
    alpha = JointVar[1]
    d = JointVar[2]
    theta = JointVar[3]
    A_private = np.array([[[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.cos(alpha), a * np.cos(theta)],
                   [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                   [0, np.sin(alpha), np.cos(alpha), d],
                   [0, 0, 0, 1]]])
    return (A_private)

#function for finding the T matrix that will be used to find 'z' & 'o'
def Tmat(n, JointVar):
    A = np.zeros([n, 4, 4]) #create 'n' Homogeneous transformation matrices (4X4) for each joint

    for x in range(n):
        A[x-1] = Transmat(JointVar[n-1]) # stores A1, A2, A3... and so on

    T = np.zeros([n, 4, 4])
    #Self-Note: Try this using recursive loop too
    for x in range(n):
        temp = eye(4)
        for i in range(x+1):
            temp = np.matmul(temp, A[i])
        T[x] = temp
    return T # T is an array of T01, T02, T03 and so on,, matrices stacked in 3rd dim

#pre-def fuction for calculating Cross Product
def crossprod(a, b):
    cp = np.array([[0, a[2], a[1]], [-a[2], 0, a[0]], [-a[1], -a[0], 0]])
    return np.matmul(cp, b)

# Function to find Z & O. Intermediate calculations
def Rev_Jacob(n, T, type=0):
    o = np.zeros([n, 4])
    O_extract = np.array([0,0,0,1])
    for i in range(n):
        o[i] = np.matmul(T[i], O_extract)
    o = np.delete(o, 3, 1)  #deletes the last column of 'o'

    z = np.zeros([n, 4])
    Z_extract = np.array([0, 0, 1, 0])
    for j in range(n):
        z[j] = np.matmul(T[j], Z_extract)
    z = np.delete(z, 3, 1)  #deletes the last column of 'z'
    
    J = np.zeros([6, n])
    for k in range(n):
        if type == 0:
            cr1 = crossprod(z[k], (o[n-1]-o[k]))
            cr2 = z[k]
            J[:, k] = np.hstack((cr1, cr2))
        elif type[k] == 1:
            cp1 = z[k-1]            
            cp2 = np.zeros(3)
            J[:,k] = np.hstack((cp1,cp2))
        else:
            cr1 = crossprod(z[k], (o[n-1]-o[k]))
            cr2 = z[k]
            J[:, k] = np.hstack((cr1, cr2))
    return J

Q5 = [[0, -np.pi/4, 20, 0], [0, -np.pi/4, 20, 0], [0, 0, 20, 0]];
a2 = Tmat(3, Q5)
b2 = Rev_Jacob(3, a2, [1, 1, 1])
print("The Jacobian will be",b2)