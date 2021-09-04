import sympy as sm
import numpy as np


#Shubham Singh
#21250025
#Assignment-2
#Planar Manipulator Jacobian (RRR Configuration)

T1 = int(input('Theta_1:\n'))   #Input Joint Variable
T2 = int(input('Theta_2:\n'))
T3 = int(input('Theta_3:\n'))
l1 = int(input('Length_1 :\n'))
l2 = int(input('Length_2 :\n'))
l3 = int(input('Length_3 :\n'))
T1 = np.deg2rad(T1)       #Converting angles to radian
T2 = np.deg2rad(T2)

J_ = np.array([[(-l1*np.sin(T1)+l2*np.sin(T1+T2)+l3*np.sin(T1+T2+T3)), (-l2*np.sin(T1+T2)+l3*np.sin(T1+T2+T3)), (-l3*np.sin(T1+T2+T3))],               #Calculating Jacobian Matrix
                        [(l1*np.cos(T1)+ l2*np.cos(T1+T2)), (l3*np.cos(T1+T2+T3), l2*np.cos(T1+T2)+l3*np.cos(T1+T2+T3)), (l3*np.cos(T1+T2+T3))],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [1, 1, 1]]) 
 
print("Jacobian Matrix for Planar Manipulator is")
print(J_)