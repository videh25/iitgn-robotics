import sympy as sm
import numpy as np

#Shubham Singh
#21250025
#Assignment-2
#SCARA Manipulator Jacobian (RRP Configuration)

T1 = int(input('Theta_1:\n'))     #Input Joint Variable
T2 = int(input('Theta_2:\n'))
a1 = int(input('a1:\n'))
a2 = int(input('a2:\n'))

T1 = np.deg2rad(T1)    #Converting angles to radian
T2 = np.deg2rad(T2)

J_ = np.array([[-a1*np.sin(T1)-a2*np.sin(T1+T2), -a2*np.sin(T1), 0],            #Calculating Jacobian Matrix
                        [a1*np.cos(T1)+ a2*np.cos(T1), a2*np.cos(T1), 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [1, 1, -1]]) 
 
print("Manipulator Jacobian Matrix for SCARA is\n")
print(J_)