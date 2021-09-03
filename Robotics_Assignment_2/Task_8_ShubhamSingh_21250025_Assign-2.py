import numpy as np 
import math
# Shubham Singh
# 21250025
# Assignment-2
# Stanford Manipulator

T1 = int(input('Theta-1:\n'))      #taking input values for joint configuration
T2 = int(input('Theta-2:\n')) 
d1 = int(input('d1:\n'))
d2 = int(input('d2:\n'))
d3 = int(input('d3:\n'))


T1 = np.deg2rad(T1)     #Converting angles to radians
T2 = np.deg2rad(T2)


rot_mat_0_1 = np.array([[np.cos(T1), 0, -np.sin(T1)],         #Rotation matrix of 1 frame relative to 0 frame
                        [np.sin(T1), 0,  np.cos(T1)],
                        [0, -1, 0]]) 
rot_mat_1_2 = np.array([[np.cos(T2), 0, np.sin(T2)],           #Rotation matrix of 2 frame relative to 1 frame
                        [np.sin(T2), 0, -np.cos(T2)],
                        [0, 1, 0]]) 
rot_mat_2_3 = np.array([[1, 0, 0],                           #Rotation matrix of 3 frame relative to 2 frame
                        [0, 1, 0],
                        [0, 0, 1]]) 

rot_mat_0_3 = rot_mat_0_1 * rot_mat_1_2 * rot_mat_2_3     #Rotation matrix of 6 frame relative to 0 frame
 

disp0_1 = np.array([[0],
                         [0],
                         [0]])
disp1_2 = np.array([[0],
                         [0],
                         [d2]])
disp2_3 = np.array([[0],
                         [0],
                         [d3]])

extra_row_homgen = np.array([[0, 0, 0, 1]])
P_0 = np.array([[(d3*np.cos(T1)*np.sin(T2))-(d2*np.sin(T1))],
                        [(d3*np.sin(T1)*np.sin(T2))+(d2*np.cos(T1))],
                        [d3*np.cos(T2)],
                        [1]])
 

homgen_0_1 = np.concatenate((rot_mat_0_1, disp0_1), axis=1)           #Homogeneous Transformation
homgen_0_1 = np.concatenate((homgen_0_1, extra_row_homgen), axis=0) 
 
homgen_1_2 = np.concatenate((rot_mat_1_2, disp1_2), axis=1)
homgen_1_2 = np.concatenate((homgen_1_2, extra_row_homgen), axis=0)

homgen_2_3 = np.concatenate((rot_mat_2_3, disp2_3), axis=1)
homgen_2_3 = np.concatenate((homgen_2_3, extra_row_homgen), axis=0)


homgen_0_3 = homgen_0_1 * homgen_1_2 * homgen_2_3     #Homogeneous transformation of 3 relative to 0
 
print("Homogeneous Transformation Matrix is:\n")
print(homgen_0_3)
print("\n")
print("Position vector of the End Effector of the Stanford Manipulator is:\n")
print(P_0)