import numpy as np 
import math

#Shubham Singh
#21250025
# Assignment-2
# SCARA Manipulator

T1 = int(input('Theta-1:\n'))    #Taking input values for joint configuration
T2 = int(input('Theta-2:\n'))

a1 = int(input('a1:\n'))         
a2 = int(input('a2:\n'))
d1 = int(input('d1:\n'))
d2 = int(input('d2:\n'))
d3 = int(input('d3:\n'))


T1 = np.deg2rad(T1)  # Converting angle to radians
T2 = np.deg2rad(T2)

rot_mat_0_1 = np.array([[np.cos(T1), -np.sin(T1), 0],          #Rotation matrix of 1 frame relative to 0 frame
                        [np.sin(T1), np.cos(T1), 0],
                        [0, 0, 1]]) 
rot_mat_1_2 = np.array([[np.cos(T2), np.sin(T2), 0],           #Rotation matrix of 2 frame relative to 1 frame
                        [np.sin(T2), -np.cos(T2), 0],
                        [0, 0, -1]]) 
rot_mat_2_3 = np.array([[1, 0, 0],                             #Rotation matrix of 3 frame relative to 2 frame
                        [0, 1, 0],
                        [0, 0, 1]]) 

                       
rot_mat_0_3 = rot_mat_0_1 * rot_mat_1_2 * rot_mat_2_3     #Rotation matrix of 4 frame relative to 0 frame

disp0_1 = np.array([[a1*np.cos(T1)],
                         [a1*np.sin(T1)],
                         [0]])
disp1_2 = np.array([[a2*np.cos(T2)],
                         [a2*np.sin(T2)],
                         [0]])
disp2_3 = np.array([[0],
                         [0],
                         [d3]])

extra_row_homgen = np.array([[0, 0, 0, 1]])
P_0 = np.array([[a1*np.cos(T1)+a2*np.cos(T1+T2)],
                        [a1*np.sin(T1)+a2*np.sin(T1+T2)],
                        [-d3],
                        [1]])

homgen_0_1 = np.concatenate((rot_mat_0_1, disp0_1), axis=1)      # Homogeneous transformation of frame 1 realtive to frame 0.
homgen_0_1 = np.concatenate((homgen_0_1, extra_row_homgen), axis=0) 
 
homgen_1_2 = np.concatenate((rot_mat_1_2, disp1_2), axis=1)
homgen_1_2 = np.concatenate((homgen_1_2, extra_row_homgen), axis=0)

homgen_2_3 = np.concatenate((rot_mat_2_3, disp2_3), axis=1)
homgen_2_3 = np.concatenate((homgen_2_3, extra_row_homgen), axis=0)

homgen_0_3 = homgen_0_1 * homgen_1_2 * homgen_2_3  

print("Homogeneous Transformation Matrix from Frame 0 to Frame 4 is:\n")
print(homgen_0_3)
print("\n")

print("Position vector of the End Effector is:\n")
print(P_0)
