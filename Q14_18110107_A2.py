
import numpy as np




a1 = input('Enter the a1 link length')
a2 = input('Enter the a2 link length')
a3 = input('Enter the a3 link length')

theta1 = input('Enter the angle 1 in degrees')
theta2 = input('Enter the angle 2 in degrees')
theta3 = input('Enter the angle 3 in degrees')


j11 = -((a1*np.sin(theta1))+(a2*np.sin(theta1+theta2)+(a3*np.sin(theta1+theta2+theta3))))
j12 = -((a2*np.sin(theta1+theta2))+(a3*np.sin(theta1+theta2+theta3)))
j13 = -a3*np.sin(theta1+theta2+theta3)
j21 = a3*np.cos(theta1+theta2+theta3)
j22 = ((a2*np.cos(theta1+theta2))+(a3*np.cos(theta1+theta2+theta3)))
j23 = ((a1*np.cos(theta1))+(a2*np.cos(theta1+theta2)+(a3*np.cos(theta1+theta2+theta3))))
j31 = 0
j32 = 0
j33 = 0
j41 = 0
j42 = 0
j43 = 0
j51 = 0
j52 = 0
j53 = 0
j61 = 1
j62 = 1
j63 = 1

J = np.matrix ('j11 j12 j13; j21 j22 j23; j31 j32 j33; j41 j42 j43; j51 j52 j53; j61 j62 j63')

