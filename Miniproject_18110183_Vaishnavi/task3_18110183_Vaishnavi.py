#vaishnavi kokadwar (18110183)

import math
import numpy as np
import matplotlib.pyplot as plt

#TASK 3

#defining constants
l1 = 5 #in meters
l2 = 5
m1 = 1 #in kg
m2 = 1
I1 = 25
I2 = 25
hinge = (0,0)
k = 10 #spring constant
g = 9.8 #in m/s^2

#define end effector mean position
x0 = 1
y0 = 1

#defining initial position of end manipulator
q1 = 45*3.14/180
q2 = 45*3.14/180
omega1 = 0 #q_dot1
omega2 = 0 #q_dot2
alpha1 = 0 #q_doubledot1
alpha2 = 0 #q_doubledot2
dt = 0.01

#calculating initial position of end effector
x1 = hinge[0] + l1*np.cos(q1)
y1 = hinge[1] + l1*np.sin(q1)

x2 = x1 + l2*np.cos(q2)
y2 = y1 + l2*np.sin(q2)

tau1s = k*(l1*np.sin(q1)+l2*np.sin(q2))*l1*np.cos(q1) - k*y0 - k*(l1*np.cos(q1)+l2*np.cos(q2))*l1*np.sin(q1) - k*x0
tau2s = k*(l1*np.sin(q1)+l2*np.sin(q2))*l1*np.cos(q2) - k*y0 - k*(l1*np.cos(q1)+l2*np.cos(q2))*l2*np.sin(q2) - k*x0

tau1 = (1/3)*m1*l1**2*alpha1 + m2*l1**2*alpha1 + m2*l2*l1*alpha2*0.5*np.cos(q2-q1) - m2*l2*l1*alpha2*(omega2-omega1)*0.5*np.sin(q2-q1) + m1*g*l1*np.cos(q1)
tau2 = (1/3)*m2*l2**2*alpha2 + m2*l2**2*alpha2*0.25 + m2*l1*l2*alpha1*np.cos(q2-q1)*0.5 - m2*l1*l2*omega1*(omega2-omega1)*np.sin(q2-q1)*0.5 + m2*g*l2*0.5*np.sin(q2)

omega1_prev = omega1
omega2_prev = omega2

omega1 += ((tau1s+tau1)/I1)*dt
omega2 += ((tau2s+tau2)/I2)*dt

alpha1 = (omega1-omega1_prev)/dt
alpha2 = (omega2-omega2_prev)/dt

q1 += omega1*dt
q2 += omega2*dt

x1 = hinge[0] + l1*np.cos(q1)
y1 = hinge[1] + l1*np.sin(q1)

x2 = x1 + l2*np.cos(q2)
y2 = y1 + l2*np.sin(q2)

plt.ion()
plt.show()
    
plt.clf() # clear figure before each plot

# set axis limits
plt.xlim([-10, 10])
plt.ylim([-10, 10])


plt.plot([hinge[0],x1],[hinge[1],y1],'-o')
plt.plot([x1,x2],[y1,y2],'-o')

# pause so that the figure can be seen
plt.pause(0.0001)

while (x2-x0 != 0 or y2-y0 != 0) or (omega1!=0 or omega2!=0):
        
    tau1s = k*(l1*np.sin(q1)+l2*np.sin(q2))*l1*np.cos(q1) - k*y0 - k*(l1*np.cos(q1)+l2*np.cos(q2))*l1*np.sin(q1) - k*x0
    tau2s = k*(l1*np.sin(q1)+l2*np.sin(q2))*l1*np.cos(q2) - k*y0 - k*(l1*np.cos(q1)+l2*np.cos(q2))*l2*np.sin(q2) - k*x0
    
    tau1 = (1/3)*m1*l1**2*alpha1 + m2*l1**2*alpha1 + m2*l2*l1*alpha2*0.5*np.cos(q2-q1) - m2*l2*l1*alpha2*(omega2-omega1)*0.5*np.sin(q2-q1) + m1*g*l1*np.cos(q1)
    tau2 = (1/3)*m2*l2**2*alpha2 + m2*l2**2*alpha2*0.25 + m2*l1*l2*alpha1*np.cos(q2-q1)*0.5 - m2*l1*l2*omega1*(omega2-omega1)*np.sin(q2-q1)*0.5 + m2*g*l2*0.5*np.sin(q2)
    
    omega1_prev = omega1
    omega2_prev = omega2

    omega1 += ((tau1s+tau1)/I1)*dt
    omega2 += ((tau2s+tau2)/I2)*dt

    alpha1 = (omega1-omega1_prev)/dt
    alpha2 = (omega2-omega2_prev)/dt
    
    q1 += omega1*dt
    q2 += omega2*dt
    
    x1 = hinge[0] + l1*np.cos(q1)
    y1 = hinge[1] + l1*np.sin(q1)

    x2 = x1 + l2*np.cos(q2)
    y2 = y1 + l2*np.sin(q2)
        
    plt.clf() # clear figure before each plot

    # set axis limits
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])


    plt.plot([hinge[0],x1],[hinge[1],y1],'-o')
    plt.plot([x1,x2],[y1,y2],'-o')

    # pause so that the figure can be seen
    plt.pause(0.0001)
        
plt.ioff()
plt.show()