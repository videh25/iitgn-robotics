import cv2
import numpy as np
import scipy
import scipy.integrate
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

l1=10
l2=10

q10=0
q20=0
plt.ion()
plt.show()

t=-np.pi/8
dt=np.pi/8
while(t<=6*np.pi+np.pi/8):
    t=t+dt

    #Position of end point on spring when displaced by distance A from the mean positon(x0,y0) x(t)=x0+Acos((k/m)^0.5*t+phi)
    # let us take Fx=1.5N, Fy=1.5N()forces applied on the end point of manipulator. Therefore, A=1.5, x0=6, y0=14, k=1, m=1, phi=pi/2
     
    x=6+1.5*np.cos(t+np.pi/2)
    y=14+1.5*np.cos(t+np.pi/2)

    if(x>=0 and y>=0):
        theta=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
        q1=np.arctan(y/x) - np.arctan((l2*np.sin(theta))/(l1+l2*np.cos(theta)))
        q2=q1+theta

    elif(x<=0 and y<=0):
        theta=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
        q1=-np.pi+(np.arctan(np.abs(y)/np.abs(x)) - np.arctan((l2*np.sin(theta))/(l1+l2*np.cos(theta))))
        q2=q1+theta

    elif(x<=0 and y>=0):
        theta=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
        q1=np.pi-(np.arctan(np.abs(y)/np.abs(x)) + np.arctan((l2*np.sin(theta))/(l1+l2*np.cos(theta))))
        q2=q1+theta

    else:
        theta=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
        q1=-(np.arctan(np.abs(y)/np.abs(x)) + np.arctan((l2*np.sin(theta))/(l1+l2*np.cos(theta))))
        q2=q1+theta
    

    q1s=np.linspace(q10,q1,1)
    q2s=np.linspace(q20,q2,1)
    q10=q1
    q20=q2

    for i in range(len(q1s)):
        q1n= q1s[i]
        q2n= q2s[i]

        O2 = (l1 * np.cos(q1n), l1 * np.sin(q1n))
        E  = (l1 * np.cos(q1n)+ l2* np.cos(q2n), l1 * np.sin(q1n)+l2 * np.sin(q2n))
        
        plt.clf() # clear figure before each plot

        # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
        plt.xlim([-20, 20])
        plt.ylim([-20, 20])

        hinge = (0, 0)
        
        plt.plot([hinge[0], O2[0], E[0]], [hinge[1], O2[1], E[1]], '-o',linewidth=3)
        plt.plot(6,14,'o')

        # pause so that the figure can be seen
        plt.pause(0.0001)

    
plt.ioff()
plt.show()