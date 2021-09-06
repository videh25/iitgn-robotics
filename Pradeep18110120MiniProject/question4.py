import cv2
import numpy as np
import scipy
import scipy.integrate
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

l1=5
l2=5


plt.ion()
plt.show()



q1s=np.linspace(0.612,2.531,15)
q2s=np.linspace(0.612,2.531,10)
En1=[]
En2=[]
for i in q1s:
    x1=l1*np.cos(i)
    y1=l1*np.sin(i)
    O2= (x1,y1)
    Es1=[]
    Es2=[]
    for j in q2s:
        x2=x1+l2*np.cos(j)
        y2=y1+l2*np.sin(j)
        E= (x2,y2)
        Es1=Es1+[x2]
        Es2=Es2+[y2]
        En1=En1+[Es1]
        En2=En2+[Es2]

        plt.clf() # clear figure before each plot

        # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
        plt.xlim([-15, 15])
        plt.ylim([-2, 15])

        
        hinge = (0, 0)
        
        plt.plot([hinge[0], O2[0], E[0]], [hinge[1], O2[1], E[1]], '-o', linewidth=3)
        #plt.plot(Es1,Es2,color='blue')
        for i in range(0,len(En1)):
            plt.plot(En1[i],En2[i],color='red')
        # pause so that the figure can be seen
        plt.pause(0.0001)
        
plt.ioff()
plt.show()