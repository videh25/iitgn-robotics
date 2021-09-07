#vaishnavi kokadwar (18110183)

import math
import numpy as np
import matplotlib.pyplot as plt

#TASK 4

#defining constants
l1 = 5
l2 = 5
hinge = (0,0)

#joint angles to swept are from 35 degrees to 145 degrees
q1 = np.linspace(35,145,110)
q2 = np.linspace(35,145,110)

#defining empty arrays for end positions of both arms
x1 = [0]*len(q1)
y1 = [0]*len(q1) #for endpoint of first arm
x2 = [0]*len(q1)
y2 = [0]*len(q1) #for endpoint of second arm

for i in range (len(q1)):
    x1[i] = hinge[0] + l1*np.cos(q1[i]*math.pi/180)
    y1[i] = hinge[1] + l1*np.sin(q1[i]*math.pi/180)
    
    x2[i] = x1[i] + l2*np.cos(q2[i]*math.pi/180)
    y2[i] = y1[i] + l2*np.sin(q2[i]*math.pi/180)
    
plt.ion()
plt.show()

for i in range(len(q1)):
    
    plt.clf() # clear figure before each plot

    # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    
    
    plt.plot([hinge[0],x1[i]],[hinge[1],y1[i]],'-o')
    plt.plot([x1[i],x2[i]],[y1[i],y2[i]],'-o')
    
    # pause so that the figure can be seen
    plt.pause(0.0001)
    
plt.ioff()
plt.show()