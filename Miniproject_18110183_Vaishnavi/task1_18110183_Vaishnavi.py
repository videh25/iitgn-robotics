#vaishnavi kokadwar (18110183)

import math
import numpy as np
import matplotlib.pyplot as plt

#TASK 1

#defining constants
l1 = 5
l2 = 5

#initial states
q1 = 0
q2 = 0
hinge = (0,0)

x = np.linspace(0.001,1,50)
y = [0] * len(x)

for i in range(len(x)):
    #chosen function: y = sinx + cosx
    y[i] = np.sin(x[i]) + np.cos(x[i]) #find y values for specific function
    
#define q1 and q2 arrays and end positions of both arms
q1 = [0] * len(x)
q2 = [0] * len(x)
x1 = [0] * len(x)
x2 = [0] * len(x)
y1 = [0] * len(x)
y2 = [0] * len(x)

#calculating q1 and q2 values for each position
for i in range(len(x)):
    theta = math.acos((x[i]**2+y[i]**2-l1**2-l2**2)/(2*l1*l2))
    q1[i] = math.atan(y[i]/x[i]) - math.atan((l2*np.sin(theta))/(l1+l2*np.cos(theta)))
    q2[i] = theta + q1[i]
    x1[i] = hinge[0] + l1*np.cos(q1[i])
    y1[i] = hinge[1] + l1*np.sin(q1[i])
    x2[i] = x1[i] + l2*np.cos(q2[i])
    y2[i] = y1[i] + l2*np.sin(q2[i])
    
#now plotting the manipultor arms

plt.ion()
plt.show()

for i in range(len(x)):
    
    plt.clf() # clear figure before each plot

    # set axis limits
    plt.xlim([-5, 6])
    plt.ylim([-5, 6])
    
    
    plt.plot([hinge[0],x1[i]],[hinge[1],y1[i]],'-o')
    plt.plot([x1[i],x2[i]],[y1[i],y2[i]],'-o')
    
    # pause so that the figure can be seen
    plt.pause(0.0001)
    
plt.ioff()
plt.show()