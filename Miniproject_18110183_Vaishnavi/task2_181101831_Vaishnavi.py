#vaishnavi kokadwar (18110183)

import math
import numpy as np
import matplotlib.pyplot as plt

#TASK 2

#defining constants
l1 = 5
l2 = 5
hinge = (0,0)
I1 = 25
I2 = 25

#consider the position of the wall to be
xw = 4  
yw = 4
wall = (xw,yw)

#the following values depend on the position and orientation of the wall
thetaw = math.acos((xw**2+yw**2-l1**2-l2**2)/(2*l1*l2))
q1w = math.atan(yw/xw) - math.atan((l2*np.sin(thetaw))/(l1+l2*np.cos(thetaw)))
q2w = q1w + thetaw

#reaching out to this orientation
q11 = np.linspace(0,q1w,50)
q21 = np.linspace(0,q2w,50)

#animation
x11 = [0]*len(q11)
x21 = [0]*len(q11)
y11 = [0]*len(q11)
y21 = [0]*len(q11)

for i in range(len(q11)):
    x11[i] = hinge[0] + l1*np.cos(q11[i])
    y11[i] = hinge[1] + l1*np.sin(q11[i])
    
    x21[i] = x11[i] + l2*np.cos(q21[i])
    y21[i] = y11[i] + l2*np.sin(q21[i])
    
plt.ion()
plt.show()

for i in range(len(q11)):
    
    #plotting
    plt.clf()
    
    #setting limits
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    
    plt.plot([hinge[0],x11[i]],[hinge[1],y11[i]],'-o')
    plt.plot([x11[i],x21[i]],[y11[i],y21[i]],'-o')
    
    # pause so that the figure can be seen
    plt.pause(0.01)
    
plt.ioff()
plt.show()



#applying force depending on orientation of the wall

fx = 25
fy = 25

tau1 = fy*l1*np.cos(q1w*3.14/180) - fx*l1*np.sin(q1w*3.14/180)
tau2 = fy*l2*np.cos(q2w*3.14/180) - fx*l2*np.sin(q2w*3.14/180)

q1 = [0]*100
q2 = [0]*100
q1[0] = q1w
q2[0] = q2w

x1 = [0]*100
x1[0] = l1*np.cos(q1w)
x2 = [0]*100
x2[0] = xw
y1 = [0]*100
y1[0] = l2*np.sin(q1w)
y2 = [0]*100
y2[0] = yw

for i in range(1,100):
    q1[i] = q1[i-1] + (tau1/I1)*0.01**2
    q2[i] = q2[i-1] + (tau2/I2)*0.01**2
    
    x1[i] = hinge[0] + l1*np.cos(q1[i])
    y1[i] = hinge[1] + l1*np.sin(q1[i])
    
    x2[i] = x1[i] + l2*np.cos(q2[i])
    y2[i] = y1[i] + l2*np.sin(q2[i])
    
plt.ion()
plt.show()

for i in range(100):
    
    #plotting
    plt.clf()
    
    #setting limits
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    
    plt.plot([hinge[0],x1[i]],[hinge[1],y1[i]],'-o')
    plt.plot([x1[i],x2[i]],[y1[i],y2[i]],'-o')
    
    # pause so that the figure can be seen
    plt.pause(0.01)
    
plt.ioff()
plt.show()
    
    
    