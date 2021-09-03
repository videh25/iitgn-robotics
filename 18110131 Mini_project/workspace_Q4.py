#In this code, the 2R manipulator follows trajectory using (1) Direct angle calculation (kinematics only) (2)
# Torques(dynamics included)

#Imports:
from matplotlib import pyplot as plt
import numpy as np
import os
from scipy.spatial import ConvexHull
import alphashape
from descartes import PolygonPatch



#Declaration of constants:
m1=1 #in kg
l1=10   #in m
J1= (1/12)*m1*l1**2  + m1*(l1/2)**2 #Inertia of link one with respect to end point

m2=1   #in kg
l2=10  #in m
J2=(1/12)*m2*l2**2  #Inertia of link 2 with respect to its cetre of mass

g=9.81


#Angles from 35 to 135
Q1=np.deg2rad(np.linspace(35,135, 50)) #in degrees. Convert to radians, 50 steps( number of elements in the array)
Theta=np.deg2rad(np.linspace(35,135,50))


#End points of links, x1,y1 for link 1    and x2,y2 for link 2
x0=0 #Start point
y0=0

def workspace(Q1,Theta):

    #End factor point collections
    X1=[]
    Y1=[]
    X2=[]
    Y2=[]

    for q1 in Q1:
        x1=l1*np.cos(q1)
        y1=l1*np.sin(q1)
        X1.append(x1)
        Y1.append(y1)

        for theta in Theta:
            q2=theta+q1
            x2=x1+l2*np.cos(q2)
            y2=y1+l2*np.sin(q2)
            X2.append(x2)
            Y2.append(y2)
            
    
    return [X1,Y1,X2,Y2]

def output():
    [X1,Y1,X2,Y2]=workspace(Q1,Theta)

    endPoints=np.column_stack([X2,Y2])
    #hull=ConvexHull(endPoints)
    concave_hull=alphashape.alphashape(endPoints,2.0)
    
    plt.ion()
    plt.show()
    
    count=0
    for x1,y1 in zip(X1,Y1) :
        #for a certain x1,y1 there are len(Theta) values of x2,y2. Uisng count to use respective values of x2,y2
        for x2,y2 in zip(X2[count*len(Theta):(count+1)*len(Theta)],Y2[count*len(Theta):(count+1)*len(Theta)]):
            plt.clf() # clear figure before each plot

            # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
            plt.xlim([-20, 20])
            plt.ylim([-20, 20])

            plt.plot([x0,x1], [y0,y1], '-o')
            plt.plot([x1,x2],[y1,y2],'-o')
            
            #plt.fill(endPoints[hull.vertices,0],endPoints[hull.vertices,1],'k',alpha=0.3)
            ax=plt.gca()
            ax.add_patch(PolygonPatch(concave_hull,alpha=0.2))
            # pause so that the figure can be seen
            plt.pause(0.00001)
        count=count+1
    plt.ioff()
    plt.show()


output()
os._exit(0)