#In this code, the 2R manipulator goes to wall and applies force on the wall

#Imports:
import os
from matplotlib import animation,pyplot as plt
import numpy as np

#Declaration of constants:
m1=1 #in kg
l1=10   #in m
J1= (1/12)*m1*l1**2  + m1*(l1/2)**2 #Inertia of link one with respect to end point

m2=1   #in kg
l2=10  #in m
J2=(1/12)*m2*l2**2  #Inertia of link 2 with respect to its cetre of mass

g=9.81

#States
q1=np.pi/6
q1_dot=0
q2=np.pi/6
q2_dot=0

#Trajectory of end effector
steps=100  # Relative to total time taken for the process

#Wall location (line)
Xw=[3,13]
Yw=[3,-13]

#To reach middle of the wall
xmid=(Xw[0]+Xw[1])/2
ymid=(Yw[0]+Yw[1])/2

#initial position of end effector
xi0=0
yi0=3

#Ground attached joint location
x0=0
y0=0

#Normal force to be applied on the wall is to be
F=10 #in N

#Animation
fig,ax=plt.subplots()

def angleFromPos(Xw,Yw,xi0,yi0):

    X=np.linspace(xi0, xmid, steps)
    Y=np.linspace(yi0,ymid,steps)

    #Based theoritical calculations
    Q1=[]
    Theta=[]
    for i in range(0,steps):
        theta=np.arccos((X[i]**2+Y[i]**2-l1**2-l2**2)/(2*l1*l2))
        q1=np.arctan2(Y[i],X[i])-np.arctan2((l2*np.sin(theta)),(l1+l2*np.cos(theta)))
        Theta.append(theta)
        Q1.append(q1)
    
    #q2=theta+q1
    return[Q1,Theta]


def updatePlot(i,Q1,Theta,F):
    q1=Q1[i]
    q2=Theta[i]+q1

    x1=l1*np.cos(q1)
    y1=l1*np.sin(q1)

    x2=x1+l2*np.cos(q2)
    y2=y1+l2*np.sin(q2)
    
    ax.clear() # clear figure before each plot

    # set axis limits. Without this the limits will be autoadjusted which will make it difficult to understand.
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])

    ax.plot([x0,x1], [y0,y1], 'b-o')
    ax.plot([x1,x2],[y1,y2],'y-o')

    ax.plot(Xw,Yw, 'k',linewidth=4) #Plotting trajectory line
        
    if i==(steps-1):
        #Normal reaction of wall is equal to Force applied normally. Magnitude wise N=F. Therefore, Nx=Fx  and Ny=Fy
        phi=np.arctan2(Yw[1]-Yw[0],Xw[1]-Xw[0])

        print(phi)
        #Forces positive along global x and y axis respectively
        Ny=-F*np.cos(phi)
        Nx=F*np.sin(phi)
        
        print([Nx,Ny])
        
        #Torques. Positive means positive z axis
        tau1=Ny*l1*np.cos(q1)-Nx*l1*np.sin(q1)
        tau2=Ny*l2*np.cos(q2)-Nx*l2*np.sin(q2)

        ax.text(0.5,0.5,"Torque 1="+str(np.round_(tau1,decimals=3))+"Nm",ha='left')
        ax.text(x1-0.5,y1-1.5,"Torque 2="+str(np.round_(tau2,decimals=3))+"Nm")
        ax.text(xmid+2,ymid-2,"Fx="+str(np.abs(np.round_(Nx,decimals=3)))+"N",color="k")
        ax.text(xmid+1,ymid+2,"Fy="+str(np.abs(np.round_(Ny,decimals=3)))+"N",color="r")

        print([tau1,tau2])
        ax.plot([x2,x2-3*np.sign(Nx)],[y2,y2],'k->',label='Fx')
        ax.plot([x2,x2],[y2,y2-3*np.sign(Ny)],'r->',label='Fy')
        #plt.quiver([x2,x2],[y2,y2],[-3*np.sign(Nx),0],[0,-3*np.sign(Ny)],color=['k','r'],angles='xy',scale_units='xy',label=['Fx','Fy'])
        

def output(F):
    #Position control uisng trajectory
    [Q1,Theta]=angleFromPos(Xw,Yw,xi0,yi0)
    
    #animate
    anim=animation.FuncAnimation(fig,updatePlot,frames=steps,interval=60,fargs=[Q1,Theta,F,])

    anim.save('18110131_q2.mp4')



#Main function calling above 3 parts
if __name__=="__main__":
    output(F)            
        
    os._exit(0)