#In this code, the 2R manipulator follows trajectory using (1) Direct angle calculation (kinematics only) (2)
# Torques(dynamics included)

#Imports:
import os
from matplotlib import animation,pyplot as plt
import numpy as np
import scipy.integrate



#Declaration of constants:
m1=1 #in kg
l1=10   #in m

m2=1   #in kg
l2=10  #in m

g=0

k=0.05

#Trajectory of end effector
steps=100# Relative to total time required to reach end point
X=np.linspace(3, 13, steps)
Y=np.linspace(3,-13,steps)

#Initial States
q10=np.pi/10
q1_dot0=0
q20=np.pi/6
q2_dot0=0

#Ground attached joint location
x0=0
y0=0

#Animation
fig,ax=plt.subplots()

def updatePlot(i,Q1,Theta):
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

    
def dynamicSystem(y,t,X,Y):

    q1=y[0]
    q1_dot=y[1]
    q2=y[2]
    q2_dot=y[3]

    #Based on solution in pdf attached
    A= np.array([[(1/3)*m1*l1**2+m2*l1**2, (1/2)*m2*l1*l2*np.cos(q2-q1)],[(1/2)*m2*l1*l2*np.cos(q2-q1), ((1/12)+(1/4))*m2*l2**2]])
    B=np.array([[0,-(1/2)*m2*l1*l2*(q2_dot-q1_dot)*np.sin(q2-q1)],[(1/2)*m2*l1*l2*(q2_dot-q1_dot)*np.sin(q2-q1),0]])
    P=np.array([[(1/2)*m1*g*l1*np.cos(q1)+m2*g*l1*np.cos(q1)],[(1/2)*m2*g*l2*np.cos(q2)]])
    Q_dot=np.array([[q1_dot],[q2_dot]])
    Tau=np.array([[0],[0]])#Input torques
    
    Tau_s=np.array([[-k*((l1*np.sin(q1)+l2*np.sin(q2)*l1*np.cos(q1))+(l1*np.cos(q1)+l2*np.cos(q2)*l1*np.sin(q1)))],[-k*((l1*np.sin(q1)+l2*np.sin(q2))*l2*np.cos(q2)+(l1*np.cos(q1)+l2*np.cos(q2))*l2*np.sin(q2))]])
    Q_ddot=np.matmul(np.linalg.inv(A),(Tau +Tau_s- P - np.matmul(B,Q_dot)))
    dydt=[q1_dot,Q_ddot[0],q2_dot,Q_ddot[1]] #q1_dot, q1_ddot, q2_dot, q2_ddot

    return dydt


def angleFromPos(X,Y):
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


def output():
    #Position uisng trajectory
    [Q1,Theta]=angleFromPos(X,Y)
    solution=scipy.integrate.odeint(dynamicSystem,y0=[q10,q1_dot0,q20,q2_dot0],t=np.linspace(0,steps,steps),args=(X,Y,))
    Q1=np.array(solution[:,0])
    Theta= np.array(solution[:,2])-Q1
    #print(Q1)
    anim=animation.FuncAnimation(fig,updatePlot,frames=steps,interval=60,fargs=[Q1,Theta,])

    anim.save('18110131_q3.mp4')

#Main function calling above 3 parts
if __name__=="__main__":
    output()
    os._exit(0)