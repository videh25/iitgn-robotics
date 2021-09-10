from render import Renderer
import cv2
import numpy as np
from scipy.integrate import odeint
import scipy 

class Manipulator(Renderer):
    def __init__(self,x,y,x0,y0, recordLocation= None):
        super().__init__(recordLocation=recordLocation)
        self.ode = scipy.integrate.ode(self.func).set_integrator('vode', nsteps = 500 , method = 'bdf')
        self.l1=200
        self.l2=200
        self.m1=1
        self.m2=1
        self.I1=self.m1*self.l1**2/3    #inertia
        self.I2=self.m2*self.l2**2/3

        origin=self.getangle(x0,y0)     #Angles of Mean Position
        self.q10=origin[0]
        self.q20=origin[1]

        start=self.getangle(x,y)        
        self.q1=start[0]
        self.q2=start[1]
        self.q1dot=0
        self.q2dot=0

        self.K=0.5 #Spring constant

    def getangle(self,x,y):
        theta=np.arccos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        q1=np.arctan(y/x)-np.arctan((self.l2*np.sin(theta))/(self.l1+self.l2*np.cos(theta)))
        q2=theta+q1
        return (q1,q2)

    def getInfo(self):
        return {}
    
    def draw(self,image):
        end1=(int(200+self.l1*np.cos(self.q1)),int(300+self.l1*np.sin(self.q1)))
        end2=(int(end1[0]+self.l2*np.cos(self.q2)),int(end1[1]+self.l2*np.sin(self.q2)))

        cv2.line(image,(200,300),end1,(255,0,0),1)
        cv2.line(image,end1,end2,(0,255,0),1)

        return image

    def func(self,t,y):  #getting omega and alpha
        q1=y[0]
        q2=y[1]
        q1dot=y[2]
        q2dot=y[3]

        Fx=self.K*(self.l1*np.cos(q1)+self.l2*np.cos(q2)-(self.l1*np.cos(self.q10)+self.l2*np.cos(self.q20)))
        Fy=self.K*(self.l1*np.sin(q1)+self.l2*np.sin(q2)-(self.l1*np.sin(self.q10)+self.l2*np.sin(self.q20)))
        Tow1=Fy*self.l1*np.cos(q1)-Fx*self.l1*np.sin(q1)
        Tow2=Fy*self.l2*np.cos(q2)-Fx*self.l2*np.sin(q2)

        #ow2=self.K*(self.l1*np.sin(q1)+self.l2*np.sin(q2))*self.l2*np.cos(q2)-self.K*(self.l1*np.cos(q1)+self.l2*np.cos(q2))*self.l2*np.sin(q2)
        #Tow1=self.K*(self.l1*np.sin(q1)+self.l2*np.sin(q2))*self.l1*np.cos(q1)-self.K*(self.l1*np.cos(q1)+self.l2*np.cos(q2))*self.l2*np.sin(q1)
        dydt=[q1dot,q2dot,-Tow1/self.I1,-Tow2/self.I2]
        return dydt

    def step(self,dt):
        state=[self.q1,self.q2,self.q1dot,self.q2dot]
        
        self.ode.set_initial_value(state, 0)
        newstate = self.ode.integrate(dt)

        self.q1=newstate[0]
        self.q2=newstate[1]
        self.q1dot=newstate[2]
        self.q2dot=newstate[3]





manipulator=Manipulator(200,-200,150,-150, recordLocation='Task3video.mp4')

for i in range(400):
    manipulator.step(0.1)
    manipulator.render()





