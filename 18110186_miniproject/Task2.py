from render import Renderer
import numpy as np
import cv2

class Manipulator(Renderer):
    def __init__(self,x0,y0,xf,yf,recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.l1=200
        self.l2=200

        origin=self.getangle(x0,y0)   #inital orientation
        self.q10=origin[0]
        self.q20=origin[1]

        final=self.getangle(xf,yf)
        self.q1f=final[0]
        self.q2f=final[1]

   

    
    def getangle(self,x,y):
        theta=np.arccos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        q1=np.arctan(y/x)-np.arctan((self.l2*np.sin(theta))/(self.l1+self.l2*np.cos(theta)))
        q2=theta+q1
        return (q1,q2)


    def getInfo(self):
        info = {
            "Position" : self.ballpos
        }
        return info

    def step(self,dt):

        if((self.q1-self.q1f)>0.001 and (self.q2-self.q2f)>0.001):
            self.q1+=self.q1*dt
            self.q2+=self.q2*dt
       




    def draw(self,image):
        end1=(int(100+self.l1*np.cos(self.q1)),int(300+self.l1*np.sin(self.q1)))
        end2=(int(end1[0]+self.l2*np.cos(self.q2)),int(end1[1]+self.l2*np.sin(self.q2)))

        cv2.line(image,(100,300),end1,(0,0,255),1)
        cv2.line(image,end1,end2,(0,255,0),1)
        cv2.line(image,(300,-500),(300,500),(0,255,0),5)      #wall

        return image

manipulator=Manipulator(-100,-100,200,200)

for i in range(200):
    manipulator.step(0.2)
    manipulator.render()