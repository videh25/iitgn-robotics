from render import Renderer
import numpy as np
import cv2

class Elbow(Renderer):

    def __init__(self, recordLocation= None):
        super().__init__(recordLocation=recordLocation)
        self.q1=np.pi/4
        self.q2=np.pi/4
        self.l1=300
        self.l2=300
        self.t=0
        self.trajectory=[]

    def step(self,dt):
        self.t+=dt
        #defining Vertical SHM (Sinosoidal trajectory on a vertical straight line path)
        x=300
        y=100+150*np.sin(0.3*self.t)

        theta=np.arccos((x**2+y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        self.q1=np.arctan(y/x)-np.arctan((self.l2*np.sin(theta))/(self.l1+self.l2*np.cos(theta)))
        self.q2=theta+self.q1
        

    def draw(self,image):
        end1=(int(100+self.l1*np.cos(self.q1)),int(300+self.l1*np.sin(self.q1)))
        end2=(int(end1[0]+self.l2*np.cos(self.q2)),int(end1[1]+self.l2*np.sin(self.q2)))

        cv2.line(image,(100,300),end1,(0,0,255),1)
        cv2.line(image,end1,end2,(0,255,0),1)

        #trajectory tracking
        self.trajectory.append(end2)
        for i in self.trajectory:
            cv2.circle(image,i,5,(255,255,0),-1)
        return image

    def getInfo(self):
        return {}

elbow=Elbow(recordLocation='Task1video.mp4')

for i in range(500):
    elbow.step(0.1)
    elbow.render()