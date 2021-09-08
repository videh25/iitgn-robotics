from render import Renderer
import numpy as np
import cv2 


class Elbow(Renderer):
    def _init_(self):
        super()._init_()
        self.l1 = 100
        self.l2 = 100
        self.q1 = 0
        self.q2 = 0
        self.t = 0
        self.omega = np.pi/6
        
    def step(self, dt):
        self.t += dt
        
        x = 140 * np.sin(self.omega * self.t)
        y = 100
        
        self.theta = np.arccos((x*2 + y**2 -self.l12 - self.l2*2)/(2*self.l1*self.l2))
        
        self.q1 = np.arctan(y/x) - np.arctan(self.l2*np.sin(self.theta)/(self.l1 + self.l2*np.cos(self.theta)))
        
        self.q2 = self.theta + self.q1
        
    def getInfo(self):
        info = {}
        return info
    
    def draw(self, image):
        self.x1 = self.l1 * np.cos(self.q1)   #x1 = length of 1st line in x direction 
        self.y1 = self.l1 * np.sin(self.q1)   # y1 => similarly

        self.x2 = self.l2 * np.cos(self.q2)   #x1 = length of 1st line in x direction 
        self.y2 = self.l2 * np.sin(self.q2)   # y1 => similarly
 

        cv2.line(image, (300, 300), (300 + self.x1, 300 + self.y1), (255,0,0), 1)
        cv2.line(image, (300 + self.x1, 300 + self.y1), (300 + self.x1 + self.x2, 300 + self.y1 + self.y2), (255,0,0), 1)

elbow = Elbow()

for i in range(100):
    elbow.step(0.2)
    elbow.render()