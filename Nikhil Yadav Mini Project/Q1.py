

from render import Renderer
import numpy as np
import cv2 


class Elbow(Renderer):
    def __init__(self):
        super().__init__()
        self.l1 = 300
        self.l2 = 300
        self.q1 = 0
        self.q2 = 0
        self.t = 0
        self.omega = np.pi/6
        
    def step(self, dt):
        self.t += dt
        
        x = 200 + 100 * np.sin(self.omega * self.t)
        y = 100
        
        self.theta = np.arccos((x**2 + y**2 -self.l1**2 - self.l2**2)/(2*self.l1*self.l2))
        
        self.q1 = np.arctan(y/x) - np.arctan(self.l2*np.sin(self.theta)/(self.l1 + self.l2*np.cos(self.theta)))
        
        self.q2 = self.theta + self.q1
        
    def getInfo(self):
    
        return {}
    
    def draw(self, image):
        self.x1 = int(200 + self.l1 * np.cos(self.q1))   
        self.y1 = int(200 + self.l1 * np.sin(self.q1))   

        self.x2 = int(self.l2 * np.cos(self.q2)) + self.x1  
        self.y2 = int(self.l2 * np.sin(self.q2)) + self.y1 
        
        #x1y1 = (300 + self.l1 * np.cos(self.q1), 100 + self.l1 * np.sin(self.q1)))   #x1 = length of 1st line in x direction 
        

        #x2y2 = int((x1y1[0] + self.l2 * np.cos(self.q2), x1y1[1] + self.l2 * np.sin(self.q2)))   #x1 = length of 2nd line in x direction 
        # y2 =    # y1 => similarly
 

        cv2.line(image, (200, 200), (self.x1, self.y1), (255,0,0), 1)
        cv2.line(image, (self.x1, self.y1), (self.x2 , self.y2), (255,0,0), 1)
        return image


elbow = Elbow()

for i in range(300):
    elbow.step(0.1)
    elbow.render()
       


    