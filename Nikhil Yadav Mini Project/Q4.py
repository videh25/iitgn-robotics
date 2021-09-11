

from render import Renderer
import numpy as np
import cv2 
import matplotlib.pyplot as plt


class Elbow(Renderer):
    def __init__(self):
        super().__init__()
        self.l1 = 100       #dimensions of link
        self.l2 = 100
        self.q1 = np.pi*35/180      #initial angle of link
        self.q2 = np.pi*35/180
        self.x = []     # to take input of all cordinates for graph
        self.y = []

                
    def step(self):
        
        x = self.l1 * np.cos(self.q1) + self.l2 * np.cos(self.q2)   #end effector
        y = self.l1 * np.sin(self.q1) + self.l2 * np.sin(self.q2)

        self.x.append(x)   
        self.y.append(y)

elbow = Elbow()


while (elbow.q1 <= np.pi*145/180):
    while (elbow.q2 <= np.pi*145/180):
        elbow.step()
        elbow.q2 += 0.005

    elbow.q1 += 0.005
    elbow.q2 = np.pi*35/180

plt.scatter(elbow.x,elbow.y)
plt.show()

