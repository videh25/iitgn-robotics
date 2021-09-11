

from render import Renderer
import numpy as np
import cv2 
import scipy
from scipy.integrate import odeint



class Elbow(Renderer):
    def __init__(self):
        super().__init__()

        self.ode = scipy.integrate.ode(self.func).set_integrator('vode', nsteps = 500, method = 'bdf')

        self.q1 = 0
        self.q2 = 0
        self.q1dot = 0
        self.q2dot = 0

        self.l1 = 100
        self.l2 = 100
        self.k = 1

        self.I1 = 200
        self.I2 = 200



    def func(self, t, y):

        q1 = y[0]
        q2 = y[1]

        q1dot = y[2]
        q2dot = y[3]

        self.x1 = int(300 + self.l1 * np.cos(self.q1 - np.pi/4))   
        self.y1 = int(300 + self.l1 * np.sin(self.q1 - np.pi/4))   

        self.x2 = int(self.l2 * np.cos(self.q2 - np.pi/4)) + self.x1  
        self.y2 = int(self.l2 * np.sin(self.q2 - np.pi/4)) + self.y1 


        fx = self.k * self.x2
        fy = self.k * self.y2

        t1 = fy * self.l1 * np.cos(q1) - fx * self.l1 * np.sin(q1)
        t2 = fy * self.l2 * np.cos(q2) - fx * self.l2 * np.sin(q2)

        a = 0.1

        dydt = [q1dot, q2dot, -t1/self.I1 - a * q2dot]


        return dydt

    def step(self, dt):
        state = [self.q1, self.q2, self.q1dot, self.q2dot]

        self.ode.set_initial_value(state, 0)



    def getinfo(self):
        info = {}

        return info

    def draw(self, image):

        cv2.line(image, (200, 200), (self.x1, self.y1), (255,0,0), 1)
        cv2.line(image, (self.x1, self.y1), (self.x2 , self.y2), (255,0,0), 1)
        return image


elbow = Elbow()

for i in range(300):
    elbow.step(0.1)
    elbow.render()
       


    

