import matplotlib.pyplot as plt
import numpy as np

class Manipulator():
    def __init__(self):
        super().__init__()
        self.l1=100
        self.l2=150
        self.q1=35*np.pi/180
        self.q2=35*np.pi/180

        self.xarray=[]
        self.yarray=[]

    def step(self):
        x=self.l1*np.cos(self.q1)+self.l2*np.cos(self.q2)
        y=self.l1*np.sin(self.q1)+self.l2*np.sin(self.q2)
        self.xarray.append(x)
        self.yarray.append(y)


manipulator=Manipulator()

while(manipulator.q1<=145*np.pi/180):
    while(manipulator.q2<=145*np.pi/180):
        manipulator.step()
        manipulator.q2+=0.005
    manipulator.q2=35*np.pi/180 #resetting q2
    manipulator.q1+=0.005 

plt.scatter(manipulator.xarray,manipulator.yarray,marker='.',color='b')
plt.show()

        
