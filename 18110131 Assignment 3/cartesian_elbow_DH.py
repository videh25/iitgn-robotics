#Answers for Q5 and Q6

#Testing cartesian and elbow with spherical wrist.
#Make sure that DH_jacobian_calculator.py is in same folder.

#Imports
from DH_jacobian_calculator import Jacobian_EndPoint_DH
import numpy as np
import os

#Link Lengths
l1=5
l2=3

def Cartesian_DH(Q):
    d1=Q[0,0]
    d2=Q[1,0]
    d3=Q[2,0]
    linkCount=3

    l0=0.1

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='P'
    DH_param[0:1,1:5]=np.array([0,np.pi/2   ,l0+d1 ,np.pi/2])
    DH_param[1,0]='P'
    DH_param[1:2,1:5]=np.array([0,-np.pi/2  ,d2   ,np.pi/2])
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0         ,d3   ,0]) 

    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(linkCount,DH_param,P)
    print("Jacobian :")
    print(np.round_(J0_P,decimals=5))
    print("End Position :")
    print(P0)


def Elbow_DH(Q):
    q1=Q[0,0]
    q2=Q[1,0]
    q3=Q[2,0]
    q4=Q[3,0]
    q5=Q[4,0]
    q6=Q[5,0]
    l1=l2=l3=2
    l4=l5=0.5
    l6=0.1

    linkCount=6

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([0,np.pi/2   ,l1     ,q1     ])
    DH_param[1,0]='R'
    DH_param[1:2,1:5]=np.array([l2,0        ,0      ,q2     ])
    DH_param[2,0]='R'
    DH_param[2:3,1:5]=np.array([l3,0        ,0      ,q3+np.pi/2]) 
    DH_param[3,0]='R'
    DH_param[3:4,1:5]=np.array([l4,-np.pi/2 ,0      ,q4-np.pi/2])
    DH_param[4,0]='R'
    DH_param[4:5,1:5]=np.array([l5,np.pi/2  ,-l4    ,q5+np.pi/2])
    DH_param[5,0]='R'
    DH_param[5:6,1:5]=np.array([l6,0        ,0      ,q6     ])

    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(linkCount,DH_param,P)
    print("Jacobian :")
    print(np.round_(J0_P,decimals=5))
    print("End Position :")
    print(P0)

#Main function calling above 3 parts
if __name__=="__main__":
    userInput=0

    #Joint variables
    Q1=np.array([[1],[1.5],[0.5]])
    Q2=np.array([[np.pi/3],[np.pi/6],[np.pi/4],[np.pi/2],[np.pi/2],[0]])

    while not(userInput == 3):
        print("Choose Manipulator : \n1. Cartesian\n2. Elbow\n3. Exit\nEnter index for respective result: ")
        userInput=int(str(input()))
        if userInput==1:
            Cartesian_DH(Q1)           
        elif userInput==2:
            Elbow_DH(Q2)
        elif userInput==3:
            os._exit(0)
        else:
            print("Invalid Input. Enter again\n")
    os._exit(0)
