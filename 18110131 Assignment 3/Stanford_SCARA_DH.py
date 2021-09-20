#Answr for q4.
#Testing for Scara and stanford type
#Make sure that DH_jacobian_calculator.py is in same folder.

#Imports
from DH_jacobian_calculator import Jacobian_EndPoint_DH
import numpy as np
import os

#Link Lengths
l1=5
l2=3

def SCARA_DH(Q):
    q1=Q[0,0]
    q2=Q[1,0]
    d3=Q[2,0]
    linkCount=3

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([l1,0,0,q1])
    DH_param[1,0]='R'
    DH_param[1:2,1:5]=np.array([l2,np.pi,0,q2])
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0,d3,0]) 

    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(linkCount,DH_param,P)
    print("Jacobian :")
    print(np.round_(J0_P,decimals=5))
    print("End Position :")
    print(P0)


def Stanford_DH(Q):
    q1=Q[0,0]
    q2=Q[1,0]
    d3=Q[2,0]
    linkCount=3

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([0,np.pi/2,l1,q1])
    DH_param[1,0]='R'
    DH_param[1:2,1:5]=np.array([0,np.pi/2,l2,q2+np.pi/2])#q2+pi/2 so that x aligns with y axis and we can then rotate z on to prismatic side
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0,d3,0]) 

    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(linkCount,DH_param,P)
    print("Jacobian :")
    print(np.round_(J0_P,decimals=5))
    print("End Position :")
    print(P0)

#Main function calling above 3 parts
if __name__=="__main__":
    userInput=0
    q1=np.pi/6
    q2=np.pi/3
    d3=0.5

    #Joint variables
    Q=np.array([[q1],[q2],[d3]])

    while not(userInput == 3):
        print("Choose Manipulator : \n1. SCARA\n2. Stanford\n3. Exit\nEnter index for respective result: ")
        userInput=int(str(input()))
        if userInput==1:
            SCARA_DH(Q)           
        elif userInput==2:
            Stanford_DH(Q)
        elif userInput==3:
            os._exit(0)
        else:
            print("Invalid Input. Enter again\n")
    os._exit(0)
