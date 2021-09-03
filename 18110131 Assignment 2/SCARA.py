#SCARA Manipulator. Q7, 12

#Imports:
import os
import numpy as np

#Link Lengths
l1=5
l2=3

#to make skew symmetric from vector
def skew(z):
    skew_z=np.zeros([3,3])
    skew_z[0,1]=-z[2,0]
    skew_z[1,0]=z[2,0]
    skew_z[0,2]=z[1,0]
    skew_z[2,0]=-z[1,0]
    skew_z[1,2]=-z[0,0]
    skew_z[2,1]=z[0,0]
    return skew_z

# End effector Position
def end_position(Q):
    q1=Q[0,0]
    q2=Q[1,0]
    d3=Q[2,0]

    #Frame transform o_1:
    R0_1=np.array([[np.cos(q1),-np.sin(q1),0],[np.sin(q1),np.cos(q1),0],[0,0,1]])
    d0_1=np.array([[0],[0],[0]])
    H0_1=np.zeros([4,4])
    H0_1[3,3]=1 #Last element 
    H0_1[0:3,0:3]=R0_1
    H0_1[0:3,3:4]=d0_1

    #Frame transform 1_2
    R1_2=np.array([[np.cos(q2),-np.sin(q2),0],[np.sin(q2),np.cos(q2),0],[0,0,1]])
    d1_2=np.array([[l1],[0],[0]])
    H1_2=np.zeros([4,4])
    H1_2[3,3]=1 #Last element 
    H1_2[0:3,0:3]=R1_2
    H1_2[0:3,3:4]=d1_2

    #Frame transform 2_3
    R2_3=np.array([[1,0,0],[0,1,0],[0,0,1]])
    d2_3=np.array([[l2],[0],[-d3]])
    H2_3=np.zeros([4,4])
    H2_3[3,3]=1 #Last element 
    H2_3[0:3,0:3]=R2_3
    H2_3[0:3,3:4]=d2_3

    #End effector w.r.t frame 3
    P3=np.array([[0],[0],[0]])

    #End Effector Position w.r.t fixed frame 0
    temp=H0_1@H1_2@H2_3@np.vstack([P3,1])
    P0=temp[0:3]

    print("End Effector Position is:\n")
    print(P0)

    return P0

def jacobian(Q):
    q1=Q[0,0]
    q2=Q[1,0]
    d3=Q[2,0]

    #Frame transform o_1:
    R0_1=np.array([[np.cos(q1),-np.sin(q1),0],[np.sin(q1),np.cos(q1),0],[0,0,1]])
    d0_1=np.array([[0],[0],[0]])
    H0_1=np.zeros([4,4])
    H0_1[3,3]=1 #Last element 
    H0_1[0:3,0:3]=R0_1
    H0_1[0:3,3:4]=d0_1

    #Frame transform 1_2
    R1_2=np.array([[np.cos(q2),-np.sin(q2),0],[np.sin(q2),np.cos(q2),0],[0,0,1]])
    d1_2=np.array([[l1],[0],[0]])
    H1_2=np.zeros([4,4])
    H1_2[3,3]=1 #Last element 
    H1_2[0:3,0:3]=R1_2
    H1_2[0:3,3:4]=d1_2

    #Frame transform 2_3
    R2_3=np.array([[1,0,0],[0,1,0],[0,0,1]])
    d2_3=np.array([[l2],[0],[-d3]])
    H2_3=np.zeros([4,4])
    H2_3[3,3]=1 #Last element 
    H2_3[0:3,0:3]=R2_3
    H2_3[0:3,3:4]=d2_3

    O=np.array([[0],[0],[0],[1]])
    temp=np.matmul(H0_1,O)
    O1=temp[0:3]

    temp=H0_1@H1_2@O
    O2=temp[0:3]

    temp=H0_1@H1_2@H2_3@O
    Op=temp[0:3]

    #Z axis unit vectors
    z=np.array([[0],[0],[1]])
    z1=np.matmul(R0_1,z)
    skew_z1=skew(z1)
    z2=R0_1@R1_2@z
    skew_z2=skew(z2)
    z3=R0_1@R1_2@R2_3@z
    skew_z3=skew(z3)

    J0_P=np.zeros([6,3])
    J0_P[0:3,0:1]=np.matmul(skew_z1,np.subtract(Op,O1))
    J0_P[0:3,1:2]=np.matmul(skew_z2,np.subtract(Op,O2))
    J0_P[0:3,2:3]=z3
    J0_P[3:6,0:1]=z1
    J0_P[3:6,1:2]=z2

    print("Jacobian of SCARA : \n")
    print(J0_P)
    return J0_P

#Main function calling above 3 parts
if __name__=="__main__":
    userInput=0
    q1=np.pi/6
    q2=np.pi/3
    d3=0.5

    #Joint variables
    Q=np.array([[q1],[q2],[d3]])

    while not(userInput == 3):
        print("For SCARA Manipulator : \n1. End Effector Position\n2. Jacobian\n3. Exit\nEnter index for respective result: ")
        userInput=int(str(input()))
        if userInput==1:
            P0=end_position(Q)            
        elif userInput==2:
            J0_P=jacobian(Q)
        elif userInput==3:
            os._exit(0)
        else:
            print("Invalid Input. Enter again\n")
    os._exit(0)