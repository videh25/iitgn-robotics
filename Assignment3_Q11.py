import math
import numpy as np
import scipy
import sympy as sp

def ComputeEOM(D,V,n):
    phi=[0]*n
    c = [[[0] * n] * n] * n
    Tau=[0]*n
    d=0
    ct=0

    for i in range(n):
        for j in range(n):
            for k in range(n):
                c[k][j][i]=0.5*(sp.diff(D[i][j], q[k]) + sp.diff(D[i][k], q[j]) - sp.diff(D[k][j], q[i]))
    for i in range(n):
        phi[i]=sp.diff(V,q[i])
        for j in range(n):
            d += D[i][j]*q_doubledot[j]
            for k in range(n):
                ct+= c[k][j][i]*q_dot[k]*q_dot[j]
        Tau[i]=d+ct+phi[i]
    for i in range(n):
      print('Tau' + str(i+1) + ' = ' + str(Tau[i]))




n=2
# q1=sp.symbols('q1')
# q2=sp.symbols('q2')
# q=np.matrix([[q1],[q2]])
# q1_dot=sp.symbols('q1_dot')
# q2_dot=sp.symbols('q2_dot')
# q_dot=np.matrix([[q1_dot],[q2_dot]])
# q1_doubledot=sp.symbols('q1_doubledot')
# q2_doubledot=sp.symbols('q2_doubledot')
# q_doubledot=([[q1_doubledot],[q2_doubledot]])
q = []
q_dot = []
q_doubledot = []
for i in range(1000):
  q.append(sp.symbols('q' + str(i + 1)))
  q_dot.append(sp.symbols('q"' + str(i + 1)))
  q_doubledot.append(sp.symbols('q""' + str(i + 1)))
l1=sp.symbols('l1')
l2=sp.symbols('l2')
m1=sp.symbols('m1')
m2=sp.symbols('m2')
Tau_1=sp.symbols('Tau_1')
Tau_2=sp.symbols('Tau_2')
g=sp.symbols('g')
D=np.matrix([[(m1*(l1**2))/3+m2*(l1**2),m2*l1*l2*sp.cos(q2-q1)/2], [m2*l1*l2*sp.cos(q2-q1)/2,m2*(l2**2)/3]])
V=m1*g*l1*sp.sin(q1)/2 + m2*g*(l1*sp.sin(q1) + l2*sp.sin(q2)/2)
ComputeEOM(D,V,2)
# n=3
# c = [[[0] * n] * n] * n
# print(c)
