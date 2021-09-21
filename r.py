from operator import eq
import numpy as np
from sympy import *

n = int(input("no. of links: "))

q_ddot = [sympify("q{}_ddot".format(i)) for i in range(n)] 
q_dot = [sympify("q{}_dot".format(i)) for i in range(n)]   
q = [sympify("q{}".format(i)) for i in range(n)]

Vq = sympify(input("V(q): "))
Dq = eye(n)

for i in range(n):
    for j in range(n):
        Dq[i,j] = sympify(input("enter d({},{}): ".format(i,j))) 

for i in range(n):
    var1 = 0
    var2 = 0
    
    for j in range(n):
        var1 += Dq[i,j]*q_ddot[j]
    
    for k in range(n):

        for l in range(n):
            var2 += (diff(Dq[i,l],q[k])-0.5*diff(Dq[k,l],q[i]))*q_dot[k]*q_dot[l]

    var3 = diff(Vq,q[i])
    torque = sympify("T({})".format(i))
    resultant = var1 + var2 - var3
    print("{} = {}".format(sympify(resultant),torque))