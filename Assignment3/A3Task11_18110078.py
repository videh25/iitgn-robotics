import sympy as sym
import numpy as np

# Define required parameters here.
l1 = sym.Symbol('l1')
l2 = sym.Symbol('l2')
m1 = sym.Symbol('m1')
m2 = sym.Symbol('m2')
g = sym.Symbol('g')
I1 = sym.Symbol('I1')
I2 = sym.Symbol('I2')

# Write Number of states
n = 2
q1 = sym.Symbol('q1')
q2 = sym.Symbol('q2')

Q = [q1, q2]

# write D(q) and V(q) matrix in terms of defined parameters.

d11 = (m1*l1*l1)/4 + m2*l1*l1 + I1
d12 = (m2*l1*l2*sym.cos(q2 - q1))/2
d21 = (m2*l1*l2*sym.cos(q2 - q1))/2
d22 = (m2*l2*l2)/4 + I2

D = [[d11, d12], [d21, d22]]

V = sym.Symbol('V')

phi = np.zeros(n)

C = np.zeros([n, n, n])
for k in range(n):
    for i in range(n):
        for j in range(n):
            C[k][i][j] = 1/2*(sym.diff(D[k][j], Q[i]) + sym.diff(D[k][i], Q[j] - sym.diff(D[i][j], Q[k])))
    phi[k] = sym.diff(V[k], Q[k])

