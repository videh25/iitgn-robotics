import numpy as np


def cp(a, b):
    c = np.array([[0, a[2], a[1]], [-a[2], 0, a[0]], [-a[1], -a[0], 0]])
    return np.matmul(c, b)


# Link Parameters:    [a, alpha, d, theta]
# For Multiple Links: [[a, alpha, d, theta]
#                      [a, alpha, d, theta]
#                      [a, alpha, d, theta]]

def atransformation(LinkParameters):
    a = LinkParameters[0]
    d = LinkParameters[2]
    alpha = LinkParameters[1]
    theta = LinkParameters[3]
    A = np.array([[[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.cos(alpha), a * np.cos(theta)],
                   [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                   [0, np.sin(alpha), np.cos(alpha), d],
                   [0, 0, 0, 1]]])
    return (A)


def dhconvention(n, LinkParameters):
    A = np.zeros([n, 4, 4])
    for x in range(n):
        A[x-1] = atransformation(LinkParameters[n-1])
    T = np.zeros([n, 4, 4])
    for x in range(n):
        B = np.identity(4)
        for i in range(x+1):
            B = np.matmul(B, A[i])
        T[x] = B
    return T

# Optional argument to define linear joints, can be added as :
# [0, 0, 0, 1, 1]
# [R, R, R, L, L]

def jacobian(n, T, type=0):
    o = np.zeros([n, 4])
    for i in range(n):
        o[i] = np.matmul(T[i], np.array([0, 0, 0, 1]))

    o = np.delete(o, 3, 1)
    z = np.zeros([n, 4])
    for j in range(n):
        z[j] = np.matmul(T[j], np.array([0, 0, 1, 0]))

    z = np.delete(z, 3, 1)

    J = np.zeros([6, n])
    for k in range(n):
        if type == 0:
            c1 = cp(z[k], (o[n - 1] - o[k]))
            c2 = z[k]
        else:
            if type[k] == 1:
                c1 = z[k]
                c2 = [0, 0, 0]
            else:
                c1 = cp(z[k], (o[n - 1] - o[k]))
                c2 = z[k]

        J[:, k] = np.hstack((c1, c2))
    return o, z, J


# Question 4, Jacobian for Stanford RRP Manipulator

RRPParameters = [[0, 0, 20, np.pi], [20, 10, 0, 0], [30, 0, 0, 0]]
a = dhconvention(3, RRPParameters)
b = jacobian(3, a, [0, 0, 1])
print(b)

SCARAParameters = [[0, 0, 20, np.pi], [200, 0, 0, np.pi], [30, 0, 0, np.pi], [0, 0, 50, np.pi/2]];
a1 = dhconvention(4, SCARAParameters)
b1 = jacobian(4, a1)
print(b1)


# Question 5, Jacobian for textbook 3-7

Q5Parameters = [[0, -np.pi/2, 10, 0], [0, -np.pi/2, 20, 0], [0, 0, 30, 0]];
a2 = dhconvention(3, Q5Parameters)
b2 = jacobian(3, a1, [1, 1, 1])
print(b2)

# Question 6, Jacobian for textbook 3-8

Q6Parameters = [[0, -np.pi/2, 10, (np.pi*2)/3], [20, 0, 20, np.pi/2], [30, 0, 0, np.pi], [0, np.pi/2, 0, 0], [0, -np.pi/2, 0, 0], [0, 0, 30, np.pi]];
a3 = dhconvention(6, Q6Parameters)
b3 = jacobian(6, a3, )
print(b3)