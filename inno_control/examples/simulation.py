

import numpy as np
import control as ct

m_p = 0.077 # mass of pendulum bob
m_c = 0.4 # mass of cart
M = m_c + m_p
l = 0.36 # length of pendulum
g = 9.81 

A = np.array([[0, 1, 0, 0],
             [g/l/(4/3 - m_p / (m_p + m_c)), 0, 0, 0],
             [0,0,0,1],
             [(m_p * l) / (m_p + m_c) * (1 - g / (l * (4/3 - m_p/(m_p+m_c)))), 0, 0, 0]])

B = np.array([[0],
             [1/(l / (m_c+m_p) * (4/3 - m_p/(m_p+m_c)))],
             [0],
             [1/(m_c + m_p)]])


# B_plus = np.linalg.pinv(B)

C = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])



K, L = None, None

def LQR(): 
    global K, L
    Q = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    R = [1]

    K, _, _ = ct.lqr(A, B, Q, R)

    R = [[1,0,0], [0,1,0], [0,0,1]]
    L, _, _ = ct.lqr(A.transpose(), C.transpose(), Q, R)
    L = L.transpose()

LQR()


from scipy.integrate import odeint
import matplotlib.pyplot as plt

def LTI(x, t):
    return A.dot(x) - B @ K.dot(x)

time = np.linspace(0, 10, 1000)    # interval from 0 to 10
x0 = np.random.rand(4)            # initial state

solution = odeint(LTI, x0, time)

plt.plot(time, solution)
plt.xlabel('time')
plt.ylabel('x(t)')
plt.grid(True)
plt.show()