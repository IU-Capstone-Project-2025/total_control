from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')



if port:
    device = CartPole(port)
else:
    device = CartPole(find_your_device())


device.connect(do_init_activity = True)

max_force = 110


# ----------------------------------  theoretical control  ----------------------------------

m_p = 0.38 # mass of pendulum bob
m_c = 0.7 # mass of cart
M = m_c + m_p
l = 0.36 # length of pendulum
g = 9.81 

import numpy as np
import control as ct


A = np.array([[0, 1, 0, 0],
             [g/l/(4/3 - m_p / (m_p + m_c)), 0, 0, 0],
             [0,0,0,1],
             [(m_p * l) / (m_p + m_c) * (1 - g / (l * (4/3 - m_p/(m_p+m_c)))), 0, 0, 0]])

B = np.array([[0],
             [1/(l / (m_c + m_p) * (4/3 - m_p/(m_p+m_c)))],
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

def control_with_out_observer(x):
    u = (-K @ x)[0]
    s = np.sign(u)
    u = 2.7 ** (abs(u))
    return min(max(s*(40 + u), -120), 120)


K[0][1] = 0
K[0][2] = 0
K[0][3] = 0

# ----------------------------------  experiment  ----------------------------------

print('Press enter to start control')
input()
device.start_experimnet()

while True:
    try:
        res = device.get_joint_state()
        if res:
            res = res.split()

            # print(res)
            theta = float(res[0])         / 180 * np.pi  
            theta_dot = float(res[1])     / 180 * np.pi
            p = float(res[2])             / 1000           
            p_dot = float(res[3])         / 1000

            u = control_with_out_observer(np.array([-theta, -theta_dot, p, p_dot]))

            print(u)
            device.set_joint_efforts(u)
            

    except (Exception) as e:
        print(e)
        continue
