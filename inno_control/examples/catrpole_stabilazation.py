from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')



if port:
    device = CartPole(port)
else:
    device = CartPole(find_your_device())


device.connect(do_init_activity = True)

max_force = 100

lenght_ticks = 6251 * 2
lenght_meters = 0.472
lenght_coeff = lenght_meters / lenght_ticks

angle_offset = 173

# ----------------------------------  theoretical control  ----------------------------------

m = 0.077 # mass of pendulum bob
M = 0.4 # mass of cart
pendulumn_length = 0.36 # length of pendulum
g = 9.81 

import numpy as np
import control as ct


A = np.array([[0, 1, 0, 0],
             [(M + m)*g /(M*pendulumn_length), 0, 0, 0],
             [0,0,0,1],
             [m*g/M, 0, 0, 0]])

B = np.array([[0],
             [1/(M*pendulumn_length)],
             [0],
             [1/M]])

B_plus = np.linalg.pinv(B)

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
    u = -K @ x
    f = np.sign(u) 
    return min(max(50 * f + u, -max_force), max_force) 


# print(K)

# ----------------------------------  experiment  ----------------------------------

print('Press enter to start control')
input()
device.start_experimnet()

while True:
    try:
        res = device.get_joint_state()
        if res:
            res = res.split()
            
            theta = float(res[0]) / 360 * 2 * np.pi - 1.7
            if theta > np.pi:
                theta = np.pi * 2 - theta
            elif theta < -np.pi:
                theta = np.pi * 2 + theta

            # print(theta)
            theta_dot = float(res[1]) / 360 * 2 * np.pi
            p = int(res[2]) * lenght_coeff
            p_dot = int(res[3]) * lenght_coeff

            # print(control_with_out_observer(np.array([theta, theta_dot, p, p_dot])))
            device.set_joint_efforts(control_with_out_observer(np.array([theta, theta_dot, p, p_dot])))
            

    except (Exception) as e:
        print(e)
        continue
