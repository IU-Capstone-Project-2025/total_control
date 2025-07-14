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

# K = np.array([[-15, 0, 0.25, 0]])

def control_with_out_observer(x):
    u = (-K @ x)[0] / 3
    # f = np.sign(u) 
    return min(max(u, -max_force), max_force) 


print(K)

# ----------------------------------  experiment  ----------------------------------

print('Press enter to start control')
input()
device.start_experimnet()

while True:
    try:
        res = device.get_joint_state()
        if res:
            res = res.split()

            theta = float(res[0])       / 180 * np.pi  
            theta_dot = float(res[1])   / 180 * np.pi
            p = float(res[2])            
            p_dot = float(res[3])       

            u = control_with_out_observer(np.array([theta, theta_dot, p, p_dot]))

            print(u)
            device.set_joint_efforts(u)
            

    except (Exception) as e:
        print(e)
        continue
