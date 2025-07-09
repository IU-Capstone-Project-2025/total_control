from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')



if port:
    device = CartPole(port)
else:
    device = CartPole(find_your_device())


device.connect(do_init_activity = True)

max_force = 150

centre_ticks = 6251
lenght_ticks = centre_ticks * 2
lenght_meters = 0.472
lenght_coeff = lenght_meters / lenght_ticks

angle_offset = 173

# ----------------------------------  theoretical control  ----------------------------------

m = 0.15 # mass of pendulum bob
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
    return u



# ----------------------------------  experiment  ----------------------------------

print('Press enter to start control')
input()
device.start_experimnet()

while True:
    try:
        res = device.get_joint_state()
        if res:
            # print(res)
            res.split()
            theta =     (float(res[0]) + angle_offset) % 360
            theta_dot = float(res[1])
            p =         int(res[2]) * lenght_coeff
            p_dot =     int(res[3]) * lenght_coeff
            
            device.set_joint_efforts(control_with_out_observer(np.array([theta, theta_dot, p, p_dot])))


    except (Exception) as e:
        print(e)
        continue
