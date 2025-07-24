import time
import inno_control
import inno_control.mujoco
import numpy as np
import mujoco


sim = inno_control.mujoco.Simulation()


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

while True:
    try:
        res = sim.get_state()
        if res:
            # print(res)
            theta = float(res[2])
            theta_dot = float(res[3])
            p = float(res[0])        
            p_dot = float(res[1])

            u = control_with_out_observer(np.array([-theta, -theta_dot, p, p_dot]))
            print(theta, theta_dot, p, p_dot)
            print(u)
            sim.set_control(u*2)
            
    except (Exception) as e:
        print(e)
        continue















# import mujoco
# import mujoco.viewer
# import numpy as np
# import os

# # Загрузка модели
# model_path = os.path.join(os.path.dirname(__file__), '..', '..', 'inno_control', 'src', 'inno_control', 'models', 'cart_pole', 'cart-pole.xml')
# model_path = os.path.abspath(model_path)
# model = mujoco.MjModel.from_xml_path(model_path)
# data = mujoco.MjData(model)
# # # Пример управления: постоянно подаём положительный ток на мотор каретки
# # def control(data):
# #     data.ctrl[0] = 5.0  # это индекс actuators[0] — `carriage_motor`

# # Открываем визуализатор и запускаем симуляцию
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     while viewer.is_running():
#         # control(data)                  # применяем управляющее воздействие
#         mujoco.mj_step(model, data)   # один шаг симуляции
#         viewer.sync()