import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

# ---------------------- Модель системы ----------------------
# Математическая модель маятника на тележке с трением:
# Состояние x = [p, p_dot, theta, theta_dot]
# Уравнения движения:
#   (M + m) * p_ddot + c_visc * p_dot + c_coulomb * sign(p_dot) - m * l * theta_ddot * cos(theta) + m * l * theta_dot^2 * sin(theta) = u
#   l * theta_ddot + c_visc_pole * theta_dot + g * sin(theta) - p_ddot * cos(theta) = 0
# где:
#   p — положение тележки,
#   theta — угол маятника от вертикали,
#   M — масса тележки, m — масса маятника,
#   l — длина маятника,
#   u — приложенная горизонтальная сила,
#   c_visc — коэффициент вязкого трения тележки,
#   c_coulomb — коэффициент сухого трения тележки,
#   c_visc_pole — вязкое трение в шарнире маятника.
# Решая эту систему относительно p_ddot и theta_ddot, получаем динамику полного состояния.

# ---------------------- Реальное Gym-среда ----------------------
class RealCartPoleEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, port=None):
        super().__init__()
        # Инициализация устройства
        if port:
            self.device = CartPole(port)
        else:
            self.device = CartPole(find_your_device())
        self.device.connect(do_init_activity=False)
        # Дискретное (или непрерывное) пространство действий
        # Здесь непрерывное: ток от -120 до 120
        self.action_space = spaces.Box(low=-120.0, high=120.0, shape=(1,), dtype=np.float32)
        # Пространство состояний: [theta, theta_dot, p, p_dot]
        high = np.array([np.pi, np.finfo(np.float32).max, np.finfo(np.float32).max, np.finfo(np.float32).max], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

    def reset(self, seed=None, options=None):
        # Сброс эксперимента
        self.device.re_init()
        # Читаем сразу начальное состояние
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        # Отправляем команду усилия
        u = float(np.clip(action, self.action_space.low, self.action_space.high))
        self.device.set_joint_efforts(u)
        # Добавляем небольшую задержку для устройства
        time.sleep(0.02)  # 20 мс

        # Читаем новое состояние
        obs = self._get_obs()
        theta, theta_dot, p, p_dot = obs
        # Завершение эпизода при больших отклонениях
        done = bool(abs(theta) > np.pi/6)
        # Примерная функция награды: держим маятник вертикально
        reward = 1.0 if not done else -10.0
        return obs, reward, done, False, {}

    def _get_obs(self):
        s = self.device.get_joint_state()
        if s is None:
            return np.zeros(4, dtype=np.float32)
        vals = list(map(float, s.split()))
        theta = vals[0] / 180 * np.pi
        theta_dot = vals[1] / 180 * np.pi
        p = vals[2] / 1000
        p_dot = vals[3] / 1000
        return np.array([theta, theta_dot, p, p_dot], dtype=np.float32)

    def render(self):
        # Реальный контроллер управляет визуализацией на устройстве
        pass

    def close(self):
        self.device.stop_experiment()
        self.device.disconnect()

# ---------------------- Пример использования ----------------------
if __name__ == '__main__':
    # Ввод порта и создание среды
    port = '/dev/ttyUSB0'  # Замените на ваш порт
    env = RealCartPoleEnv(port)
    obs, _ = env.reset()
    done = False
    total_reward = 0.0
    while not done:
        # Случайное действие (пример), заменить на PID/LQR/RL политику
        action = env.action_space.sample()
        obs, reward, done, truncated, info = env.step(action)
        total_reward += reward
        print(f"Obs: {obs}, Action: {action}, Reward: {reward}, Done: {done}")
    print(f"Episode finished, total reward: {total_reward}")
    env.close()

