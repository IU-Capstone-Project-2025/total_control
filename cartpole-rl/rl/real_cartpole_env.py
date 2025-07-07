

import numpy as np
from rl.base_env import BaseCartPoleEnv
from inno_control.devices import CartPole
import time

class RealCartPoleEnv(BaseCartPoleEnv):
    def __init__(self, port="/dev/cu.usbserial-110"):
        super().__init__()
        self.device = CartPole(port)
        self.device.connect()
        self.device.start_experimnet()

        self.action_space = np.array([-1.0, 0.0, 1.0])
        self.max_force = 90
        self.max_steps = 1000
        self.state = None
        self.step_count = 0

    def reset(self):
        self.step_count = 0
        self.device.start_experimnet()
        time.sleep(0.5)
        self.state = self._get_state()
        return self.state

    def step(self, action_index):
        force = self.action_space[action_index] * self.max_force
        self.device.set_joint_efforts(force)
        time.sleep(0.02)
        next_state = self._get_state()

        x, x_dot, theta, theta_dot = next_state
        reward = 1.0 - abs(theta) * 10.0
        done = abs(theta) > 0.2 or abs(x) > 0.4 or self.step_count >= self.max_steps

        self.step_count += 1
        self.state = next_state
        return next_state, reward, done, {}

    def _get_state(self):
        try:
            state_str = self.device.get_joint_state()
            return np.array([float(x) for x in state_str.split()])
        except Exception as e:
            print(f"[ERROR] Failed to parse joint state: {e}")
            return np.zeros(4)

    def close(self):
        self.device.set_joint_efforts(0)
