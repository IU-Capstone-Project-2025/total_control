# cartpole_rl_physical.py
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecEnv
from typing import List, Optional, Union, Dict, Any, Tuple
import time
import gymnasium as gym
import torch

class PhysicalCartPoleEnv(VecEnv):
    def __init__(self, device, max_steps=500, control_freq=50):
        self.device = device
        self.max_steps = max_steps
        self.control_period = 1.0 / control_freq
        self.num_envs = 1  # Required for VecEnv
        
        # Action space: continuous torque [-1, 1] (will be scaled to actual torque)
        self.action_space = gym.spaces.Box(
            low=-100.0, 
            high=100.0, 
            shape=(1,), 
            dtype=np.float32
        )
        
        # Observation space remains the same
        self.observation_space = gym.spaces.Box(
            low=np.array([-2.4, -3.0, -0.26, -3.0]),
            high=np.array([2.4, 3.0, 0.26, 3.0]),
            dtype=np.float32
        )
        
        self.filtered_obs = np.zeros(4)
        self.current_step = 0
        
        # Safety limits
        self.pos_limit = 0.16    # meters from center
        self.torque_limit = 100  # Nm

    # Required VecEnv abstract methods
    def env_is_wrapped(self, wrapper_class, indices=None) -> List[bool]:
        """Check if environments are wrapped"""
        return [False]  # No wrappers applied

    def get_attr(self, attr_name: str, indices=None) -> List[Any]:
        """Get attribute from environments"""
        if hasattr(self, attr_name):
            return [getattr(self, attr_name)]
        raise AttributeError(f"Attribute {attr_name} not found")

    def set_attr(self, attr_name: str, value: Any, indices=None) -> None:
        """Set attribute in environments"""
        if hasattr(self, attr_name):
            setattr(self, attr_name, value)
        else:
            raise AttributeError(f"Attribute {attr_name} not found")

    def step_async(self, actions: np.ndarray) -> None:
        """Asynchronous step - store actions for next step_wait"""
        self.stored_actions = actions

    def step_wait(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[Dict]]:
        """Execute stored actions from step_async"""
        return self.step(self.stored_actions)[:4]  # obs, rewards, dones, infos

    def env_method(self, 
                 method_name: str, 
                 *method_args, 
                 indices=None, 
                 **method_kwargs) -> List[Any]:
        """Call instance methods of vectorized environments"""
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            return [method(*method_args, **method_kwargs)]
        raise AttributeError(f"Method {method_name} not found")

    def seed(self, seed: Optional[int] = None) -> List[Union[None, int]]:
        """Set random seed - not applicable for physical device"""
        return [None]

    # Core environment methods
    def reset(self) -> np.ndarray:
        """Reset the physical device and return observation"""
        self.device.re_init()  # Zero torque
        raw_obs = self._read_sensors()
        self.filtered_obs = raw_obs
        self.current_step = 0
        return np.array([self.filtered_obs])

    def step(self, actions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[Dict]]:
        """Execute one control step"""
        start_time = time.time()
        
        # Apply action with safety checks
        torque = np.clip(actions[0], -self.torque_limit, self.torque_limit)
        self.device.set_joint_efforts(torque)
        
        # Read sensors
        try:
            raw_obs = self._read_sensors()
        except:
            raw_obs = [0,0,0,0]
        # Apply low-pass filter
        self.filtered_obs = 0.8 * self.filtered_obs + 0.2 * raw_obs
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check termination
        done = self._check_done()
        self.current_step += 1
        
        # Maintain control frequency
        elapsed = time.time() - start_time
        if elapsed < self.control_period:
            time.sleep(self.control_period - elapsed)
        
        info = {"raw_obs": raw_obs, "filtered_obs": self.filtered_obs.copy()}
        return np.array([self.filtered_obs]), np.array([reward]), np.array([done]), [info]

    def close(self) -> None:
        """Cleanup environment resources"""
        self.device.set_joint_efforts(0)
        self.device.disconnect()

    # Helper methods
    def _read_sensors(self) -> np.ndarray:
        """Read and preprocess sensor data"""
        
        
        
        try:
            x = self.device.get_joint_state().split()
            if not x:
                return None
            state = np.array(
            [float(x[2]) / 1000, 
             float(x[3]) / 1000, 
             float(x[0]) / 180 * np.pi, 
             float(x[1]) / 180 * np.pi], 
             dtype=np.float32)
            return state
        except Exception as e:
            print(e)
            raise e
    def _calculate_reward(self) -> float:
        """Custom reward function"""
        pos, vel, theta, theta_dot = self.filtered_obs
        return 1.0 - (theta**2 + 0.1*pos**2 + 0.001*theta_dot**2)

    def _check_done(self) -> bool:
        """Check termination conditions"""
        pos, _, theta, _ = self.filtered_obs
        return (
            abs(pos) > self.pos_limit or 
            abs(theta) > 0.26 or  # ~15 degrees
            self.current_step >= self.max_steps
        )

    # Additional useful methods
    def get_raw_observation(self) -> np.ndarray:
        """Direct sensor reading without filtering"""
        return self._read_sensors()

    def set_safety_limits(self, pos_limit: float, torque_limit: float) -> None:
        """Dynamically adjust safety limits"""
        self.pos_limit = pos_limit
        self.torque_limit = torque_limit





# Initialize hardware
from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')
device = CartPole(port if port else find_your_device())
device.connect(do_init_activity=True)


# Create environment
env = PhysicalCartPoleEnv(device, control_freq=50)  # 30Hz control

# Train or load existing model
# try:
#     model = PPO.load("cartpole_ppo_physical", env=env)
#     print("Loaded existing model")
# except FileNotFoundError:
model = PPO(
    "MlpPolicy",
    env,
    policy_kwargs={
        'net_arch': [dict(pi=[64, 64], vf=[64, 64])],
        'activation_fn': torch.nn.Tanh
    },
    learning_rate=3e-4,
    n_steps=256,
    batch_size=64,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,
    verbose=1
)
print("Created new model")

# Training callback
class SafetyCallback:
    def __init__(self, env):
        self.env = env
    
    def __call__(self, _locals, _globals):
        if self.env.filtered_obs[0] > 0.4 * self.env.pos_limit:
            print("Warning: Approaching position limit!")
        return True

device.start_experimnet()
# Train with safety monitoring
model.learn(
    total_timesteps=20_000,
    callback=SafetyCallback(env),
    progress_bar=True
)

# Save the trained policy
model.save("cartpole_ppo_physical")

# Run trained policy
obs = env.reset()
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, _, done, _ = env.step(action)
    if done:
        obs = env.reset()