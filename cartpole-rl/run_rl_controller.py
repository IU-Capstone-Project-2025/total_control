from rl.real_cartpole_env import RealCartPoleEnv
from rl.dqn_agent import DQNAgent
from rl.utils import load_model
import time

env = RealCartPoleEnv()
agent = DQNAgent(state_dim=4, action_dim=3)
load_model(agent.model, "models/dqn_cartpole.pth")

state = env.reset()
done = False

while not done:
    action = agent.act(state, epsilon=0.0)
    state, _, done, _ = env.step(action)
    time.sleep(0.02)

env.close()
