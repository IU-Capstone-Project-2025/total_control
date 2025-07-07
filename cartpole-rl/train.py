from rl.real_cartpole_env import RealCartPoleEnv
from rl.dqn_agent import DQNAgent
import time

env = RealCartPoleEnv()
agent = DQNAgent(state_dim=4, action_dim=3)

episodes = 200

for ep in range(episodes):
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = agent.act(state, epsilon=0.1)
        next_state, reward, done, _ = env.step(action)
        agent.remember(state, action, reward, next_state, done)
        agent.replay()
        state = next_state
        total_reward += reward

    agent.update_target_model()
    print(f"Episode {ep+1}: Total reward = {total_reward:.2f}")

env.close()
