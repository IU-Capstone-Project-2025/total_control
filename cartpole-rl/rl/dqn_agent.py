import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random

class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128), nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.net(x)

class DQNAgent:
    def __init__(self, state_dim, action_dim, epsilon=0.05):
        self.model = DQN(state_dim, action_dim)
        self.epsilon = epsilon
        self.action_dim = action_dim

    def predict(self, state):
    if isinstance(state, list) or isinstance(state, np.ndarray):
        state = torch.tensor(state, dtype=torch.float32)
    if state.ndim == 1:
        state = state.unsqueeze(0)

    if random.random() < self.epsilon:
        return random.randint(0, self.action_dim - 1)
    with torch.no_grad():
        q_values = self.model(state)
        return int(torch.argmax(q_values).item())

    def load(self, path):
        self.model.load_state_dict(torch.load(path))
