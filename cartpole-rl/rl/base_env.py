class BaseCartPoleEnv:
    def reset(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError

    def close(self):
        pass
