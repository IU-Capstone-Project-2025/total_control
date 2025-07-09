import torch
import os

def normalize_state(state, mean=None, std=None):
    
    state = torch.tensor(state, dtype=torch.float32)
    if mean is not None and std is not None:
        state = (state - mean) / (std + 1e-8)
    return state

def save_model(model, filepath):
    
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    torch.save(model.state_dict(), filepath)

def load_model(model, filepath):

    model.load_state_dict(torch.load(filepath))
    model.eval()
    return model
