"""DQN with experience replay, a frozen target network and Huber loss."""
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import torch
from torch import nn


@dataclass
class Config:
    hidden: int = 128
    gamma: float = 0.98
    learning_rate: float = 0.001
    capacity: int = 50_000
    batch_size: int = 64
    target_interval: int = 250


class Network(nn.Module):
    def __init__(self, inputs, hidden=128):
        super().__init__()
        self.layers = nn.Sequential(nn.Linear(inputs, hidden), nn.ReLU(),
                                    nn.Linear(hidden, hidden), nn.ReLU(),
                                    nn.Linear(hidden, 4))

    def forward(self, state):
        return self.layers(state)


class Replay:
    def __init__(self, capacity, inputs, seed):
        self.states = np.empty((capacity, inputs), dtype=np.float32)
        self.next_states = np.empty_like(self.states)
        self.actions = np.empty(capacity, dtype=np.int64)
        self.rewards = np.empty(capacity, dtype=np.float32)
        self.terminated = np.empty(capacity, dtype=np.float32)
        self.index, self.size, self.capacity = 0, 0, capacity
        self.rng = np.random.default_rng(seed)

    def add(self, state, action, reward, next_state, terminated):
        i = self.index
        self.states[i], self.actions[i], self.rewards[i] = state, action, reward
        self.next_states[i], self.terminated[i] = next_state, terminated
        self.index = (i+1) % self.capacity
        self.size = min(self.size+1, self.capacity)

    def sample(self, count):
        indices = self.rng.choice(self.size, count, replace=False)
        return tuple(torch.from_numpy(array[indices]) for array in (
            self.states, self.actions, self.rewards, self.next_states, self.terminated))


def bellman_target(rewards, terminated, next_values, gamma):
    """A time-limit truncation still bootstraps; a true goal termination does not."""
    return rewards + gamma * (1-terminated) * next_values


class Agent:
    def __init__(self, inputs, seed=7, config=None):
        self.config = config or Config()
        self.inputs = inputs
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        torch.set_num_threads(1)
        torch.manual_seed(seed)
        self.online = Network(inputs, self.config.hidden)
        self.target = Network(inputs, self.config.hidden)
        self.target.load_state_dict(self.online.state_dict())
        self.target.eval()
        self.target.requires_grad_(False)
        self.optimizer = torch.optim.Adam(self.online.parameters(),
                                          lr=self.config.learning_rate)
        self.replay = Replay(self.config.capacity, inputs, seed)
        self.updates = 0

    def act(self, observation, epsilon=0.):
        if self.rng.random() < epsilon:
            return int(self.rng.integers(4))
        with torch.no_grad():
            return int(self.online(torch.as_tensor(observation)).argmax().item())

    def learn(self):
        if self.replay.size < self.config.batch_size:
            return None
        states, actions, rewards, next_states, terminated = self.replay.sample(
            self.config.batch_size)
        predicted = self.online(states).gather(1, actions[:, None]).squeeze(1)
        with torch.no_grad():
            expected = bellman_target(rewards, terminated,
                                      self.target(next_states).max(1).values,
                                      self.config.gamma)
        loss = nn.functional.smooth_l1_loss(predicted, expected)
        self.optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(self.online.parameters(), 10.)
        self.optimizer.step()
        self.updates += 1
        if self.updates % self.config.target_interval == 0:
            self.target.load_state_dict(self.online.state_dict())
        return float(loss.item())

    def save(self, path, metadata):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix+'.tmp')
        torch.save({'format': 1, 'inputs': self.inputs, 'seed': self.seed,
                    'config': asdict(self.config), 'updates': self.updates,
                    'weights': self.online.state_dict(), 'metadata': metadata}, temporary)
        temporary.replace(path)

    @classmethod
    def load(cls, path):
        checkpoint = torch.load(path, map_location='cpu', weights_only=True)
        if checkpoint.get('format') != 1:
            raise ValueError('Unsupported DQN checkpoint format')
        agent = cls(checkpoint['inputs'], checkpoint['seed'], Config(**checkpoint['config']))
        agent.online.load_state_dict(checkpoint['weights'])
        agent.online.eval()
        agent.target.load_state_dict(checkpoint['weights'])
        agent.updates = checkpoint['updates']
        return agent, checkpoint['metadata']
