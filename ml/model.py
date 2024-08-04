import numpy as np
import random
from collections import deque
import tensorflow as tf
from tensorflow.keras import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam

# Define the environment
class PetEnv:
    def __init__(self):
        self.action_space = [0, 1]  # 0: don't move, 1: move
        self.state_size = 3  # displacement, RSSI, obstacles
        self.state = self.reset()
        
    def reset(self):
        self.state = [0.0, 0.0, 0.0]  # Initial state (displacement, RSSI, obstacles)
        return self.state
    
    def step(self, action):
        displacement = random.uniform(0, 1)  # Simulate displacement
        rssi = random.uniform(-100, 0)  # Simulate RSSI
        obstacles = random.randint(0, 1)  # Simulate obstacles (0: no obstacle, 1: obstacle)
        
        next_state = [displacement, rssi, obstacles]
        reward = displacement if action == 1 else 0  # Reward is the displacement if the action is "move"
        
        done = False  # This example does not include a terminal state
        return next_state, reward, done

    def get_state(self):
        return self.state

# Define the DQN agent
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()
        self.target_model = self._build_model()
        self.update_target_model()
        
    def _build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = self.model.predict(state)
            if done:
                target[0][action] = reward
            else:
                t = self.target_model.predict(next_state)[0]
                target[0][action] = reward + self.gamma * np.amax(t)
            self.model.fit(state, target, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)

# Train the model
env = PetEnv()
state_size = env.state_size
action_size = len(env.action_space)
agent = DQNAgent(state_size, action_size)
done = False
batch_size = 32

for e in range(1000):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    for time in range(500):
        action = agent.act(state)
        next_state, reward, done = env.step(action)
        next_state = np.reshape(next_state, [1, state_size])
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        if done:
            agent.update_target_model()
            print("episode: {}/{}, score: {}, e: {:.2}".format(e, 1000, time, agent.epsilon))
            break
        if len(agent.memory) > batch_size:
            agent.replay(batch_size)
    if e % 10 == 0:
        agent.save("dqn_model.h5")
