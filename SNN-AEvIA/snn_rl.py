import random
from collections import deque
import torch.optim as optim
import rosbag
from dvs_msgs.msg import EventArray
import torch
from spikingjelly.activation_based import layer, neuron, surrogate
from spikingjelly.activation_based import functional
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.tensorboard import SummaryWriter



# Define the Spiking Neural Network (SNN) using SpikingJelly
class DSQN(nn.Module):
    def __init__(self, use_cuda=True):
        super(DSQN, self).__init__()
        self.device = torch.device("cuda")
        self.model_name = 'spiking_dqn'
        self.network = nn.Sequential(
                layer.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
                neuron.LIFNode(),

                # layer.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
                # neuron.LIFNode(),
            
                layer.Flatten(),
                layer.Linear(346 * 260 * 8, 512),
                neuron.NonSpikingLIFNode()
            )

        self.decoder = nn.Linear(512, 2)

        functional.set_step_mode(self.network, step_mode='m')
        if use_cuda:
            functional.set_backend(self.network, backend='torch')
        # Initialize weights to all ones
        # self.initialize_weights()

    def initialize_weights(self):
        # Set all weights and biases to 1
        for m in self.network:
            if isinstance(m, layer.Conv2d) or isinstance(m, nn.Linear):
                nn.init.constant_(m.weight, 1)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

        nn.init.constant_(self.decoder.weight, 0.01)
        if self.decoder.bias is not None:
            nn.init.constant_(self.decoder.bias, 0)

    def forward(self, x):
        # x = x.unsqueeze(0).repeat(self.T, 1, 1, 1, 1)  # [N, C, H, W] -> [T, N, C, H, W]
        
        # fr-mlp
        x = self.network(x)
        # fr = x_seq.mean(0)

        return self.decoder(x)

# Initialize the SNN model
device = torch.device("cuda")
width, height = 346, 260  # Adjust based on your sensor's resolution

def add_spike(spike_tensor, ixy, y, x, polarity):
    # Add spike if within bounds
    if ixy < spike_tensor.shape[0]:
        spike_tensor[ixy, 0, 0, y, x] = polarity
    else:
        print("Timestep Out")



# Define the Experience Replay Buffer
class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state):
        self.buffer.append((state, action, reward, next_state))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state = zip(*batch)
        return (torch.stack(state), torch.tensor(action), torch.tensor(reward), torch.stack(next_state))

    def clear(self):
        # Clear all elements in the buffer
        self.buffer.clear()

    def __len__(self):
        return len(self.buffer)

# Initialize the policy and target networks
policy_net = DSQN().to(device)
target_net = DSQN().to(device)
target_net.load_state_dict(policy_net.state_dict())  # Start with identical networks
target_net.eval()  # Target network in evaluation mode

# Define the Q-learning Agent
class QLearningAgent:
    def __init__(self, policy_net, target_net, buffer_size=100, batch_size=32, gamma=0.99, lr=0.001):
        self.policy_net = policy_net
        self.target_net = target_net
        self.buffer = ReplayBuffer(buffer_size)
        self.batch_size = batch_size
        self.gamma = gamma
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.device = policy_net.device
        self.epsilon = 0.1  # Exploration rate

    def select_action(self, state):
        if random.random() > self.epsilon:
            with torch.no_grad():
                temp = self.policy_net(state).argmax(dim=1).item() 
                functional.reset_net(self.policy_net)
                return temp  # Greedy action
        else:
            return random.choice([0, 1])  # Random action

    def optimize_model(self):
        if len(self.buffer) < self.batch_size:
            return

        # Sample a batch from the replay buffer
        states, actions, rewards, next_states = self.buffer.sample(self.batch_size)
        states = states.permute(1, 0, 2, 3, 4, 5).squeeze(3)
        states = states.to(self.device)
        next_states = next_states.to(self.device)

        # Compute Q(s, a) using policy network
        actions = actions.to(self.device)
        q_values = self.policy_net(states).gather(1, actions.unsqueeze(1)).squeeze()
        functional.reset_net(self.policy_net)

        # Compute the target Q values with the target network
        with torch.no_grad():
            next_states = next_states.permute(1, 0, 2, 3, 4, 5).squeeze(3)
            next_states = next_states.to(self.device)
            next_q_values = self.target_net(next_states).max(1)[0]
            functional.reset_net(self.target_net)
            rewards = rewards.to(self.device)
            target_q_values = rewards + (self.gamma * next_q_values)

        # Calculate the loss
        loss = nn.functional.mse_loss(q_values, target_q_values)

        # Backpropagation and optimization step
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def update_target_net(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())

# Instantiate the agent
agent = QLearningAgent(policy_net, target_net, batch_size=2)

# Modify the DVS event reading function to include Q-learning
def read_dvs_events_with_q_learning(bag_path, num_episodes=100, target_update_freq=10, timestep=0.1):
    bag = rosbag.Bag(bag_path, 'r')
    spike_tensor = torch.zeros((101, 1, 1, height, width), dtype=torch.float32).to(device)
    last_activity = 0
    activity = 0
    last_action = 0

    writer = SummaryWriter('runs/spiking_dqn')  # Specify a directory for TensorBoard logs

    for episode in range(num_episodes):
        current_time, init_time = None, None
        state = spike_tensor.clone()
        done = False
        agent.buffer.clear()
        episode_rewards = 0
        actions = []
        last_action1 = 0
        for topic, msg, t in bag.read_messages(topics=['/davis/left/events']):
            if current_time is None:
                current_time = t.to_sec()
                init_time = current_time

            for event in msg.events:
                event_time = event.ts.to_sec()
                if current_time - init_time > 5:
                    done = True
                    break

                x, y = event.x, event.y
                polarity = 1 if event.polarity else -1
                # print(int((event_time - current_time) * 10000), x , y)
                add_spike(spike_tensor, int((event_time - current_time) * 1000), y, x, polarity) # simulated time resolution 1ms
                activity += 1
                if event_time - current_time >= timestep:
                    action = agent.select_action(state)
                    actions.append(action)
                    # Placeholder for reward function
                    if action == 0:
                        reward = last_activity * 0.8
                    else:
                        reward = activity
                        last_activity = activity
                        if(last_action1 - current_time < 0.005):
                            reward = 0
                    activity = 0

                    if action == 1:
                        last_action1 = current_time
                    # Update the next state
                    next_state = spike_tensor.clone()

                    # Update total reward for the episode
                    episode_rewards += reward

                    agent.buffer.push(state, action, reward, next_state)  # Store transition in replay buffer
                    
                    # Optimize the policy network
                    agent.optimize_model()


                    # Update state and time for the next timestep
                    state = next_state
                    spike_tensor_temp = spike_tensor[11:100].clone()
                    spike_tensor.zero_()
                    spike_tensor[0:89] = spike_tensor_temp
                    current_time += 0.01
                    # print(current_time - init_time)

            if done:
                break

        # Periodic target network update
        if episode % target_update_freq == 0:
            agent.update_target_net()

        writer.add_scalar('Episode/Total_Reward', episode_rewards, episode)
        print(actions)
    bag.close()
    writer.close()  # Close the writer after logging is complete

# Path to your ROS bag file
bag_file_path = '/home/frankxcr/S-AEvIA/Datasets/upenn/indoor_flying1_edited.bag'
read_dvs_events_with_q_learning(bag_file_path)
