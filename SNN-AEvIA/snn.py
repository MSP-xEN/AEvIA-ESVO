import rosbag
from dvs_msgs.msg import EventArray
import torch
from spikingjelly.activation_based import layer, neuron, surrogate
from spikingjelly.activation_based import functional
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt


# Define the Spiking Neural Network (SNN) using SpikingJelly
class DSQN(nn.Module):
    def __init__(self, use_cuda=True):
        super(DSQN, self).__init__()

        self.model_name = 'spiking_dqn'
        self.network = nn.Sequential(
                layer.Conv2d(1, 16, kernel_size=3, stride=1, padding=1),
                neuron.LIFNode(),

                # layer.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
                # neuron.LIFNode(),
            
                layer.Flatten(),
                layer.Linear(346 * 260 * 16, 512),
                neuron.NonSpikingLIFNode()
            )

        self.decoder = nn.Linear(512, 1)

        functional.set_step_mode(self.network, step_mode='m')
        if use_cuda:
            functional.set_backend(self.network, backend='torch')
        # Initialize weights to all ones
        self.initialize_weights()

    def initialize_weights(self):
        # Set all weights and biases to 1
        for m in self.network:
            if isinstance(m, layer.Conv2d) or isinstance(m, nn.Linear):
                nn.init.constant_(m.weight, 1)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

        # Initialize the decoder layer weights and biases to 1
        # nn.init.constant_(self.decoder.weight, -0.01)
        # if self.decoder.bias is not None:
        #     nn.init.constant_(self.decoder.bias, 1000)

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
snn_model = DSQN().to(device)

def add_spike(spike_tensor, ixy, y, x, polarity):
    # Add spike if within bounds
    if ixy < spike_tensor.shape[0]:
        spike_tensor[ixy, 0, 0, y, x] = polarity
    else:
        # Shift the tensor along the time dimension when ixy > 16
        # shift_tensor(spike_tensor, polarity, y, x)
        print("Timestep Out")

def shift_tensor(spike_tensor, polarity, y, x):
    # Shift tensor along the time dimension by copying elements one step back
    spike_tensor[:-1] = spike_tensor[1:].clone()
    
    # Set the last time slice to zero
    spike_tensor[-1] = 0
    
    # Add the new spike at the shifted location
    spike_tensor[-1, 0, 0, y, x] += polarity

# Define the ROS bag reading function
def read_dvs_events(bag_path, timestep=0.010):
    bag = rosbag.Bag(bag_path, 'r')
    spike_tensor = torch.zeros((101, 1, 1, height, width), dtype=torch.float32).to(device)  # Initial spike tensor for SNN
    
    try:
        topic_name = '/davis/left/events'  # Adjust if topic name is different
        current_time = None
        init_time = 0
        ixy = 0
        timestamps, q_values = [], []  # Lists to store timestamps and Q-values
        activities = []
        activity = 0
        non_zero_sums = [] 
        cnt = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Initialize the timestamp for the first message
            if current_time is None:
                current_time = t.to_sec()
                init_time = current_time

            # Process each event in the EventArray


            for event in msg.events:
                event_time = event.ts.to_sec()
                # print(event_time)
                if current_time - init_time > 25:
                    break

                # Accumulate spikes for the current timestep
                x, y = event.x, event.y
                polarity = 1 if event.polarity else -1
                # print(int((event_time - current_time)*10000))
                add_spike(spike_tensor, int((event_time - current_time) * 10000) , y, x, polarity)
                activity += 1
                # spike_tensor[ixy, 0, 0, y, x] += (polarity)  # Add spike according to polarity
                # print(spike_tensor[ixy][0][0])

                # Accumulate events within each timestep
                if event_time - current_time >= timestep:
                    # Feed the spike_tensor to the SNN
                    non_zero_sum = (spike_tensor != 0).sum()
                    print("Non-Zero Sum", non_zero_sum)
     
                    non_zero_sums.append(non_zero_sum.item())

                    with torch.no_grad():
                        output = snn_model(spike_tensor)  # Generate Q-values
                    
                        functional.reset_net(snn_model)
                    print(current_time - init_time)
                    print("Q-values:", output)
                    
                    timestamps.append(current_time - init_time)  # Save the relative time
                    q_values.append(output.item())  # Save the Q-value
                    activities.append(activity)
                    # print("Activities", activity)
                    activity *= 0.9

                    # Reset spike tensor and move to the next timestep
                    # if ixy > 100:
                    spike_tensor_temp = spike_tensor[11:100].clone()
                    # spike_tensor[90:100] = torch.zeros((10, 1, 1, height, width), dtype=torch.float32)

                    spike_tensor.zero_()
                    spike_tensor[0:89] = spike_tensor_temp

                    current_time = current_time + 0.001
                    # current_time = event_time



    finally:
        # Close the bag after processing
        bag.close()

    # Plotting the Q-values over time
    print("plotting")
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, q_values, label="Q-values")
    plt.plot(timestamps, non_zero_sums, label="Non-Zero Sum")
    # plt.plot(timestamps, activities, label="Acticities")
    plt.xlabel("Time (s)")
    plt.ylabel("Q-value")
    plt.title("DSQN Q-values over Time")
    plt.legend()
    plt.show()

# Path to your ROS bag file
bag_file_path = '/home/frankxcr/S-AEvIA/Datasets/upenn/indoor_flying1_edited.bag'
read_dvs_events(bag_file_path)
