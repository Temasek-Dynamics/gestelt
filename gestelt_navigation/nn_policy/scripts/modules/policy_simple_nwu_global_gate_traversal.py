#!/home/yanrui/miniconda3/envs/difflying/bin/python

import torch
import torch.nn as nn
import torch.nn.functional as F

# Track velocity
# Input dim (13) = attitude (4) + qd (6) + target velocity (3)
class TrackVelGate(nn.Module):
    def __init__(self, input_dim=13, output_dim=4):
        super(TrackVelGate, self).__init__()
        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 256)
        self.fc4 = nn.Linear(256, 512)
        self.fc5 = nn.Linear(512, 256)
        self.fc6 = nn.Linear(256, 128)
        self.fc7 = nn.Linear(128, 64)
        self.fc8 = nn.Linear(64, 32)
        
        self.output_layer = nn.Linear(32, output_dim)
        
        
    def forward(self, x):

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))     
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        x = F.relu(self.fc6(x))
        x = F.relu(self.fc7(x))
        x = F.relu(self.fc8(x))
        

        output = self.output_layer(x)
        first_output = torch.sigmoid(output[:, 0:1])  # First node (0 to 1)
        remaining_outputs = torch.tanh(output[:, 1:])  # Remaining nodes (-1 to 1)

        # Combine outputs
        final_output = torch.cat((first_output, remaining_outputs), dim=1)
        # self.output.retain_grad()
        # self.final_output.retain_grad()
        
        return final_output
    
class TrackVelGRU(nn.Module):
    """GRU-based velocity-tracking policy.

    Maintains a hidden state across timesteps so the policy can exploit
    temporal context (e.g. recent error trends, oscillation history).

    Usage in a sim loop:
        h = None                        # reset at episode start
        for step in range(sim_steps):
            a, h = policy(x, h)         # h carries state forward
    """

    def __init__(self, input_dim=13, hidden_size=256, num_layers=1, output_dim=4):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers

        self.encoder = nn.Sequential(
            nn.Linear(input_dim, hidden_size),
            nn.ELU(),
        )
        self.gru = nn.GRU(hidden_size, hidden_size, num_layers, batch_first=True)
        self.decoder = nn.Sequential(
            nn.Linear(hidden_size, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, output_dim),
        )

    def forward(self, x, h=None):
        # x: (batch, input_dim)
        enc = self.encoder(x).unsqueeze(1)     # (batch, 1, hidden_size)
        out, h_next = self.gru(enc, h)          # out: (batch, 1, hidden_size)
        out = self.decoder(out.squeeze(1))      # (batch, output_dim)
        first = torch.sigmoid(out[:, 0:1])
        rest  = torch.tanh(out[:, 1:])
        return torch.cat([first, rest], dim=1), h_next
