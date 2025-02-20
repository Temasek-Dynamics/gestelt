import torch
import torch.nn as nn
import torch.nn.functional as F

# Track velocity
# Input dim (13) = attitude (4) + qd (6) + target velocity (3)
class TrackVel(nn.Module):
    def __init__(self, input_dim=13, output_dim=4):
        super(TrackVel, self).__init__()
        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 32)
        
        self.output_layer = nn.Linear(32, output_dim)
        
        
    def forward(self, att, qd, t_vel):
        x = torch.cat((att, qd, t_vel), dim=1)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))     
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        

        output = self.output_layer(x)
        first_output = torch.sigmoid(output[:, 0:1])  # First node (0 to 1)
        remaining_outputs = torch.tanh(output[:, 1:])  # Remaining nodes (-1 to 1)

        # Combine outputs
        final_output = torch.cat((first_output, remaining_outputs), dim=1)
        # self.output.retain_grad()
        # self.final_output.retain_grad()
        
        return final_output
    
    