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
        
        
    def forward(self, x):

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
    

class CNNImageEncoder(nn.Module):
    def __init__(self, image_res=(128, 128), latent_dims=64):
        super(CNNImageEncoder, self).__init__()
        self.image_res = image_res
        self.latent_dims = latent_dims
        
        # Feature extraction with stride convolutions
        self.features = nn.Sequential(
            # Block 1: [1, 135, 240] -> [32, 68, 120]
            nn.Conv2d(1, 32, kernel_size=5, stride=2, padding=2),
            nn.ELU(),
            
            # Block 2: [32, 68, 120] -> [64, 34, 60]
            nn.Conv2d(32, 64, kernel_size=5, stride=2, padding=2),
            nn.ELU(),
            
            # Block 3: [64, 34, 60] -> [128, 17, 30]
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ELU(),
            
            # Block 4: [128, 17, 30] -> [256, 9, 15]
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ELU()
        )
        
        # Final projection to latent dims
        self.projection = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),  # [256, 1, 1]
            # nn.AdaptiveMaxPool2d(1),
            nn.Conv2d(256, latent_dims, kernel_size=1),  # [64, 1, 1]
            nn.Flatten()  # [64]
        )

        # 添加sigmoid层
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        # Reshape input if needed
        if len(x.shape) == 3:  # [batch, H, W]
            x = x.unsqueeze(1)  # [batch, 1, H, W]
        
        # Forward pass
        features = self.features(x)
        latent = self.projection(features)
        return latent

class PolicyNetwork(nn.Module):
    def __init__(self, input_dim, output_dim, img_size):
        super(PolicyNetwork, self).__init__()
        
        self.image_encoder = CNNImageEncoder(image_res=img_size, latent_dims=64)
        
        # Original network with expanded input dimension (21 + 64 = 85)
        self.network = nn.Sequential(
            nn.Linear(input_dim + 64, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, output_dim)
        )
    
    def forward(self, x, depth_imgs=None):
        if depth_imgs is not None:
            # Encode depth images using CNN
            depth_latent = self.image_encoder(depth_imgs)
            x = torch.cat([x, depth_latent], dim=1)
        else:
            depth_latent = x.new_zeros(x.shape[0], 64)
            x = torch.cat([x, depth_latent], dim=1)

        output = self.network(x)
        first_output = torch.sigmoid(output[:, 0:1])  # First node (0 to 1)
        remaining_outputs = torch.tanh(output[:, 1:])  # Remaining nodes (-1 to 1)

        # Combine outputs
        final_output = torch.cat((first_output, remaining_outputs), dim=1)

        return final_output
    
