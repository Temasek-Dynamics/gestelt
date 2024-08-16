import numpy as np
import matplotlib.pyplot as plt
import os
current_dir = os.path.dirname(os.path.abspath(__file__))

reward_file = os.path.join(current_dir, 'training_data/every_reward49.npy')
reward_data = np.load(reward_file)
# 绘制奖励图
plt.figure(figsize=(10, 5))
plt.plot(reward_data[49], label='Reward')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.title('Reward per Episode')
plt.legend()
plt.grid(True)
plt.show()