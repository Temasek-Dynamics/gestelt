import numpy as np
import matplotlib.pyplot as plt
import os
current_dir = os.path.dirname(os.path.abspath(__file__))

reward_data = os.path.join(current_dir, 'Every_Reward0.npy')

# 绘制奖励图
plt.figure(figsize=(10, 5))
plt.plot(reward_data, label='Reward')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.title('Reward per Episode')
plt.legend()
plt.grid(True)
plt.show()