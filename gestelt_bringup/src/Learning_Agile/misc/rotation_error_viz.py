import numpy as np
import matplotlib.pyplot as plt

# 定义计算 trace(R) 的函数
def trace_R(theta):
    return 1 + 2 * np.cos(theta)

# 创建 theta 的范围，取值从 -π 到 π
theta_values_limited = np.linspace(-np.pi, np.pi, 500)

# 计算对应的 trace(R) 值
trace_values_limited = trace_R(theta_values_limited)

# 画出 trace(R) 随 theta 变化的图像（范围限制在 -π 到 π）
plt.figure(figsize=(8, 6))
plt.plot(theta_values_limited, trace_values_limited, label=r'trace($R$) = 1 + 2cos($\theta$)', color='b', linewidth=2)
plt.title('Trace(R) as a Function of Theta (Limited Range)', fontsize=16)
plt.xlabel(r'$\theta$ (radians)', fontsize=14)
plt.ylabel(r'trace($R$)', fontsize=14)
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.xlim(-np.pi, np.pi)
plt.legend(fontsize=12)
plt.show()