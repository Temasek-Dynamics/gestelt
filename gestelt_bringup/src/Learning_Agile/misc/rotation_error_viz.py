"""GPT coding"""
import numpy as np
import matplotlib.pyplot as plt

# 定义旋转矩阵的函数 (绕 z 轴的旋转矩阵)
def rotation_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0,              0,            1]])

# 定义 trace(I - R_d^T R) 的函数，R_d 为期望的旋转矩阵，R 为真实旋转矩阵
def trace_I_minus_RdT_R(Rd, theta):
    R = rotation_matrix_z(theta)
    return 2*np.trace(np.eye(3) - np.dot(Rd.T, R))


# Chordal distance
def chordal_distance(Rd, theta):
    R = rotation_matrix_z(theta)
    c_dis = 0
    # for i in range(3):
    #     c_dis += np.dot(Rd[:,i]-R[:,i],Rd[:,i]-R[:,i])
    c_dis = np.dot(np.linalg.norm(Rd-R),np.linalg.norm(Rd-R))
    return c_dis

# 定义期望旋转矩阵 R_d (例如设为单位矩阵，即无旋转)
Rd = np.eye(3)

# 创建 theta 的范围，取值从 -π 到 π
theta_values = np.linspace(-np.pi, np.pi, 500)

# 计算 trace(I - R_d^T R) 的值
trace_values = [trace_I_minus_RdT_R(Rd, theta) for theta in theta_values]
c_dis= [chordal_distance(Rd, theta) for theta in theta_values]


# 画出 trace(I - R_d^T R) 随 theta 变化的图像
plt.figure(figsize=(8, 6))
plt.plot(theta_values, c_dis, label=r'chordal_distance', color='r', linewidth=2)
plt.plot(theta_values, trace_values, label=r'trace($I - R_d^T R$)', color='b', linewidth=2)
plt.title(r'trace($I - R_d^T R$) as a Function of Theta', fontsize=16)
plt.xlabel(r'$\theta$ (radians)', fontsize=14)
plt.ylabel(r'trace($I - R_d^T R$)', fontsize=14)
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.xlim(-np.pi, np.pi)
plt.legend(fontsize=12)
plt.show()