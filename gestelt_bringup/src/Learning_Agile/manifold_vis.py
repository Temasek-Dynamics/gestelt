import numpy as np
import matplotlib.pyplot as plt

def generate_random_matrix():
    """生成一个随机的2x2矩阵"""
    return np.random.randn(2, 2)

def closest_rotation_matrix(A):
    """使用 SVD 找到最接近 A 的旋转矩阵"""
    U, _, Vt = np.linalg.svd(A)
    R = U @ Vt  # 计算最接近的旋转矩阵
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1  # 处理反射情况
        R = U @ Vt
    return R

def plot_transformation(A, R):
    """可视化矩阵 A 和最接近的旋转矩阵 R"""
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    
    vectors = np.eye(2)  # 标准基向量 e1=(1,0), e2=(0,1)
    transformed_vectors_A = A @ vectors
    transformed_vectors_R = R @ vectors
    
    for i in range(2):
        ax[0].quiver(0, 0, transformed_vectors_A[0, i], transformed_vectors_A[1, i], 
                     angles='xy', scale_units='xy', scale=1, color=['r', 'b'][i])
        ax[1].quiver(0, 0, transformed_vectors_R[0, i], transformed_vectors_R[1, i], 
                     angles='xy', scale_units='xy', scale=1, color=['r', 'b'][i])
    
    ax[0].set_xlim(-2, 2)
    ax[0].set_ylim(-2, 2)
    ax[0].set_title("Original Matrix A")
    ax[0].grid()
    
    ax[1].set_xlim(-2, 2)
    ax[1].set_ylim(-2, 2)
    ax[1].set_title("Closest Rotation Matrix R")
    ax[1].grid()
    
    plt.show()

def main():
    A = generate_random_matrix()
    R = closest_rotation_matrix(A)
    print("the A 's determinant is ",np.linalg.det(A))
    print("the R 's determinant is ",np.linalg.det(R))    
    print("Original Matrix A:")
    print(A)
    print("Closest Rotation Matrix R:")
    print(R)
    plot_transformation(A, R)

if __name__ == "__main__":
    main()
