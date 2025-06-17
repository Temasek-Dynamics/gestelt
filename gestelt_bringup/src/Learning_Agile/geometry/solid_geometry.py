# solid geometry
# this file is to do some calculation of solid geometry to do the collision detection of quadrotor
# this file consists of several classes
import numpy as np
import casadi as ca
import os

from scipy.spatial.transform import Rotation as R
os.environ["JAX_PLATFORM_NAME"] = "cpu" 
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
import copy



def qr_eigen(A, iterations=10):
    """ 
    perform the QR algorithm for eigenvalue decomposition
    source code from https://gist.github.com/edxmorgan/51bdb566592a3bc0e386db1f8c50104b
    """
    pQ = ca.SX.eye(A.size1())
    X = copy.deepcopy(A)
    for _ in range(iterations):
        Q, R = ca.qr(X)  # QR decomposition in CasADi
        pQ = pQ @ Q # Update eigenvector matrix for next iteration
        X = R @ Q  # Update eigenvalue matrix for next iteration
    return ca.diag(X), pQ    

# def mat2vec(mat, dimb=3):
#     # Same as table.T.reshape(-1, 1)
#     return rearrange(mat, "a b -> (b a)", a=3, b=dimb)


# def vec2mat(vec, dimb=3):
#     return rearrange(vec, "(b a) -> a b", a=3, b=dimb)

# @jax.jit
# def SVD_M_to_SO3(m: np.ndarray) -> np.ndarray:
#     """Maps 3x3 matrices onto SO(3) via symmetric orthogonalization.
#     Source: Google research - https://github.com/google-research/google-research/blob/193eb9d7b643ee5064cb37fd8e6e3ecde78737dc/special_orthogonalization/utils.py#L93-L115
#     """
#     m=m.reshape(3,3)
#     m = jnp.asarray(m)
#     U, _, Vh = jnp.linalg.svd(m, full_matrices=False)
#     det = jnp.linalg.det(jnp.matmul(U, Vh))
#     R= jnp.matmul(jnp.c_[U[:, :-1], U[:, -1] * det], Vh)
#     return R

class SVD():
    def __init__(self,dim=3):
        self.dim=dim
        self.m_flatten = ca.SX.sym('m_flatten',dim*dim)
        self.sigma = ca.SX.sym('sigma',dim,dim)
        


    def SVD_M_to_SO3_ca(self,m_flatten):
        self.m_flatten = m_flatten
        m=ca.reshape(m_flatten,self.dim,self.dim)
        """Maps 3x3 matrices onto SO(3) via symmetric orthogonalization using CasADi with symbolic matrix."""
        # Perform singular value decomposition using CasADi
        mTm = ca.mtimes(m.T, m)
        self.sigma, V = qr_eigen(mTm)
        
        eps = 1e-8
        singular_val=ca.sqrt(self.sigma+eps)
        U=ca.mtimes(ca.mtimes(m,V),ca.diag(1/singular_val))  # U = m * V * diag(1/sigma + eps)
        
        # Calculate the determinant of the product of U and V.T
        det = ca.det(ca.mtimes(U, V.T))
        
        # Adjust the last column of U based on the determinant
        U[:, -1] = U[:, -1] * det
        
        # Return the orthogonalized matrix
        return ca.mtimes(U, V.T)  # Return U * V^T (which is orthogonalized)

    def get_sigma_expr(self,m_flatten):
        """
        this function return the sigma expression
        """
        m=ca.reshape(m_flatten,self.dim,self.dim)
        mTm = ca.mtimes(m.T, m)
        sigma, _ = qr_eigen(mTm)
        return ca.diag(sigma)

    def SVD_M_to_SO3_ca_func(self):
        """
        this function wrap the symbolic SVD_M_to_SO3_ca function
        """
        SVD_func = ca.Function('SVD_func', [self.m_flatten], [self.SVD_M_to_SO3_ca(self.m_flatten)])
        sigma_func = ca.Function('sigma_func', [self.m_flatten], [self.get_sigma_expr(self.m_flatten)])
        
        dsigma_dm = ca.jacobian(self.get_sigma_expr(self.m_flatten),self.m_flatten)
        dsigma_dm_func = ca.Function('dsigma_dm_func', [self.m_flatten], [dsigma_dm])

        return {'SVD_func': SVD_func, 
                'sigma_func': sigma_func,
                'dsigma_dm_func': dsigma_dm_func
                }
        

    
def verify_SVD_ca(des_tra_m):
    ## call the SVD casADi function separately, to verify the SVD result
    svd= SVD()
    func = svd.SVD_M_to_SO3_ca_func()
    SVD_func = func['SVD_func']
    dsigma_dm_func = func['dsigma_dm_func']


    
    verify_tra_R = SVD_func(des_tra_m)
    verify_tra_R=verify_tra_R.toarray()
    verify_tra_R=verify_tra_R.T
    
    sigma = func['sigma_func'](des_tra_m).toarray()
    dsigma_dm = dsigma_dm_func(des_tra_m).toarray()


    return {'verify_tra_R': verify_tra_R, 
            'sigma': sigma, 
            'dsigma_dm': dsigma_dm
            } 


def verify_SVD_PR_ca(des_tra_pitch_m,des_tra_roll_m):
    ## call the SVD casADi function separately, to verify the SVD result
    svd= SVD(dim=2)
    SVD_func=svd.SVD_M_to_SO3_ca_func()
    
    tra_yaw_B_I = np.eye(3)
    tra_pitch_B_I = np.eye(3)
    tra_roll_B_I= np.eye(3)

    tra_pitch_2d,_=SVD_func(des_tra_pitch_m)
    tra_roll_2d,_=SVD_func(des_tra_roll_m)

    tra_pitch_2d = tra_pitch_2d.toarray()
    tra_roll_2d = tra_roll_2d.toarray()

    tra_pitch_B_I[0,0]=tra_pitch_2d[0,0]
    tra_pitch_B_I[0,2]=tra_pitch_2d[0,1]
    tra_pitch_B_I[2,0]=tra_pitch_2d[1,0]
    tra_pitch_B_I[2,2]=tra_pitch_2d[1,1]

    tra_roll_B_I[1,1]=tra_roll_2d[0,0]
    tra_roll_B_I[1,2]=tra_roll_2d[0,1]
    tra_roll_B_I[2,1]=tra_roll_2d[1,0]
    tra_roll_B_I[2,2]=tra_roll_2d[1,1]

    ## follow the zyx order
    verify_tra_R = tra_yaw_B_I @ tra_pitch_B_I @ tra_roll_B_I
    verify_tra_R= tra_roll_B_I @ tra_pitch_B_I @ tra_yaw_B_I
    verify_tra_R=verify_tra_R.T
    # print("sigma=",sigma)
    # print("NN pose det after SVD",np.linalg.det(verify_tra_R))
    return verify_tra_R,_

def pitch_from_gate(gate_point):
    """
    Calculate the pitch angle of the gate from the gate points
    """
    gate_pitch = ca.atan((gate_point[1,2]-gate_point[0,2])/(gate_point[0,0]-gate_point[1,0])) # compute the actual gate pitch ange in real-time 
    return gate_pitch

def recover_euler_from_9d(outputs,deg_unit=False):
    
    R_nn=verify_SVD_ca(outputs[3:12])['verify_tra_R']
    quat_nn=R.from_matrix(R_nn.reshape(3,3))
    euler_nn=quat_nn.as_euler('zyx', degrees=deg_unit)
    
    #backward
    # deuler_dR=dEulerZYX_dR(R_nn)
    # deuler_dm=ca.mtimes(deuler_dR,dR_dm)
    return euler_nn #deuler_dm.toarray()

## return the maginitude of a vector
def magni(vector):
    return np.sqrt(np.dot(np.array(vector),np.array(vector)))



def magni_casadi(vector):
    return ca.norm_2(vector)

## return the unit vector of a vector
def norm(vector):
    return np.array(vector)/magni(np.array(vector))

def dir_cosine_np(q):  # world frame to body frame
    C_B_I = np.array([
        [1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])],
        [2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])],
        [2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)]
    ])
    return C_B_I

def dir_cosine(q): # world frame to body frame
    C_B_I = ca.vertcat(
        ca.horzcat(1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])),
        ca.horzcat(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])),
        ca.horzcat(2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
    )
    return C_B_I

## define a class of a plane (using three points on the plane)
class plane():
    def __init__(self, point1, point2, point3):

        # point1 is the centroid of the gate
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.point3 = np.array(point3)
        self.vec1 = self.point2 - self.point1
        self.vec2 = self.point3 - self.point1
        self.normal = norm(np.cross(self.vec2,self.vec1))
    
    # normal vector of the plane 
    def nor_vec(self, ):
        return self.normal

    # normal vector of one side 
    def n1(self):
        return norm(np.cross(self.vec1,self.normal))

    # normal vector of one side 
    def n2(self):
        return norm(np.cross(self.normal,self.vec2))

    # normal vector of one side 
    def n3(self):
        self.vec3 = self.point3 - self.point2
        return norm(np.cross(self.normal,self.vec3))
    
    ## intersection with another line 
    def interpoint(self, point1, point2):
        dir = norm(np.array(point1)-np.array(point2))
        t = 1/(np.dot(dir,self.normal))*(np.dot(self.normal,np.array(point1)-self.point1))
        point = np.array(point1) - t * dir
        return point

## define a class of a line
class line():
    def __init__(self, point1, point2):
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.dir = norm(self.point1 - self.point2)

    ## return the distance from a point to the line
    def vertical(self, point):
        point3 = np.array(point)
        normal = np.cross(point3 - self.point1, self.dir)
        return magni(normal)

    ## return the distance from a point to the line section
    def distance(self,point):
        a = self.vertical(point)
        b = magni(point-self.point1)
        c = magni(point-self.point2)
        d = magni(self.point1-self.point2)
        if(b>c):
            if((b**2-d**2)>a**2):
                dis = c
            else:
                dis = a
        else:
            if((c**2-d**2)>a**2):
                dis = b
            else:
                dis = a
        return dis

def dEulerZYX_dR(R: np.ndarray) -> np.ndarray:
    """
    计算 Z-Y-X 顺序定义的欧拉角 (alpha, beta, gamma)
    对 3x3 旋转矩阵 R 的偏导数 (雅可比矩阵)。
    
    参数:
    -------
    R : np.ndarray
        形状 (3, 3) 的旋转矩阵

    返回:
    -------
    J : np.ndarray
        形状 (3, 9) 的雅可比矩阵, 
        J[i, j] = d (euler_i) / d (R_flat_j),
        其中 R_flat = [R[0,0], R[0,1], ..., R[2,2]] 按行展开。
        
    注意:
    1. 假设 R 确实是合法的旋转矩阵 (正交、行列式=1)。
    2. 若 R[2,0] 接近 +/-1, 或者 (R[0,0]^2 + R[1,0]^2) 太小,
    会出现欧拉角奇异(万向锁) 或数值不稳定。
    """
    # 确保输入是 (3,3)
    R = np.asarray(R, dtype=float)
    assert R.shape == (3,3), "R 必须为 3x3 矩阵"

    #------------------------------
    # 1. 提取需要的矩阵元素
    #------------------------------
    R00 = R[0, 0]
    R10 = R[1, 0]
    R20 = R[2, 0]
    R21 = R[2, 1]
    R22 = R[2, 2]

    #------------------------------
    # 2. 计算欧拉角(仅供参考, 不一定要实际用到):
    #    beta   = -arcsin(R20)
    #    alpha  = arctan2(R10, R00)
    #    gamma  = arctan2(R21, R22)
    #------------------------------
    # beta = -np.arcsin(R20)
    # alpha = np.arctan2(R10, R00)
    # gamma = np.arctan2(R21, R22)
    # (这里可以不显式算出, 因为最终只需要用它们的偏导数公式)

    #------------------------------
    # 3. 根据公式, 构造偏导数:
    #   α = arctan2(y, x) => 
    #       dα/dx = -y / (x^2 + y^2)
    #       dα/dy =  x / (x^2 + y^2)
    #   β = -arcsin(R20) =>
    #       dβ/d(R20) = - d/d(R20)[arcsin(R20)] = -[1 / sqrt(1 - R20^2)]
    #                 = 1 / sqrt(1 - R20^2)   (多看公式符号)
    #   γ = arctan2(R21, R22) =>
    #       dγ/d(R22) = - R21 / (R22^2 + R21^2)
    #       dγ/d(R21) =   R22 / (R21^2 + R22^2)
    #------------------------------
    # 注意: 其他对 R[i,j] 的偏导均为 0

    # 对 alpha = arctan2(R10, R00):
    denom_alpha = (R00**2 + R10**2)
    # 防止分母过小造成数值问题, 可酌情加个 epsilon
    # denom_alpha = max(denom_alpha, 1e-12)
    
    d_alpha_d_R00 = 0.0
    d_alpha_d_R10 = 0.0
    if denom_alpha > 1e-15:
        d_alpha_d_R00 = -R10 / denom_alpha  # ∂alpha/∂R00
        d_alpha_d_R10 =  R00 / denom_alpha  # ∂alpha/∂R10

    # 对 beta = -arcsin(R20):
    #   => dbeta/d(R20) = 1 / sqrt(1 - R20^2)
    denom_beta = np.sqrt(max(1.0 - R20**2, 1e-15))
    d_beta_d_R20 = 1.0 / denom_beta

    # 对 gamma = arctan2(R21, R22):
    denom_gamma = (R22**2 + R21**2)
    d_gamma_d_R21 = 0.0
    d_gamma_d_R22 = 0.0
    if denom_gamma > 1e-15:
        d_gamma_d_R22 = -R21 / denom_gamma
        d_gamma_d_R21 =  R22 / denom_gamma

    #------------------------------
    # 4. 组装成 3×9 的雅可比矩阵
    #    按 [R00, R01, R02, R10, R11, R12, R20, R21, R22] 顺序展开
    #------------------------------
    J = np.zeros((3, 9), dtype=float)

    # -- alpha (行0) 的非零项:
    #   alpha depends on R[0,0] 与 R[1,0]
    #   => J[0, 0] = d_alpha/d_R00
    #   => J[0, 3] = d_alpha/d_R10
    J[0, 0] = d_alpha_d_R00
    J[0, 3] = d_alpha_d_R10

    # -- beta (行1) 的非零项:
    #   beta depends on R[2,0]
    #   => J[1, 6] = d_beta/d_R20
    J[1, 6] = d_beta_d_R20

    # -- gamma (行2) 的非零项:
    #   gamma depends on R[2,1], R[2,2]
    #   => J[2, 7] = d_gamma/d_R21
    #   => J[2, 8] = d_gamma/d_R22
    J[2, 7] = d_gamma_d_R21
    J[2, 8] = d_gamma_d_R22

    return J