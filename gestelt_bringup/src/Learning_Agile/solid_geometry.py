# solid geometry
# this file is to do some calculation of solid geometry to do the collision detection of quadrotor
# this file consists of several classes
import numpy as np
import casadi as ca
import jax
import jax.numpy as jnp
from einops import rearrange
import copy
from casadi import Opti


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

def mat2vec(mat, dimb=3):
    # Same as table.T.reshape(-1, 1)
    return rearrange(mat, "a b -> (b a)", a=3, b=dimb)


def vec2mat(vec, dimb=3):
    return rearrange(vec, "(b a) -> a b", a=3, b=dimb)

@jax.jit
def SVD_M_to_SO3(m: np.ndarray) -> np.ndarray:
    """Maps 3x3 matrices onto SO(3) via symmetric orthogonalization.
    Source: Google research - https://github.com/google-research/google-research/blob/193eb9d7b643ee5064cb37fd8e6e3ecde78737dc/special_orthogonalization/utils.py#L93-L115
    """
    m=m.reshape(3,3)
    m = jnp.asarray(m)
    U, _, Vh = jnp.linalg.svd(m, full_matrices=False)
    det = jnp.linalg.det(jnp.matmul(U, Vh))
    R= jnp.matmul(jnp.c_[U[:, :-1], U[:, -1] * det], Vh)
    return R

class SVD():
    def __init__(self):
        self.m_flatten = ca.SX.sym('m_flatten',9)
        self.sigma = ca.SX.sym('sigma',3,3)

    def SVD_M_to_SO3_casadi(self,m_flatten):
        self.m_flatten = m_flatten
        m=ca.reshape(m_flatten,3,3)
        """Maps 3x3 matrices onto SO(3) via symmetric orthogonalization using CasADi with symbolic matrix."""
        # Perform singular value decomposition using CasADi
        mTm = ca.mtimes(m.T, m)
        self.sigma, V = qr_eigen(mTm)
        

        singular_val=ca.sqrt(self.sigma)
        U=ca.mtimes(ca.mtimes(m,V),ca.diag(1/singular_val))
        
        # Calculate the determinant of the product of U and V.T
        det = ca.det(ca.mtimes(U, V.T))
        
        # Adjust the last column of U based on the determinant
        U[:, -1] = U[:, -1] * det
        
        # Return the orthogonalized matrix
        return ca.mtimes(U, V.T)  # Return U * V^T (which is orthogonalized)

    def SVD_M_to_SO3_casadi_func(self):
        """
        this function wrap the symbolic SVD_M_to_SO3_casadi function
        """
        SVD_func = ca.Function('SVD_func', [self.m_flatten], [self.SVD_M_to_SO3_casadi(self.m_flatten),self.sigma])
        
        return SVD_func
    
def verify_SVD_casadi(des_tra_m):
        ## call the SVD casADi function separately, to verify the SVD result
        svd= SVD()
        SVD_func=svd.SVD_M_to_SO3_casadi_func()
        verify_tra_R,sigma=SVD_func(des_tra_m)
        verify_tra_R=verify_tra_R.toarray()
        verify_tra_R=verify_tra_R.T
        # print("sigma=",sigma)
        # print("NN pose det after SVD",np.linalg.det(verify_tra_R))
        return verify_tra_R

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

