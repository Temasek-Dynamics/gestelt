import numpy as np
from numpy.core.fromnumeric import shape
from scipy import sparse
import scipy.linalg
import math
import scipy.io as scio
import osqp
import matplotlib.pyplot as plt
import time

class MinSnap:
    def __init__(self) -> None:
        pass

    def minsnap_trajectory_single_axis(self, way_points, time_set, n_order, n_obj, v_i, a_i, v_e, a_e):
        # Set init and end point
        p_i = way_points[0]
        p_e = way_points[-1]
         
        # Set poly num and coeffient num
        n_poly = len(way_points) - 1
        n_coef = n_order  + 1

        # Compute QP cost function matrix
        q_i_cost = []
        for i in range(n_poly):
            q_i_cost.append(self.compute_q(n_order+1, n_obj, time_set[i], time_set[i+1]))
        q_cost = scipy.linalg.block_diag(*q_i_cost)
        p_cost = np.zeros(q_cost.shape[0])

        # Set equality constraints
        A_eq = np.zeros((4*n_poly + 2, n_coef*n_poly))
        b_eq = np.zeros((4*n_poly + 2, 1))
        # Init and End constraints: 6
        A_eq[0:3,0:n_coef] = [self.compute_t_vec(time_set[0], n_order, 0), \
                                                    self.compute_t_vec(time_set[0], n_order, 1), \
                                                    self.compute_t_vec(time_set[0], n_order, 2)]
        A_eq[3:6,n_coef*(n_poly-1):n_coef*n_poly] = \
                                            [self.compute_t_vec(time_set[-1], n_order, 0), \
                                            self.compute_t_vec(time_set[-1], n_order, 1), \
                                            self.compute_t_vec(time_set[-1], n_order, 2)]
        b_eq[0:6] = np.transpose(np.array([[p_i, v_i, a_i, p_e, v_e, a_e] ]))
        # Points constraints: n_poly - 1
        n_eq = 6
        for i in range(0, n_poly-1):
            A_eq[n_eq, n_coef*(i+1):n_coef*(i+2)] = self.compute_t_vec(time_set[i+1], n_order, 0)
            b_eq[n_eq] = np.array([[way_points[i+1]]])
            n_eq = n_eq+1
        # Continous constraints: (n_poly - 1)*3
        for i in range(0, n_poly-1):
            t_vec_p = self.compute_t_vec(time_set[i+1], n_order, 0)
            t_vec_v = self.compute_t_vec(time_set[i+1], n_order, 1)
            t_vec_a = self.compute_t_vec(time_set[i+1], n_order, 2)
            A_eq[n_eq,n_coef*i:n_coef*(i+1)]=t_vec_p
            A_eq[n_eq,n_coef*(i+1):n_coef*(i+2)]=-t_vec_p
            n_eq=n_eq+1
            A_eq[n_eq,n_coef*i:n_coef*(i+1)]=t_vec_v
            A_eq[n_eq,n_coef*(i+1):n_coef*(i+2)]=-t_vec_v
            n_eq=n_eq+1
            A_eq[n_eq,n_coef*i:n_coef*(i+1)]=t_vec_a
            A_eq[n_eq,n_coef*(i+1):n_coef*(i+2)]=-t_vec_a
            n_eq=n_eq+1
        
        # Set inequality constraints
        A_ieq = np.zeros((0, n_coef*n_poly))
        b_ieq = np.zeros((0, 1))

        # Convert equality constraints to inequality constraints
        A_eq_ieq = np.vstack((A_eq, -1 * A_eq))
        b_eq_ieq = np.vstack((b_eq, -1 * b_eq))
        # A_ieq = np.vstack((A_ieq, A_eq_ieq))
        # b_ieq = np.vstack((b_ieq, b_eq_ieq))
        A_ieq = A_eq_ieq
        b_ieq = b_eq_ieq

        # Solve qp(sqp) problem
        m = osqp.OSQP()
        m.setup(P=sparse.csc_matrix(q_cost), q=None, l=None, A=sparse.csc_matrix(A_ieq), u=b_ieq, verbose=False)
        results = m.solve()
        x = results.x
        return x

    def compute_q(self, n, r , t_i, t_e):
        q = np.zeros((n,n))
        for i in range(r, n):
            for j in range(i, n):
                k1 = i - r 
                k2 = j - r 
                k = k1 + k2 +1
                q[i, j] = (math.factorial(i)/math.factorial(k1)) * (math.factorial(j)/math.factorial(k2)) * (pow(t_e,k)-pow(t_i,k)) / k
                q[j,i] = q[i,j]
        return q

    def compute_t_vec(self, t, n_order, k):
        t_vector = np.zeros(n_order+1)
        for i in range(k, n_order+1):
            t_vector[i] =  (math.factorial(i)/math.factorial(i-k)) * pow(t, i-k)
        return t_vector

    def time_arrange(self, way_points, T):
        dist_vec = way_points[:, 1:] - way_points[:, :-1]
        dist = []
        for i in range(dist_vec.shape[1]):
            dist_i = 0
            for j in range(dist_vec.shape[0]):
                dist_i += dist_vec[j][i] **2
            dist_i = dist_i **0.5
            dist.append(dist_i)
        k = T/sum(dist)
        time_set = [0]
        time_i = np.array(dist)*k
        for i in range(1,len(time_i)):
            time_i[i] = time_i[i] + time_i[i-1]
        time_set.extend(time_i)
        return time_set

def minimum_snap_traj(way_points, total_time, n_order, n_obj):
    start_t = time.time()
    traj = MinSnap()
    # Reshape waypoints matrix
    way_points_n = np.array(way_points)[:,0:3]
    way_points_n = np.transpose(way_points_n)
    # Total time (s)
    T = total_time
    # n_order : Poly order
    # n_obj : Object order: 1-minimum vel 2-mimimum acc 3-minimum jerk 4-minimum snap
    # Poly num
    n_poly = len(way_points) - 1
    # Initial and end state
    v_i = [0,0,0,0]
    a_i = [0,0,0,0]
    v_e = [0,0,0,0]
    a_e = [0,0,0,0]
    # Arrange time roughly
    time_set = traj.time_arrange(way_points_n,T)
    # Get Matrix
    p_x = traj.minsnap_trajectory_single_axis(way_points_n[0,:], time_set, n_order, n_obj, v_i[0], a_i[0], v_e[0], a_e[0])
    p_y = traj.minsnap_trajectory_single_axis(way_points_n[1,:], time_set, n_order, n_obj, v_i[1], a_i[1], v_e[1], a_e[1])
    p_z = traj.minsnap_trajectory_single_axis(way_points_n[2,:], time_set, n_order, n_obj, v_i[2], a_i[2], v_e[2], a_e[2])
    Matrix_x = np.transpose(p_x.reshape(n_poly,n_order+1))
    Matrix_y = np.transpose(p_y.reshape(n_poly,n_order+1))
    Matrix_z = np.transpose(p_z.reshape(n_poly,n_order+1))
    end_t = time.time()
    # print(end_t-start_t)
    return time_set, Matrix_x, Matrix_y, Matrix_z

def minimum_snap_traj_p2p(way_points, time_set, n_order, n_obj, v_i, a_i, v_e, a_e):
    start_t = time.time()
    traj = MinSnap()
    way_points_n = np.array(way_points)[:,0:3]
    way_points_n = np.transpose(way_points_n)
    # Poly num == 1
    n_poly = len(way_points) - 1
    p_x = traj.minsnap_trajectory_single_axis(way_points_n[0,:], time_set, n_order, n_obj, v_i[0], a_i[0], v_e[0], a_e[0])
    p_y = traj.minsnap_trajectory_single_axis(way_points_n[1,:], time_set, n_order, n_obj, v_i[1], a_i[1], v_e[1], a_e[1])
    p_z = traj.minsnap_trajectory_single_axis(way_points_n[2,:], time_set, n_order, n_obj, v_i[2], a_i[2], v_e[2], a_e[2])
    Matrix_x = np.transpose(p_x.reshape(n_poly,n_order+1))
    Matrix_y = np.transpose(p_y.reshape(n_poly,n_order+1))
    Matrix_z = np.transpose(p_z.reshape(n_poly,n_order+1))
    end_t = time.time()
    # print(end_t-start_t)
    return Matrix_x, Matrix_y, Matrix_z


def get_traj(Matrix_x, Matrix_y, Matrix_z, time_set, sample_rate):
    # Init
    p = [[],[],[]]
    v = [[],[],[]]
    a = [[],[],[]]
    n = Matrix_x.shape[0] - 1
    # Get sample time list
    sample_list = []
    for t_sample in range(math.floor(time_set[-1]*sample_rate+1)):
        sample_list.append(t_sample/sample_rate)
    # Get p v a list
    for i in range(1,len(sample_list)):
        # Get id
        t = sample_list[i]
        id = 0
        for i in range(len(time_set)-1):
            if  t >= time_set[i] and t < time_set[i+1]:
                id = i
                break
            else:
                pass
        # Get t vector
        t_array_p = []
        t_array_v = []
        t_array_a = []
        for i in range(n+1):
            t_array_p.append(pow(t,i))
            t_array_v.append((i*pow(t,i-1)))
            t_array_a.append((i*(i-1)*pow(t,i-2)))
        # Get desire states
        p[0].append(np.dot(np.array(t_array_p),Matrix_x[ : , id]))
        p[1].append(np.dot(np.array(t_array_p),Matrix_y[ : , id]))
        p[2].append(np.dot(np.array(t_array_p),Matrix_z[ : , id]))
        v[0].append(np.dot(np.array(t_array_v),Matrix_x[ : , id]))
        v[1].append(np.dot(np.array(t_array_v),Matrix_y[ : , id]))
        v[2].append(np.dot(np.array(t_array_v),Matrix_z[ : , id]))
        a[0].append(np.dot(np.array(t_array_a),Matrix_x[ : , id]))
        a[1].append(np.dot(np.array(t_array_a),Matrix_y[ : , id]))
        a[2].append(np.dot(np.array(t_array_a),Matrix_z[ : , id]))
    return p, v, a, sample_list




