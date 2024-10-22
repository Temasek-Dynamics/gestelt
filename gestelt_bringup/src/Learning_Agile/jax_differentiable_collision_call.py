import jax
import jax.numpy as jnp 
from jax import jit, grad, vmap 
from jax.test_util import check_grads
from jax.scipy.spatial.transform import Rotation as R
import dpax
from dpax.mrp import dcm_from_mrp
from dpax.polytopes import polytope_proximity
import numpy as np
""" 
with this, the drone is represented by a polytope as well.
"""

class Polytope():
    
    def __init__(self):
        self.A = None
        self.b = None
        self.r = None
        self.q = None
    
    def create_rect_prism(self,length, width, height):
        # rectangular prism in Ax<=b form (halfspace form)
        self.A = jnp.array([
            [1,0,0.],
            [0,1,0.],
            [0,0,1.],
            [-1,0,0.],
            [0,-1,0.],
            [0,0,-1.]
        ])

        cs = jnp.array([
            [length/2,0,0.],
            [0,width/2,0.],
            [0.,0,height/2],
            [-length/2,0,0.],
            [0,-width/2,0.],
            [0.,0,-height/2]
        ])
        
        # b[i] = dot(A[i,:], b[i,:]) 
        self.b = jax.vmap(jnp.dot, in_axes = (0,0))(self.A, cs)

     



def JAXDifferentiableCollisionsWrapper(line_centers,
                                    R_gate,
                                    gate_quat,
                                    quad_radius,
                                    quad_half_height,
                                    drone_state):
    
    ## convert from numpy to jnp
    line_centers=jnp.array(line_centers)
    R_gate=jnp.array(R_gate)
    gate_quat=jnp.array(gate_quat)
    quad_radius=jnp.array(quad_radius)
    quad_half_height=jnp.array(quad_half_height)
    drone_state=jnp.array(drone_state)

    line_centers_G=jnp.matmul(line_centers,R_gate.T) 
    width_gap=jnp.abs(line_centers_G[1,0]-line_centers_G[3,0])
    height_gap=jnp.abs(line_centers_G[0,2]-line_centers_G[2,2])
    
    # prism_size=quad_half_height
    # A=jnp.diag([quad_radius,quad_radius,quad_half_height])
    # A_inv=jnp.linalg.inv(A)
    # P=A_inv.T@A_inv

    ## create the drone polytope
    drone_polytope=Polytope()
    drone_polytope.create_rect_prism(quad_radius*2,quad_radius*2,quad_half_height*2)
   
    # test=jnp.matmul(R_gate.T,line_centers[0,])
    # print('line_centers_G:',line_centers_G)

    prism_centers=jnp.zeros([4,3])

    # # left and right prisms centers
    # prism_centers[1,]=line_centers_G[1,]+jnp.array([ quad_radius,0,0])
    # prism_centers[3,]=line_centers_G[3,]+jnp.array([-quad_radius,0,0])

    # # up and down prisms centers
    # prism_centers[0,]=line_centers_G[0,]+jnp.array([0,0, quad_half_height])
    # prism_centers[2,]=line_centers_G[2,]+jnp.array([0,0,-quad_half_height])

    ## left and right prisms centers
    prism_centers = prism_centers.at[1].set(line_centers_G[1] + jnp.array([quad_radius, 0, 0]))
    prism_centers = prism_centers.at[3].set(line_centers_G[3] + jnp.array([-quad_radius, 0, 0]))

    ## up and down prisms centers
    prism_centers = prism_centers.at[0].set(line_centers_G[0] + jnp.array([0, 0, quad_half_height]))
    prism_centers = prism_centers.at[2].set(line_centers_G[2] + jnp.array([0, 0, -quad_half_height]))

    prism_centers_W=jnp.matmul(prism_centers,R_gate)
    
    
    ## create rectangle walls with the desired size
    P_obs = []
    for i in range(4):
        P_obs.append(Polytope())
    
    P_obs[0].create_rect_prism(width_gap, 1.0, quad_half_height*2),
    P_obs[1].create_rect_prism(quad_radius*2, 1.0, height_gap),
    P_obs[2].create_rect_prism(width_gap, 1.0, quad_half_height*2),
    P_obs[3].create_rect_prism(quad_radius*2, 1.0, height_gap)


    ## assign pose of the prisms and ellipsoid
    P_obs[0].r = prism_centers_W[0,]
    P_obs[0].q = gate_quat
    
    P_obs[1].r = prism_centers_W[1,]
    P_obs[1].q = gate_quat

    P_obs[2].r = prism_centers_W[2,]
    P_obs[2].q = gate_quat

    P_obs[3].r = prism_centers_W[3,]
    P_obs[3].q = gate_quat

    drone_polytope.r = drone_state[0:3]
    drone_polytope.q = drone_state[6:10]
    

    # return min scaling α and gradient of α wrt configurations 
    dalpha_dstate_drone=np.zeros(10) # p,v,q
    penalty=0
    for i in range(4): # P_obs[1],P_obs[3] (left and right walls)
        
        # des_alpha comes from the penalty design helper
        if i == 1 or i == 3:
            # for the left and right walls
            alpha_importance=0.1
            des_alpha=1.81825
        else:
            # for the up and down walls
            alpha_importance=1
            des_alpha=1.4325

        # dalpha_i_dstate: drone_p,drone_q,ellipse_p,ellipse_q
        # alpha_i, dalpha_i_dstate=jl.dc.proximity_gradient(Elli_drone,P_obs[i],verbose = False, pdip_tol = 1e-6)

        alpha_i = polytope_proximity(drone_polytope.A, drone_polytope.b, drone_polytope.r, drone_polytope.q,
                                     P_obs[i].A, P_obs[i].b, P_obs[i].r, P_obs[i].q)
        print(alpha_i)
        
        # calculate all the gradients 
        grad_f = jit(grad(polytope_proximity, argnums =(2,3) ))#(0,1,2,3,4,5,6,7)
        
        dalpha_i_dstate=grad_f(drone_polytope.A,drone_polytope.b,drone_polytope.r,drone_polytope.q,
                                  P_obs[i].A,P_obs[i].b,P_obs[i].r,P_obs[i].q)
        
        

        
        scaling_w=100
        penalty+=(scaling_w * alpha_importance * (alpha_i-des_alpha)**2)

        # penalty +=alpha_i*alpha_importance
        dalpha_dstate_drone[0:3] += 2 * scaling_w * alpha_importance * (alpha_i-des_alpha) * np.array(dalpha_i_dstate[0])
        dalpha_dstate_drone[6:10] += 2 * scaling_w * alpha_importance * (alpha_i-des_alpha) * np.array(dalpha_i_dstate[1])
        
    
    return penalty,dalpha_dstate_drone


if __name__ == '__main__':
    #================================================================================================
    # create polytopes 
    P_obs = []
    for i in range(2):
        P_obs.append(Polytope())

    P_obs[0].create_rect_prism(1,2,3)
    P_obs[1].create_rect_prism(2,4,3)

    # position and attitude for each polytope 
    P_obs[0].r = jnp.array([1,3,-2.])
    p1 = jnp.array([.1,.3,.4])
    Q1 = dcm_from_mrp(p1)

    P_obs[1].r = jnp.array([-1,0.1,2.])
    p2 = jnp.array([-.3,.3,-.2])
    Q2 = dcm_from_mrp(p2)

    # calculate proximity (alpha <= 1 means collision) 

    P_obs[0].q=R.from_matrix(Q1).as_quat()
    P_obs[1].q=R.from_matrix(Q2).as_quat()
    P_obs[0].q=jnp.roll(P_obs[0].q,1)
    P_obs[1].q=jnp.roll(P_obs[1].q,1)
    alpha = polytope_proximity(P_obs[0].A,P_obs[0].b,P_obs[0].r,P_obs[0].q,
                            P_obs[1].A,P_obs[1].b,P_obs[1].r,P_obs[1].q)


    print("alpha: ", alpha)

    # calculate all the gradients 
    grad_f = jit(grad(polytope_proximity, argnums =(2,3) ))#(0,1,2,3,4,5,6,7)
    grads = grad_f(P_obs[0].A,P_obs[0].b,P_obs[0].r,P_obs[0].q,
                P_obs[1].A,P_obs[1].b,P_obs[1].r,P_obs[1].q)

    d1=np.array(grads[0])
    print(d1)
    # check gradients 
    check_grads(polytope_proximity,  (P_obs[0].A,P_obs[0].b,P_obs[0].r,P_obs[0].q,
                                        P_obs[1].A,P_obs[1].b,P_obs[1].r,P_obs[1].q), order=1, atol = 2e-1)