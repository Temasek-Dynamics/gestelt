# solid geometry
# this file is to do some calculation of solid geometry to do the collision detection of quadrotor
# this file consists of several classes
import numpy as np
# from differentiable_collision_wrapper import *
import casadi as ca
import torch

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

## define the narrow window which is also the obstacle for the quadrotor
class obstacle():
    def __init__(self, point1, point2, point3, point4):
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.point3 = np.array(point3)
        self.point4 = np.array(point4)

        #define the centroid of obstacle
        self.centroid = np.array([(point1[0]+point2[0]+point3[0]+point4[0])/4,\
            (point1[1]+point2[1]+point3[1]+point4[1])/4,(point1[2]+point2[2]+point3[2]+point4[2])/4])
        
        #define the cross section
        self.plane1 = plane(self.centroid,point1,point2)
        self.plane2 = plane(self.centroid,point2,point3)
        self.plane3 = plane(self.centroid,point3,point4)
        self.plane4 = plane(self.centroid,point4,point1)

        #define the bound
        self.line1 = line(point1, point2)
        self.line2 = line(point2, point3)
        self.line3 = line(point3, point4)
        self.line4 = line(point4, point1)

    def collis_det(self, vert_traj, horizon):
        """
        detect collision for each corner of the quadrotor
        """
        ## define the state whether find corresponding plane
        collision = 0
        self.co = 0

        PASS_GATE_PLANE = False
   
        ## judge whether the first plane is the traversal plane
        # find two points of traverse
        d_min = 0.2
        for t in range(horizon):
            if(np.dot(self.plane1.nor_vec(),vert_traj[t]-self.centroid)<0):
                PASS_GATE_PLANE = True
                # intersect is s_iw
                intersect = self.plane1.interpoint(vert_traj[t],vert_traj[t-1])
                
                
                # judge whether they belong to plane1 and calculate the distance
                if(np.dot(self.plane1.n1(),intersect-self.centroid)>0 and np.dot(self.plane1.n2(),intersect-self.centroid)>0):
                    
                    if(np.dot(self.point1-intersect,self.plane1.n3())>0):

                        # m is the d_i
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line4.distance(intersect),self.line1.distance(intersect),self.line2.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                

       
                # judge whether the intersection belongs to plane2 and calculate the distance  
                if(np.inner(self.plane2.n1(),intersect-self.centroid)>0 and np.inner(self.plane2.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point2-intersect,self.plane2.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line1.distance(intersect),self.line2.distance(intersect),self.line3.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                    

                # judge whether the intersection belongs to plane3 and calculate the distance
                if(np.inner(self.plane3.n1(),intersect-self.centroid)>0 and np.inner(self.plane3.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point3-intersect,self.plane3.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line2.distance(intersect),self.line3.distance(intersect),self.line4.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                    

                # judge whether the intersection belongs to plane4 and calculate the distance
                if(np.inner(self.plane4.n1(),intersect-self.centroid)>0 and np.inner(self.plane4.n2(),intersect-self.centroid)>0):
                    if(np.dot(self.point4-intersect,self.plane4.n3())>0):
                        m = min(self.line1.vertical(intersect),self.line2.vertical(intersect),\
                            self.line3.vertical(intersect),self.line4.vertical(intersect))
                        collision = - max(0,d_min-m)**2
                        self.co = 1
                    else:
                        m = min(self.line3.distance(intersect),self.line4.distance(intersect),self.line1.distance(intersect))
                        collision =   - 2*d_min*m - d_min**2
                break
        
        if not PASS_GATE_PLANE:
            # means the solve failed, avoid this situation
            collision = 0   
                  
        return collision
    ############################################################################################################
    ###----------------------------- drone as a ellipsoid, symbolic reward ---------------------------------####
    ############################################################################################################
    # def collis_det_ellipsoid


    
   
    

        
    # Symbolic reward calculation    
    def reward_calc_sym(self,
                        quad_sym,
                        quad_radius,
                        quad_height,
                        alpha,
                        beta,
                        Q_tra,
                        w_goal):
        
        
        gate_corner_1 = (ca.vertcat(ca.SX.sym('gate_corner_1'+'_x'),\
                                    ca.SX.sym('gate_corner_1'+'_y'),\
                                    ca.SX.sym('gate_corner_1'+'_z')))
        
        gate_corner_2 = (ca.vertcat(ca.SX.sym('gate_corner_2'+'_x'),\
                                    ca.SX.sym('gate_corner_2'+'_y'),\
                                    ca.SX.sym('gate_corner_2'+'_z')))
        
        gate_line=gate_corner_2-gate_corner_1
        gate_line_dir = gate_line/ca.norm_2(gate_line)

        v1=quad_sym.quad_state[0:3]-gate_corner_1

        # the distance vector from the gate line to the drone center
        dist_v_W = ca.cross(v1,gate_line_dir)


        R_tra = dir_cosine(quad_sym.quad_state[6:10]) # from world frame to body frame
        

        A = ca.diag(ca.vertcat(quad_radius, quad_radius, quad_height))
        
        # E=ca.mtimes(R_tra,ca.mtimes(D,R_tra.T))
        # E_inv = ca.inv(E)
        # the vector from the gate point to the drone center
        # v_W = gate_point-quad_sym.quad_state[0:3] # under world frame
       
        delta_d = ca.mtimes(ca.inv(A),ca.mtimes(R_tra,dist_v_W))

        # the magnitude of the vector
        delta_d_mag = ca.norm_2(delta_d)
        
        # gate check points within the ellipsoid
        # if the gate check point is within the ellipsoid,
        # the delta_d_mag is less than 1 and the collision score is larger than 1

        single_colli_score = - alpha * ca.exp(beta*(1-delta_d_mag))

        # traverse score
        # we hope the drone could traverse the gate in the middle
        # so each point delta_d_mag should be larger than 1 but not too large
        single_trav_score = - Q_tra * (delta_d_mag) ** 2
        

        ## goal score
        single_goal_score = - w_goal * ca.norm_2(quad_sym.goal_r_I - quad_sym.quad_state[0:3])


        ## define the forward function
        self.single_colli_score_fun = ca.Function('single_colli_score_fn',
                                                    [quad_sym.quad_state,\
                                                    gate_corner_1,\
                                                    gate_corner_2],[dist_v_W,single_colli_score])
        
        self.single_trav_score_fun = ca.Function('single_trav_score_fn',
                                                    [quad_sym.quad_state,\
                                                    gate_corner_1,\
                                                    gate_corner_2],[single_trav_score])
        
        self.single_goal_score_fun = ca.Function('single_goal_score_fn',
                                                    [quad_sym.quad_state,\
                                                    quad_sym.goal_r_I],[single_goal_score])
        
        ## define the jacobian
        self.d_colli_d_state = ca.jacobian(single_colli_score,quad_sym.quad_state)
     
        self.d_trav_d_state = ca.jacobian(single_trav_score,quad_sym.quad_state)

        self.d_goal_d_state = ca.jacobian(single_goal_score,quad_sym.quad_state)

        ## define the gradient function
        self.d_colli_d_state_fun = ca.Function('d_colli_d_state',
                                             [quad_sym.quad_state,\
                                                    gate_corner_1,\
                                                    gate_corner_2],[self.d_colli_d_state])
                                                     

        self.d_trav_d_state_fun = ca.Function('d_trav_d_state',
                                            [quad_sym.quad_state,\
                                                    gate_corner_1,\
                                                    gate_corner_2],[self.d_trav_d_state])

        
        self.d_goal_d_state_fun = ca.Function('d_goal_d_state',
                                            [quad_sym.quad_state,\
                                            quad_sym.goal_r_I],[self.d_goal_d_state])
    
    def reward_calc_value(self, 
                            state_traj, 
                            gate_corners,
                            goal_pos,
                            vert_traj, 
                            horizon):
        collision_score = 0
        traverse_score = 0
        
        
        t_tra_seq_list=[]

        ## find the traversal sequence
        for t in range(horizon):
            if(np.dot(self.plane1.nor_vec(),vert_traj[t]-self.centroid)<0):
                t_tra_seq_list.append(t-3)
            if len(t_tra_seq_list)==6:
                break
        
        ## init gate check points: four corners and twelve middle points
        gate_check_points = np.zeros([16,3])
        gate_lines = np.zeros([4,3])
        drdstate_traj = np.zeros((horizon+1,state_traj.shape[1]))
        

        for node_tra in t_tra_seq_list:
            ## traverse and collision score
            ONLY_MIDDLE_POINTS = False

            d_colli_d_state =  ca.DM([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            d_trav_d_state = ca.DM([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            d_goal_d_state = ca.DM([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            
            ## for each gate check point
            for i in range(4):

 
                current_corners = gate_corners[i:i+2,:]
                if i == 3:
                    current_corners = np.vstack((gate_corners[3,:],gate_corners[0,:]))
                    

                ## check collision and get the collision and traverse score
                dist_v_W,single_colli_score = self.single_colli_score_fun(state_traj[node_tra,:],\
                                                                                current_corners[0,:],\
                                                                                current_corners[1,:])
                
                single_trav_score = self.single_trav_score_fun(state_traj[node_tra,:],\
                                                                current_corners[0,:],\
                                                                current_corners[1,:])
                
                collision_score += single_colli_score
                traverse_score += single_trav_score
                
                ## get the gradient of the scores
                d_colli_d_state += self.d_colli_d_state_fun(state_traj[node_tra,:],\
                                                            current_corners[0,:],\
                                                            current_corners[1,:])
                
                
                d_trav_d_state += self.d_trav_d_state_fun(state_traj[node_tra,:],\
                                                            current_corners[0,:],\
                                                            current_corners[1,:])
                
            
            ## for each node
            drdstate_traj[node_tra,:] = d_colli_d_state + d_trav_d_state
            
        
        # goal score
        goal_score = 0
        d_goal_d_state = ca.DM([[0, 0, 0]])

        # for last four nodes
        for i in range(-1,-5,-1):
            goal_score += self.single_goal_score_fun(state_traj[i,:],goal_pos)
            
            d_goal_d_state = self.d_goal_d_state_fun(state_traj[i,:],goal_pos)
            drdstate_traj[i,:] = d_goal_d_state


        collision_score.toarray()
        traverse_score.toarray() 
        goal_score.toarray()

        # drdstate_traj = drdstate_traj.toarray()

        reward = collision_score+traverse_score + goal_score 
        
        return float(reward), drdstate_traj, gate_check_points

    def reward_calc_differentiable_collision(self, 
                                            quad_radius,
                                            quad_height,
                                            state_traj, 
                                            gate_corners,
                                            gate_quat,
                                            vert_traj, 
                                            goal_pos,
                                            horizon):   

        collision_score = 0
        traverse_score = 0
        
        
        t_tra_seq_list=[]

        ## find the traversal sequence
        for t in range(horizon):
            if(np.dot(self.plane1.nor_vec(),vert_traj[t]-self.centroid)<0):
                t_tra_seq_list.append(t-1)
            if len(t_tra_seq_list)==2:
                break
        
        ## init gate check points: four corners and twelve middle points
        gate_check_points = np.zeros([16,3])
        gate_lines = np.zeros([4,3])
        drdstate_traj = np.zeros((horizon+1,state_traj.shape[1]))
        
        R_gate=dir_cosine_np(gate_quat) # world frame to gate frame
        reward_traj = 0
        for node_tra in t_tra_seq_list:
        
            line_centers = np.zeros([4,3])
            ## for each gate check point
            for i in range(4):

                if i == 3:
                    line_centers[i,:] =(gate_corners[9:12]+gate_corners[0:3])/2

                else:
                    line_centers[i,:]= (gate_corners[3*i:3*i+3]+gate_corners[3*i+3:3*i+6])/2

                
                
            reward_single,dalpha_dstate_drone=DifferentiableCollisionsWrapper(line_centers,
                                            R_gate,
                                            gate_quat,
                                            quad_radius,
                                            quad_height,
                                            state_traj[node_tra,:])
            
            reward_traj += reward_single
            drdstate_traj[node_tra,:] = dalpha_dstate_drone
        

         # goal score
        goal_score = 0
        
        # for last four nodes
        for i in range(-1,-5,-1): 
            goal_score += -0.5 * np.dot(state_traj[i,:3]-goal_pos,state_traj[i,:3]-goal_pos)
            drdstate_traj[i,:3] = -0.5 * 2 * (state_traj[i,:3]-goal_pos)
        
        reward_traj += goal_score
        return reward_traj, drdstate_traj, gate_check_points
############################################################################################################
###-------------------------------- support torch auto differentiation ---------------------------------####
############################################################################################################
def norm_torch(tensor):
    return tensor/torch.norm(tensor)

def magni_torch(tensor):
    return torch.norm(tensor)

# Define a class for a plane using three points on the plane
class Plane_torch:
    def __init__(self, point1, point2, point3):
        # point1 is the centroid of the gate
        # self.point1 = torch.tensor(point1,  requires_grad=True)
        # self.point2 = torch.tensor(point2,  requires_grad=True)
        # self.point3 = torch.tensor(point3,  requires_grad=True)
        self.point1=point1
        self.point2=point2
        self.point3=point3
        self.vec1 = self.point2 - self.point1
        self.vec2 = self.point3 - self.point1
        self.normal = norm_torch(torch.cross(self.vec2, self.vec1))

    # Normal vector of the plane
    def nor_vec_torch(self):
        return self.normal

    # Normal vector of one side
    def n1_torch(self):
        return norm_torch(torch.cross(self.vec1, self.normal))

    # Normal vector of one side
    def n2_torch(self):
        return norm_torch(torch.cross(self.normal, self.vec2))

    # Normal vector of one side
    def n3_torch(self):
        self.vec3 = self.point3 - self.point2
        return norm_torch(torch.cross(self.normal, self.vec3))
    
    # Intersection with another line
    def interpoint_torch(self, point1, point2):

        dir = norm_torch(point1 - point2)
        t = torch.dot(self.normal, point1 - self.point1) / torch.dot(dir, self.normal)
        point = point1 - t * dir
        return point

# Define a class for a line
class Line_torch:
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
        self.dir = norm_torch(self.point1 - self.point2)

    # Return the distance from a point to the line
    def vertical_torch(self, point):
        point3 = point
        normal = torch.cross(point3 - self.point1, self.dir)
        return magni_torch(normal)

    # Return the distance from a point to the line segment
    def distance_torch(self, point):
        point = point
        a = self.vertical_torch(point)
        b = magni_torch(point - self.point1)
        c = magni_torch(point - self.point2)
        d = magni_torch(self.point1 - self.point2)
        if b > c:
            if (b**2 - d**2) > a**2:
                dis = c
            else:
                dis = a
        else:
            if (c**2 - d**2) > a**2:
                dis = b
            else:
                dis = a
        return dis

# Define the narrow window which is also the obstacle for the quadrotor
class obstacle_torch:
    def __init__(self, point1, point2, point3, point4):
        self.point1 = torch.tensor(point1,  requires_grad=True)
        self.point2 = torch.tensor(point2,  requires_grad=True)
        self.point3 = torch.tensor(point3,  requires_grad=True)
        self.point4 = torch.tensor(point4,  requires_grad=True)

        # Define the centroid of the obstacle
        self.centroid = torch.tensor([(self.point1[0] + self.point2[0] + self.point3[0] + self.point4[0]) / 4,
                                      (self.point1[1] + self.point2[1] + self.point3[1] + self.point4[1]) / 4,
                                      (self.point1[2] + self.point2[2] + self.point3[2] + self.point4[2]) / 4],
                                       requires_grad=True)

        # Define the cross sections
        self.plane1_torch = Plane_torch(self.centroid, self.point1, self.point2)
        self.plane2_torch = Plane_torch(self.centroid, self.point2, self.point3)
        self.plane3_torch = Plane_torch(self.centroid, self.point3, self.point4)
        self.plane4_torch = Plane_torch(self.centroid, self.point4, self.point1)

        # Define the bounds
        self.line1_torch = Line_torch(self.point1, self.point2)
        self.line2_torch = Line_torch(self.point2, self.point3)
        self.line3_torch = Line_torch(self.point3, self.point4)
        self.line4_torch = Line_torch(self.point4, self.point1)

    def collis_det_torch(self, vert_traj, horizon):
        """
        Detect collision for each corner of the quadrotor
        """
        # Define the state whether the corresponding plane is found
        collision = torch.tensor(0.0, requires_grad=True)
        self.co = 0
        PASS_GATE_PLANE = False
        d_min = 0.2

        for t in range(horizon):
            if torch.dot(self.plane1_torch.nor_vec_torch(), vert_traj[t] - self.centroid) < 0:
                PASS_GATE_PLANE = True
                intersect = self.plane1_torch.interpoint_torch(vert_traj[t], vert_traj[t-1])

                # Judge whether they belong to plane1 and calculate the distance
                if torch.dot(self.plane1_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane1_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point1 - intersect, self.plane1_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_41= torch.min(self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect))
                        m= torch.min(min_41, self.line2_torch.distance_torch(intersect))
                        # m = torch.min(self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane2 and calculate the distance
                if torch.dot(self.plane2_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane2_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point2 - intersect, self.plane2_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_12 = torch.min(self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect))
                        m = torch.min(min_12, self.line3_torch.distance_torch(intersect))
                        # m = torch.min(self.line1_torch.distance_torch(intersect), self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane3 and calculate the distance
                if torch.dot(self.plane3_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane3_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point3 - intersect, self.plane3_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_23 = torch.min(self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect))
                        m = torch.min(min_23, self.line4_torch.distance_torch(intersect))
                        # m = torch.min(self.line2_torch.distance_torch(intersect), self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2

                # Judge whether the intersection belongs to plane4 and calculate the distance
                if torch.dot(self.plane4_torch.n1_torch(), intersect - self.centroid) > 0 and torch.dot(self.plane4_torch.n2_torch(), intersect - self.centroid) > 0:
                    if torch.dot(self.point4 - intersect, self.plane4_torch.n3_torch()) > 0:
                        # m = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect),
                        #               self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        min_12 = torch.min(self.line1_torch.vertical_torch(intersect), self.line2_torch.vertical_torch(intersect))
                        min_34 = torch.min(self.line3_torch.vertical_torch(intersect), self.line4_torch.vertical_torch(intersect))
                        m = torch.min(min_12, min_34)
                        collision = - torch.max(torch.tensor(0.0), d_min - m)**2
                        self.co = 1
                    else:
                        min_34 = torch.min(self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect))
                        m = torch.min(min_34, self.line1_torch.distance_torch(intersect))
                        # m = torch.min(self.line3_torch.distance_torch(intersect), self.line4_torch.distance_torch(intersect), self.line1_torch.distance_torch(intersect))
                        collision = - 2 * d_min * m - d_min**2
                break

        if not PASS_GATE_PLANE:
            # Means the solve failed, avoid this situation
            collision = torch.tensor(0.0, requires_grad=True)

        return collision