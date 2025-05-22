## this file is a package for policy search for quadrotor
import numpy as np

import scipy
from quad_OC import OCSys,LQR
from geometry.solid_geometry import pitch_from_gate
from quad_model import QuadrotorCTBRCtl, QuadrotorSRTCtl,QuadrotorWrenchCtl,QuadrotorAugmentedSRTCtl, toQuaternion,Gate
from visualization.python_sim_vis import get_quad_vert_pos,plot_position,plot_angularrate,plot_thrust
from config import train_cfg
from geometry.solid_geometry import magni, pitch_from_gate, verify_SVD_ca
from config import mission_cfg, train_cfg
from scipy.spatial.transform import Rotation as R
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']  

def get_obs(
        last_gate_points = None,    
        i = None,
        input_size= None,
        drone_state = None,
        final_point = None,
        gate_t_i= None
    ):
    """
    get both immediate and past observation from the environment
    
    Args:
        last_gate_points: the past observation
        i: the current time step
        input_size: the size of the input
        drone_state: the current drone state
        final_point: the final point of the drone
        gate_t_i: the current gate state
        
        
    Returns:
        obs: the observation for the NN input
    """
    ##==calculate the gate RM
    gate_pitch = pitch_from_gate(gate_t_i.gate_point)
    rot=R.from_euler('zyx',[0,gate_pitch,0])
    
    immed_obs=np.zeros(input_size)
    immed_obs[0:3]=drone_state[0:3]/mission_cfg['pos_norm_factor']
    immed_obs[3:6]=drone_state[3:6]/mission_cfg['vel_norm_factor']
    immed_obs[6:10]=drone_state[6:10] # quaternion
    immed_obs[10:13]=final_point/mission_cfg['pos_norm_factor']
    
    ## gate points
    relative_gate_points = gate_t_i.gate_point-drone_state[0:3]
    immed_obs[13:25]=relative_gate_points.flatten()/mission_cfg['pos_norm_factor'] # gate points
    immed_obs[25:37]=last_gate_points.flatten()/mission_cfg['pos_norm_factor'] # last gate points

    ## update the last gate points
    last_gate_points = relative_gate_points


    return immed_obs, gate_pitch, last_gate_points

def manual_set_z_forward(cur_pos:np.array=None,
                         gate_center:np.array=None,
                         gate_ori_9d:np.array=None):
    # manually set the traversal time and pose
    out=np.zeros(output_size)
    out[0:3]=gate_center-cur_pos # gate center - drone position
    out[3:12]=gate_ori_9d # manual set 9D vector (is rotation matrix directly)
    out[-8:-5]=mission_cfg['learning_agile']['wrp']
    out[-5:-2]=mission_cfg['learning_agile']['wrt']
    out[-2]=mission_cfg['learning_agile']['wqt']
    out[-1]=mission_cfg['learning_agile']['traverse_weight_span']

    # out[0:3]=np.array([1.6307370e-01, -1.1879361e+00,  2.3646435e-01])
    # out[3:12]= np.array([-1.6918890e-02,  1.9469048e-01,  3.6082739e-01, \
    #                       3.6829162e-02,  6.1634600e-01, -1.8477699e-01,\
    #                       5.4425687e-01, -7.5466178e-02,  2.5479184e-02])
    # out[-8:-5]=np.array([1.0600287e+02,  1.7199979e+02,  1.3278973e+02])
    # out[-5:-2]=np.array([1.5756940e+02, 1.7169960e+02,  1.6217084e+02])  
    # out[-2]=np.array([3.5511166e+01])
    # out[-1]=np.array([4.9806870e+01])
    
    ### SVD through CasADi
    verify_tra_R,_=verify_SVD_ca(out[3:12])

    gate_pitch=mission_cfg['mission']['gate_ori_euler'][1]
    return gate_pitch,out,verify_tra_R

class PlanFwdBwdWrapper():
    """
    this class is responsible for wrap the single MPC prediction traj for training
    Wrapper the MPC single prediction forward and backward process
    """

    def __init__(self,config:dict,options: dict):
    
        ######################################################
        #######------------ UAV PARAM----------------#########
        ######################################################
        ## definition 
        if options["CLOSE_LOOP_TRAINING"]:
            self.wing_len = config['drone']['wing_len_train'] 
            self.uav_height = config['drone']['height_train']
        else:
            self.wing_len = config['drone']['wing_len'] 
            self.uav_height = config['drone']['height']
        # --------------------------- create model1 ----------------------------------------
        if config['ctl_mode']==0:
            self.uav = QuadrotorCTBRCtl(options,config)
        elif config['ctl_mode']==1:
            self.uav = QuadrotorSRTCtl(options,config)
        elif config['ctl_mode']==2:
            self.uav = QuadrotorWrenchCtl(options,config)
        elif config['ctl_mode']==3:
            self.uav = QuadrotorAugmentedSRTCtl(options,config)
      
        self.options = options
        self.config = config

        # c is the torque constant, l is the diagonal length of the quadrotor
        self.uav.quad_dyn.initDyn(Jx=config['drone']['inertia'][0],
                                  Jy=config['drone']['inertia'][1],
                                  Jz=config['drone']['inertia'][2],
                                  mass=config['drone']['mass'],
                                  l=config['drone']['diagonal_axis_dist'],
                                  c=config['drone']['moment_constant']) # NUSWARM quadrotor
        self.uav.init_model()
        self.uav.set_bound_value(config=config)
        
        ## for the safe PDP backward
        self.uav.init_constraint()
        ######################################################
        #######------------ MPC PARAM----------------#########
        ######################################################

        # MPC prediction step, and prediction horizon
        self.horizon = config['learning_agile']['horizon']
        self.dt=config['learning_agile']['dt']
        # initialize the cost function with symbolic variables
        self.max_tra_w = config['learning_agile']['max_tra_w']

    
        # --------------------------- create PDP object1 ----------------------------------------
        # create a pdp object
        self.uavoc = OCSys(config)
       
        
     
        # set symbolic functions for the MPC solver
        self.uavoc.setStateVariable(self.uav.X,state_lb=self.uav.state_lb,state_ub=self.uav.state_ub, cur_r_I=self.uav.quad_dyn.cur_r_I)
                                  
      
        self.uavoc.setAuxvarVariable()
        self.uavoc.setControlVariable(self.uav.U,
                                       control_lb = self.uav.control_lb,
                                       control_ub = self.uav.control_ub) # thrust-to-weight = 4:1
       
        self.uavoc.setDyn(self.uav.f,self.dt)
        
        # P=self.LQR_as_terminal_cost()
        # diag_P = np.diag(P)   
        # wrt: ,gate traverse position cost
        # wqt: gate traverse attitude cost
        # wthrust: input thrust cost
        # wwt: input angular velocity cost
        # wrf: final position cost
        # wvf: final velocity cost
        # wqf: final attitude cost
        # wwf: final angular velocity cost
      
        ## initialize the cost function
        self.uav.cost_base.init_weight(#wrt=config['learning_agile']['wrt'],
                           #wqt=config['learning_agile']['wqt'],
                           wthrust=config['learning_agile']['wthrust'],
                           wdthrust=config['learning_agile']['wdthrust'],
                           w_tra_throttle=config['learning_agile']['w_tra_throttle'],
                           wm=config['learning_agile']['wm'],
                           wwt=config['learning_agile']['wwt'],
                           wwt_z=config['learning_agile']['wwt_z'], 
                             
                        #    wrp=config['learning_agile']['wrp'],
                           wvp=config['learning_agile']['wvp'],
                           wqp=config['learning_agile']['wqp'],

                           wrf=config['learning_agile']['wrf'],
                           wvf=config['learning_agile']['wvf'],
                           wqf=config['learning_agile']['wqf'],
                           max_tra_w=config['learning_agile']['max_tra_w'],
                        #    traverse_weight_span=config['learning_agile']['traverse_weight_span']
                           ) 
        self.uav.init_cost()
        self.uav.init_traCost()

        ## set the symbolic cost function to the solver
        self.uavoc.setTraCost(self.uav.tra_cost,
                               self.uav.cost_base.trav_auxvar,
                               self.uav.cost_base.t_node,
                               self.uav.cost_base.des_t_tra
                              )
        
        



        self.uavoc.setPathCost(self.uav.path_cost,
                               goal_state=self.uav.goal_state)
        self.uavoc.setFinalCost(self.uav.final_cost,goal_state=self.uav.goal_state)
    
        ## for the safe PDP backward
        self.uavoc.setInequCstr(self.uav.path_inequ_cstr,self.uav.final_inequ_cstr)
        self.uavoc.convert2BarrierOC(gamma=config['learning_agile']['barrier_gamma'])
        
        # initialize the mpc solver
        # self.uavoc.ocSolverInit(horizon=self.horizon,dt=self.dt)
        self.uavoc.AcadosModelInit()
        self.uavoc.AcadosOcSolverInit(
            horizon=self.horizon,
            dt=self.dt,
            SQP_RTI_OPTION=options['SQP_RTI_OPTION'],
            USE_PREV_SOLVER=options['USE_PREV_SOLVER']
        )
       
        ###################################################################
        ###------------ PDP auxiliary control system----------------#######
        ###################################################################
        # define the auxilary control system symbolic functions
    
       
        # if options['MPC_BACKWARD']:
            
        #     if self.options['PDP_GRADIENT']:
        self.uavoc.diffPMP()
        self.lqr_solver = LQR()
        
        self.d_st_traj_d_z=np.zeros((self.horizon+1,1,train_cfg['model']['output_size']))
        self.d_input_traj_d_z=np.zeros((self.horizon,1,train_cfg['model']['output_size']))
        
       
        
    def init_state_and_mission(
        self,
        goal_pos, 
        goal_ori,
        ini_r,
        ini_v_I, 
        ini_q
        ):  
        # goal
        self.goal_pos = goal_pos
        goal_ori = np.array(goal_ori)
        goal_vel = np.array([0, 0, 0])
        goal_w = np.array([0.0, 0.0, 0.0])
        if self.config['ctl_mode']==0 or self.config['ctl_mode']==2:
            self.hover_u = np.array([self.config['drone']['mass']*9.81,0.0,0.0,0.0])
        elif self.config['ctl_mode']==1: 
            self.hover_u = np.array([self.config['drone']['mass']*9.81/4]*4)
   
            
        # initial
        if type(ini_r) is not list:
            ini_r = ini_r.tolist()
        self.ini_r = ini_r
        self.ini_v_I = ini_v_I 
        self.ini_q = ini_q
        self.ini_w =  [0.0, 0.0, 0.0]
        self.init_SRT=[self.config['drone']['mass']*9.81/4]*4
        goal_SRT=self.init_SRT
        if self.config['ctl_mode']==0:
            self.ini_state = np.array(self.ini_r + self.ini_v_I + self.ini_q)
            self.goal_state_value=np.concatenate((goal_pos,goal_vel,goal_ori))
        elif self.config['ctl_mode']==1 or self.config['ctl_mode']==2:
            self.ini_state = np.array(self.ini_r + self.ini_v_I + self.ini_q + self.ini_w)
            self.goal_state_value=np.concatenate((goal_pos,goal_vel,goal_ori,goal_w))
        elif self.config['ctl_mode']==3:
            self.ini_state = np.array(self.ini_r + self.ini_v_I + self.ini_q + self.ini_w + self.init_SRT)
            self.goal_state_value=np.concatenate((goal_pos,goal_vel,goal_ori,goal_w,goal_SRT))

    def update_goal_pos(self,goal_pos):
        self.goal_pos = goal_pos
    

    # initialize the narrow window
    def init_obstacle(self,gate_t_i):

        gate_pitch = pitch_from_gate(gate_t_i.gate_point)
        
        self.gate_corners = gate_t_i.gate_point[:,:].reshape(12)
        self.gate_quat = toQuaternion(gate_pitch,[0,1,0]) # world frame to the body frame?
        self.point1 = self.gate_corners[0:3]
        self.point2 = self.gate_corners[3:6]
        self.point3 = self.gate_corners[6:9]
        self.point4 = self.gate_corners[9:12]     
        from penalty_cal import Obstacle   
        self.obstacle = Obstacle(self.point1,self.point2,self.point3,self.point4,self.wing_len,self.uav_height)



    def get_penalty(self,state_traj,real_state_i=None,success_rate=None):
        """
        for the predicted trajectory check
        """

        self.vert_traj = get_quad_vert_pos(wing_len = self.wing_len, state_traj = state_traj)
        penalty,self.d_L_d_st_traj,_=self.obstacle.penalty_cal_diff_collision(
            self.config,
            self.options,
            state_traj=state_traj,
            gate_corners=self.gate_corners,
            gate_quat=self.gate_quat,
            vert_traj=self.vert_traj[:,0:3],
            goal_pos=self.goal_pos,
            real_state_i=real_state_i,
            success_rate=success_rate
            )
            
        self.d_L_d_st_traj = self.d_L_d_st_traj.reshape(self.horizon+1,1,self.uavoc.n_state)
        return [penalty,self.d_L_d_st_traj]
    
    def get_penalty_demo(self, demo_state_traj, state_traj):
        """
        For learning from demonstration, the penalty MSE between the demo trajectory and the current trajectory
        """
        # compute the MSE loss between the demo trajectory and the current trajectory
        mse_loss= np.mean((demo_state_traj - state_traj)**2)
        # compute the derivative of the loss with respect to the current trajectory
        d_L_d_st_traj = 2 * (state_traj - demo_state_traj) / len(state_traj)
        d_L_d_st_traj = d_L_d_st_traj.reshape(self.horizon+1,1,self.uavoc.n_state)
        return [mse_loss,d_L_d_st_traj]
    
    def final_traj_eval(self,state_traj,gate_points_list):
        """
        for the finial trajectory check.
        generate the gate obstacle when the real drone trajectory is close to the gate (real drone trajectory y=0)
        since the gate could move
        """
        try:
            real_t_tra = np.where(np.abs(state_traj[:,1])<0.1)[0][0]
        except IndexError: 
            FAILED = True
        else:
            gate_real_t_tra= Gate(gate_points_list[int(real_t_tra)])
            self.init_obstacle(gate_real_t_tra)
            
            self.vert_traj = get_quad_vert_pos(
                wing_len = self.wing_len, 
                state_traj = state_traj
            )
            
            _,_,FAILED=self.obstacle.penalty_cal_diff_collision(
                self.config,
                options=self.options,
                state_traj=state_traj,
                gate_corners=self.gate_corners,
                gate_quat=self.gate_quat,
                vert_traj=self.vert_traj[:,0:3],
                goal_pos=self.goal_pos
            )
        
        
        return FAILED
    
    
    def PDP_grad(self, trav_auxvar_value):
        """
        calculate the analytical gradient of the penalty with respect to the traverse hyperparameters
        
        Args:
            trav_auxvar_value (np.array): the traverse hyperparameters value
        
        """
        
        ###################################################################
        ###----- Set mpc external variables VALUE to diffPMP--------#######
        ###################################################################
        self.horizon = self.sol1['control_traj_opt'].shape[0]
    
        ## using LQR solver to solve the auxilary control system to get the analytical gradient
        # set values to the auxilary control system symbolic functions 
        aux_sys = self.uavoc.getAuxSys(state_traj_opt=self.sol1['state_traj_opt'],
                                        control_traj_opt=self.sol1['control_traj_opt'],
                                        costate_traj_opt=self.sol1['costate_traj_opt'],
                                        goal_state_value=self.goal_state_value,
                                        des_t_tra_value = self.des_t_tra,
                                        auxvar_value=trav_auxvar_value)
        
        # set values to the LQR solver
        self.lqr_solver.setDyn(dynF=aux_sys['dynF'], dynG=aux_sys['dynG'], dynE=aux_sys['dynE'])
        self.lqr_solver.setPathCost(Hxx=aux_sys['Hxx'], Huu=aux_sys['Huu'], Hxu=aux_sys['Hxu'], Hux=aux_sys['Hux'],
                                Hxe=aux_sys['Hxe'], Hue=aux_sys['Hue'])
        self.lqr_solver.setFinalCost(hxx=aux_sys['hxx'], hxe=aux_sys['hxe'])


        ## solve the auxilary control system and get the analytical gradient
        aux_sol=self.lqr_solver.lqrSolver(np.zeros((self.uavoc.n_state, self.uavoc.n_trav_auxvar)), self.horizon)
        

        ## take solution of the auxiliary control system
        # which is the dtrajectory/dtraverse_auxvar 
        self.d_st_traj_d_z = np.array(aux_sol['state_traj_opt']) #(n_node,n_state,n_trav_auxvar)
        self.d_input_traj_d_z = np.array(aux_sol['control_traj_opt'])
    
    def LQR_as_terminal_cost(self):
        """
        Generate the MPC terminal cost with only the goal cost and the linearized dynamics at the hovering state,
        by using the infinite horizon LQR solver

        Returns:
            np.array: P matrix
        """
        ## linearized dynamics and the K matrix
        self.uavoc.diffContinuDyn()
        hover_state=np.zeros(self.uavoc.n_state)
        hover_state[6]=1
        hover_u = np.zeros(self.uavoc.n_control)
        hover_u[0] = self.config['drone']['mass']*9.81
        A = self.uavoc.dfx_cont_fn(hover_state, hover_u).full()
        B = self.uavoc.dfu_cont_fn(hover_state, hover_u).full()

        # A_disc=np.exp(A*self.dt)
        # B_disc=np.matmul(np.linalg.inv(A),A_disc-np.eye(A.shape[0]))@B
        
        ## check the controllability 
        rank=check_controllability(A,B)
        path_diag_vals=[30]*3+[self.config['learning_agile']['wvp']]*3+[self.config['learning_agile']['wqp']]*4
        control_diag_vals=[self.config['learning_agile']['wthrust']]+[self.config['learning_agile']['wwt']]*2+[self.config['learning_agile']['wwt_z']]
        Q=np.diag(path_diag_vals)
        R=np.diag(control_diag_vals)

        ## Calculate the optimal LQR P based on A,B,Q,R, with the discrete-time algebraic Riccati equation
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
        np.set_printoptions(suppress=True, precision=2)
        print('terminal cost weight:',P)
        return P
    

    def LQR_as_init_guess(self,trav_auxvar_value,cur_state,cur_u):
        """ 
        Generating an initial trajectory with only the goal cost and the linearized dynamics at the hovering state,
        using the Finite Horizon LQR solver, where the terminal cost is the MPC terminal cost
        """
        path_diag_vals=[trav_auxvar_value[-4]]*3+[self.config['learning_agile']['wvp']]*3+[self.config['learning_agile']['wqp']]*4
        ter_diag_vals=[self.config['learning_agile']['wrf']]*3+[self.config['learning_agile']['wvf']]*3+[self.config['learning_agile']['wqf']]*4
        control_diag_vals=[self.config['learning_agile']['wthrust']]+[self.config['learning_agile']['wwt']]*2+[self.config['learning_agile']['wwt_z']]
        Q=np.diag(path_diag_vals)
        R=np.diag(control_diag_vals)
        Qf=np.diag(ter_diag_vals)
        ## linearized dynamics and the K matrix
        A = self.uavoc.dfx_fn(cur_state, cur_u).full()
        B = self.uavoc.dfu_fn(cur_state, cur_u).full()
        
        # set values to the LQR solver
        self.lqr_solver.setDyn(dynF=A, dynG=B)
        self.lqr_solver.setPathCost(Hxx=Q, Huu=R, Hxu=np.zeros([Q.shape[0],R.shape[1]]), \
                                    Hux=np.zeros([R.shape[0],Q.shape[1]]), Hxe=-np.matmul(Q,self.goal_state_value).reshape(-1,1), \
                                    Hue=-np.matmul(R,cur_u).reshape(-1,1))
        self.lqr_solver.setFinalCost(hxx=Qf,hxe=-np.matmul(Qf,self.goal_state_value).reshape(-1,1))


        ## solve the auxilary control system and get the analytical gradient
        init_guess_sol=self.lqr_solver.lqrSolver(cur_state, self.horizon)
        # plot_position(np.array(init_guess_sol['state_traj_opt']),name='lqr_initial guess',SHOW=True)
        # plot_angularrate(np.array(init_guess_sol['control_traj_opt'])[:,1:],SHOW=True)
        # plot_thrust(np.array(init_guess_sol['control_traj_opt']),SHOW=True)
        
        return init_guess_sol
    
    # def minsnap_as_init_guess(self):
    #     """ generating an initial trajectory by using the minimum snap trajectory generation method
    #     """
    #     way_points = np.array([[self.ini_r[0],self.ini_r[1],self.ini_r[2],0.0],
    #                            [self.goal_pos[0],self.goal_pos[1],self.goal_pos[2],0.0]])
    #     final_time = np.linalg.norm(self.goal_pos-self.ini_r)/self.config['pretrain_param']['desired_average_vel']
    #     time_set = np.array([0,final_time])
    #     n_order = 5
    #     n_obj = 3
    #     sample_rate = self.horizon
    #     v_i = [0,0,0,0]
    #     a_i = [0,0,0,0]
    #     v_e = [0,0,0,0]
    #     a_e = [0,0,0,0]
    #     Matrix_x, Matrix_y, Matrix_z = minimum_snap_traj_p2p(way_points, time_set, n_order, n_obj, v_i, a_i, v_e, a_e)
    #     p, v, a, t_list= get_traj(Matrix_x, Matrix_y, Matrix_z, time_set, sample_rate)
    #     R = differential_flatness_transform(np.array(p), np.array(v), np.array(a))
    #     q = R_to_quat(R)

    #     # assemble the trajectory
    #     state_traj = np.zeros((len(t_list),self.uavoc.n_state))
    #     state_traj[:,0:3] = p
    #     state_traj[:,3:6] = v
    #     state_traj[:,6:10] = q
    #     return state_traj
    ## given initial state, control command, high-level parameters, obtain the first control command of the quadrotor
    
    def conser_mpc_as_init_guess(
        self,
        cur_state,
        trav_auxvar_value,
        des_t_tra=None,
        last_u=None,
        first_iter=False
    ):
        """use the mannual set traverse hyperparameters to generate the initial guess for the MPC solver

        Args:
            trav_auxvar_value (_type_): _description_
        """
        self.des_t_tra = des_t_tra
        self.sol1,NO_SOLUTION_FLAG = self.uavoc.AcadosOcSolver(
            cur_state=cur_state,
            goal_state_value=self.goal_state_value,
            dt=self.dt,
            trav_auxvar_value=trav_auxvar_value,
            des_t_tra=self.des_t_tra,
            last_u=last_u
        )
        
        return self.sol1,NO_SOLUTION_FLAG
        
        
    def mpc_update(
        self, 
        cur_state,
        trav_auxvar_value,
        des_t_tra=None,
        last_u=None,
        first_iter=False,
        init_guess=None,
    ):
        """ 
        collect goal, curren state, and traverse auxvar value, then ask the MPC to solve the optimal control problem
        Args:
            cur_state (_type_): _description_
            trav_auxvar_value (_type_): _description_
            last_u (_type_): _description_

        Returns:
            _type_: _description_
        """
        # if self.config['manual_init_guess'] and first_iter:
        #     # init_guess = self.LQR_as_init_guess(trav_auxvar_value,cur_state=cur_state,cur_u=last_u)
        #     init_guess = self.minsnap_as_init_guess()
        ## MPC requires both the goal state adn the traverse hyperparameters
        self.des_t_tra = des_t_tra
        
        # self.sol1 = self.uavoc.ocSolver(cur_state_control=cur_state_control,t_tra=t)
        self.sol1,NO_SOLUTION_FLAG = self.uavoc.AcadosOcSolver(
            cur_state=cur_state,
            goal_state_value=self.goal_state_value,
            dt=self.dt,
            trav_auxvar_value=trav_auxvar_value,
            des_t_tra=self.des_t_tra,
            last_u=last_u,
            init_guess=init_guess
        )
        # print('goal_pos:',self.goal_pos)
        # return control, pos_vel_cmd
        return self.sol1,NO_SOLUTION_FLAG

def check_controllability(A,B):
    """
    check the controllability of the system
    """
    n = A.shape[0]
    m = B.shape[1]
    c_matrix = np.zeros((n,n*m))
    for i in range(n):
        c_matrix[:,i*m:(i+1)*m] = np.linalg.matrix_power(A,i)@B
    rank = np.linalg.matrix_rank(c_matrix)
    return rank





# ## sample the perturbation (only for random perturbations)
# def sample(deviation):
#     act = np.random.normal(0,deviation,size=6)
#     return act


#############################################################
##-----------------ellipse collision check-----------------##
#############################################################
# initialize the drone ellipse
# self.obstacle.penalty_calc_sym(self.uav,
#                                 quad_height=self.uav_height/2,
#                                 quad_radius=self.wing_len/2,
#                                 alpha=5,
#                                 beta=10,
#                                 Q_tra=5,
#                                 w_goal=0.1)    

#                                 # alpha=5,
#                                 # beta=10,
#                                 # Q_tra=1,
#                                 # safe_margin=0.0,
#                                 # w_goal=0.1)    


# penalty,self.d_L_d_st_traj,gate_check_points=self.obstacle.penalty_calc_value(state_traj,
#                                     self.gate_corners,
#                                     goal_pos=self.goal_pos,
#                                     vert_traj=self.vert_traj[:,0:3],
#                                     horizon=self.horizon)


    # def optimize(self, t):
    #     tra_pos = self.obstacle.centroid
    #     tra_posx = self.obstacle.centroid[0]
    #     tra_posy = self.obstacle.centroid[1]
    #     tra_posz = self.obstacle.centroid[2]
    #     tra_a = 0
    #     tra_b = 0
    #     tra_c = 0
    #     tra_ang = np.array([tra_a,tra_b,tra_c])
    #     ## fixed perturbation to calculate the gradient
    #     for k in range(200):
    #         j = self.MPC_and_R (tra_pos,tra_ang,t)
    #         drdx = np.clip(self.MPC_and_R(tra_pos+[0.001,0,0],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
    #         drdy = np.clip(self.MPC_and_R(tra_pos+[0,0.001,0],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
    #         drdz = np.clip(self.MPC_and_R(tra_pos+[0,0,0.001],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
    #         drda = np.clip(self.MPC_and_R(tra_pos,tra_ang=tra_ang+[0.001,0,0], t=t) - j,-0.5,0.5)
    #         drdb = np.clip(self.MPC_and_R(tra_pos,tra_ang=tra_ang+[0,0.001,0], t=t) - j,-0.5,0.5)
    #         drdc = np.clip(self.MPC_and_R(tra_pos,tra_ang=tra_ang+[0,0,0.001], t=t) - j,-0.5,0.5)
    #         #drdt = np.clip(self.MPC_and_R(tra_pos,tra_ang,t-0.1)-j,-10,10)
    #         # update
    #         tra_posx += 0.1*drdx
    #         tra_posy += 0.1*drdy
    #         tra_posz += 0.1*drdz
    #         tra_a += (1/(500*tra_a**2+5))*drda
    #         tra_b += (1/(500*tra_b**2+5))*drdb
    #         tra_c += (1/(500*tra_c**2+5))*drdc
    #         if((self.MPC_and_R(tra_pos,tra_ang,t-0.1)-j)>2):
    #             t = t-0.1
    #         if((self.MPC_and_R(tra_pos,tra_ang,t+0.1)-j)>2):
    #             t = t+0.1
    #         t = round(t,1)
    #         tra_pos = np.array([tra_posx,tra_posy,tra_posz])
    #         tra_ang = np.array([tra_a,tra_b,tra_c])
    #         ## display the process
    #         print(str(j)+str('  ')+str(tra_pos)+str('  ')+str(tra_ang)+str('  ')+str(t)+str('  ')+str(k))
    #     return [t,tra_posx,tra_posy,tra_posz,tra_a, tra_b,tra_c, j,self.collision,self.path]

    # ## use random perturbations to calculate the gradient and update(not recommonded)
    # def LSFD(self,t):
    #     tra_posx = self.obstacle.centroid[0]
    #     tra_posy = self.obstacle.centroid[1]
    #     tra_posz = self.obstacle.centroid[2]
    #     tra_a = 0
    #     tra_b = 0
    #     tra_c = 0
    #     current_para = np.array([tra_posx,tra_posy,tra_posz,tra_a,tra_b,tra_c])
    #     lr = np.array([2e-4,2e-4,2e-4,5e-5,5e-5,5e-5])
    #     for k in range(50):
    #         j = self.MPC_and_R(current_para[0:3],current_para[3:6],t)
    #         # calculate derivatives
    #         c = []
    #         f = []
    #         for i in range(24):
    #             dx = sample(0.001)
    #             dr = self.MPC_and_R (current_para[0:3]+dx[0:3],current_para[3:6]+dx[3:6],t)-j
    #             c += [dx]
    #             f += [dr]
    #         # update
    #         cm = np.array(c)
    #         fm = np.array(f)
    #         a = np.matmul(np.linalg.inv(np.matmul(cm.T,cm)),cm.T)
    #         drdx = np.matmul(a,fm)
    #         current_para = current_para + lr * drdx
    #         j = self.MPC_and_R(current_para[0:3],current_para[3:6],t)
    #         if((self.MPC_and_R(current_para[0:3],current_para[3:6],t+0.1)-j)>20):
    #             t = t + 0.1
    #         else:
    #             if((self.MPC_and_R(current_para[0:3],current_para[3:6],t-0.1)-j)>20):
    #                 t = t - 0.1
    #         t = round(t,1) 
    #         print(str(t)+str('  ')+str(drdx)+str('  ')+str(k))
    #     return [current_para, j,self.collision,self.path]        

    ## play the animation for one set of high-level paramters of such a scenario
    # def play_ani(self, tra_pos=None,tra_ang=None, t = 3,Ulast = None):
    #     tra_atti = Rd2Rp(tra_ang)
    #     self.uav.init_TraCost(tra_pos,tra_atti)
    #     self.uavoc.setTraCost(self.uav.tra_cost,t)
    #     ## obtain the trajectory
    #     self.sol1 = self.uavoc.ocSolver(horizon=self.horizon,dt=self.dt,Ulast=Ulast)
    #     state_traj1 = self.sol1['state_traj_opt']
    #     traj = get_quad_vert_pos(wing_len = self.wing_len, state_traj = state_traj1)
    #     ## plot the animation
    #     self.uav.play_animation(wing_len = self.wing_len, state_traj = state_traj1,dt=self.dt, point1 = self.point1,\
    #         point2 = self.point2, point3 = self.point3, point4 = self.point4)


        # def tra_ang_direct_penalty(self,tra_ang):
        # self.roll_penalty = - 1000 * 0.5 * tra_ang[0]**2
        # self.drdroll = - 1000 * tra_ang[0]

        # self.yaw_penalty = - 1000 * 0.5 * tra_ang[2]**2
        # self.drdyaw = - 1000 * tra_ang[2]

# --------------------------- solution and learning---------------------------------------
    # def sol_gradient(self,tra_pos =None,tra_ang=None,t_tra=None):
    #     """
    #     deprecated in the close loop training
    #     receive the decision variables from DNN1, do the MPC, then calculate d_penalty/d_z
    #     """

    #     tra_ang = np.array(tra_ang)
    #     tra_pos = np.array(tra_pos)

    #     # run the MPC to execute plan and execute based on the high-level variables
    #     # obtain solution of trajectory
    #     if self.options['PDP_GRADIENT']:
    #         NO_SOLUTION_FLAG = False
    #         trav_auxvar_value = np.concatenate((tra_pos,tra_ang,np.array([t_tra])))
    #         self.sol1,NO_SOLUTION_FLAG =self.mpc_update(cur_state=self.ini_state, 
    #                                                     trav_auxvar_value=trav_auxvar_value)
        
        
    #     # R is the penalty
    #     R = self.MPC_and_R(tra_pos,tra_ang,t_tra)
        
    #     ############==================finite difference===========================############
    #     if not self.options['PDP_GRADIENT']:
    #         # fixed perturbation to calculate the gradient
    #         delta = 1e-3
    #         drdx = np.clip(self.MPC_and_R(tra_pos+[delta,0,0],tra_ang, t_tra) - R,-0.5,0.5)*0.1
    #         drdy = np.clip(self.MPC_and_R(tra_pos+[0,delta,0],tra_ang, t_tra) - R,-0.5,0.5)*0.1
    #         drdz = np.clip(self.MPC_and_R(tra_pos+[0,0,delta],tra_ang, t_tra) - R,-0.5,0.5)*0.1
    #         drda = np.clip(self.MPC_and_R(tra_pos,tra_ang+[delta,0,0], t_tra) - R,-0.5,0.5)*(1/(500*tra_ang[0]**2+5))
    #         drdb = np.clip(self.MPC_and_R(tra_pos,tra_ang+[0,delta,0], t_tra) - R,-0.5,0.5)*(1/(500*tra_ang[1]**2+5))
    #         drdc = np.clip(self.MPC_and_R(tra_pos,tra_ang+[0,0,delta], t_tra) - R,-0.5,0.5)*(1/(500*tra_ang[2]**2+5))
    #         drdt =0
    #         if((self.MPC_and_R(tra_pos,tra_ang,t_tra-0.1)-R)>2):
    #             drdt = -0.05
    #         if((self.MPC_and_R(tra_pos,tra_ang,t_tra+0.1)-R)>2):
    #             drdt = 0.05

    #         # print("finite diff:",np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j]))
    #         return np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,R])
        
    #     ############==============end of finite difference===========================############
        
    #     ########################################################################
    #     #=======================SYMBOLIC GRADIENT+PDP===========================
    #     ########################################################################
    #     else:

    #         ## solve the PDP
    #         trav_auxvar_value = np.concatenate(tra_pos,tra_ang,np.array([t_tra]))
    #         self.PDP_grad(trav_auxvar_value)                
    
            
    #         drdp=np.zeros(13)
    #         for i in range(self.horizon):
    #             drdp += np.matmul(self.d_L_d_st_traj[i,:,:],self.d_st_traj_d_z[i,:,:]).reshape(len(drdp))

    #         drdp += np.matmul(self.d_L_d_st_traj[self.horizon,:,:],self.d_st_traj_d_z[self.horizon,:,:]).reshape(len(drdp))   
            
    #         # clip the traverse time gradient
    #         # drdp[:]=np.clip(drdp[:],-0.1,0.1)
            
    #         # drdp[3] = drdp[3]+self.drdroll
    #         # drdp[5] = drdp[5]+self.drdyaw

    #         drdp[-1] = np.clip(drdp[-1],-0.1,0.1)

    #         drdp = drdp/20000
    #         # drdx = np.clip(drdp[0],-0.02,0.02)
    #         # drdy = np.clip(drdp[1],-0.01,0.01)
    #         # drdz = np.clip(drdp[2],-0.02,0.02)
    #         # drda = np.clip(drdp[3],-0.02,0.02)
    #         # drdb = np.clip(drdp[4],-0.15,0.15)
    #         # drdc = np.clip(drdp[5],-0.02,0.02)
    #         # drdx = drdp[0]
    #         # drdy = drdp[1]
    #         # drdz = drdp[2]
    #         # drda = drdp[3]
    #         # drdb = drdp[4]
    #         # drdc = drdp[5]
            
    #         # drdt = drdp[6]
          

        
    #         # print("analytic grad:",np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j]))
    #         # print(drdp)
    #         # return np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,R])
    #         return np.concatenate((drdp,np.array([R])))

# def MPC_and_R(self,tra_pos=None,tra_ang=None,t_tra = 3):
    #     """
    #     deprecated in the close loop training
    #     """
    #     if not self.options['PDP_GRADIENT']:
    #         NO_SOLUTION_FLAG = False
    #         ## set the traverse hyperparameters value (auxvar) here
    #         trav_auxvar_value = np.concatenate((tra_pos,tra_ang,np.array([t_tra]))) #np.array([gamma]),
    #         self.sol1,NO_SOLUTION_FLAG =self.mpc_update(cur_state=self.ini_state, 
    #                                                     trav_auxvar_value=trav_auxvar_value)
    #     # state_traj [x,y,z,vx,vy,vz,qw,qx,qy,qz]
    #     state_traj = self.sol1['state_traj_opt']
    #     # get the quadrotor both center and edges position trajectory
    #     self.vert_traj = get_quad_vert_pos(wing_len = self.wing_len, state_traj = state_traj)

        
      
    #     # calculate trajectory penalty
    #     self.collision = 0
    #     self.path = 0
    #     ## detect whether there is collision
    #     self.co = 0

        

    #     if self.options['ORIGIN_penalty']:   
    #         for c in range(4):
    #             self.collision += self.obstacle.collis_det(self.vert_traj[:,3*(c+1):3*(c+2)],self.horizon)
    #             self.co += self.obstacle.co 

    #         ## calculate the path cost
    #         # check the drone centroid position error with the goal position
    #         for p in range(4):
    #             self.path += np.dot(self.vert_traj[self.horizon-1-p,0:3]-self.goal_pos, self.vert_traj[self.horizon-1-p,0:3]-self.goal_pos)
            
    #         # the sign of the collision is already negative
    #         # pitch angle penalty temproally be here
    #         # pitch_penalty =  0 * 0.5 * tra_ang[1]**2
    #         # self.drdpitch = 0 * tra_ang[1]
            
 
    #         return 1000 * self.collision - 0.5 * self.path + 100 #+ 10 * pitch_penalty

    #     else:
    #         # self.tra_ang_direct_penalty(tra_ang)

    #         penalty,self.d_L_d_st_traj=self.obstacle.penalty_cal_diff_collision(
    #                                                             self.config,
    #                                                             state_traj=state_traj,
    #                                                             gate_corners=self.gate_corners,
    #                                                             gate_quat=self.gate_quat,
    #                                                             vert_traj=self.vert_traj[:,0:3],
    #                                                             goal_pos=self.goal_pos)
            
    #         self.d_L_d_st_traj = self.d_L_d_st_traj.reshape(self.horizon+1,1,self.uavoc.n_state)
            
    #         return penalty #+ self.roll_penalty + self.yaw_penalty#+ pitch_penalty