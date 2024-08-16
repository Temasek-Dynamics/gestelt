##this file is to obtain the optimal solution

from casadi import *
import numpy
from scipy import interpolate
import casadi
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_template import AcadosModel
import time
import scipy
from os import system
class OCSys:

    def __init__(self,config_dict, project_name="my optimal control system"):
        self.project_name = project_name
        self.config_dict=config_dict
    def setAuxvarVariable(self, auxvar=None):
        if auxvar is None or auxvar.numel() == 0:
            self.auxvar = SX.sym('auxvar')
        else:
            self.auxvar = auxvar
        self.n_auxvar = self.auxvar.numel()

    def setStateVariable(self, state, state_lb=[], state_ub=[]):
        self.state = state
        self.n_state = self.state.numel()
        if len(state_lb) == self.n_state:
            self.state_lb = state_lb
        else:
            self.state_lb = self.n_state * [-1e20]

        if len(state_ub) == self.n_state:
            self.state_ub = state_ub
        else:
            self.state_ub = self.n_state * [1e20]

    def setControlVariable(self, control, control_lb=[], control_ub=[]):
        self.control = control
        self.n_control = self.control.numel()

        if len(control_lb) == self.n_control:
            self.control_lb = control_lb
        else:
            self.control_lb = self.n_control * [-1e20]

        if len(control_ub) == self.n_control:
            self.control_ub = control_ub
        else:
            self.control_ub = self.n_control * [1e20]

    def setDyn(self, f, dt):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        # self.dyn = casadi.Function('f',[self.state, self.control],[f])
        # self.dyn = self.state + dt * f
        # self.dyn_fn = casadi.Function('dynamics', [self.state, self.control, self.auxvar], [self.dyn],['x','u','p'],['x_next'])
        self.dyn_fn_acados = casadi.Function('dynamics', [self.state, self.control, self.auxvar], [f],['x','u','p'],['rhs'])
        #M = 4
        #DT = dt/4
        #X0 = casadi.SX.sym("X", self.n_state)
        #U = casadi.SX.sym("U", self.n_control)
        # #
        #X = X0
        #for _ in range(M):
            # --------- RK4------------
        #    k1 =DT*self.dyn(X, U)
        #    k2 =DT*self.dyn(X+0.5*k1, U)
        #    k3 =DT*self.dyn(X+0.5*k2, U)
        #    k4 =DT*self.dyn(X+k3, U)
            #
        #    X = X + (k1 + 2*k2 + 2*k3 + k4)/6        
        ## Fold
        #self.dyn_fn = casadi.Function('dyn', [X0, U], [X])

    def setInputCost(self, input_cost,wInputDiff):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert input_cost.numel() == 1, "input_cost must be a scalar function"        

        self.input_cost = input_cost
        self.input_cost_fn = casadi.Function('input_cost',[self.control, self.auxvar], [self.input_cost])
        self.wInputDiff = wInputDiff
    def setPathCost(self, 
                    path_cost,
                    goal_state_sym):
        self.goal_state_sym = goal_state_sym
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert path_cost.numel() == 1, "path_cost must be a scalar function"

        self.path_cost = path_cost
        self.path_cost_fn = casadi.Function('path_cost', [self.state,self.goal_state_sym,self.auxvar], [self.path_cost])

    def setFinalCost(self, 
                     final_cost,
                     goal_state_sym):
        self.goal_state_sym = goal_state_sym
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert final_cost.numel() == 1, "final_cost must be a scalar function"

        self.final_cost = final_cost
        self.final_cost_fn = casadi.Function('final_cost', [self.state,self.goal_state_sym, self.auxvar], [self.final_cost])
    
    def setTraCost(self, 
                tra_cost, 
                des_tra_r_I,
                des_tra_q):

        self.tra_cost = tra_cost
        self.des_tra_r_I = des_tra_r_I
        self.des_tra_q = des_tra_q
        self.tra_cost_fn = casadi.Function('tra_cost', [self.state, self.des_tra_r_I, self.des_tra_q, self.auxvar], [self.tra_cost])


    def ocSolverInit(self, horizon=None, auxvar_value=1, print_level=0, dt = 0.1,costate_option=0):
        assert hasattr(self, 'state'), "Define the state variable first!"
        assert hasattr(self, 'control'), "Define the control variable first!"
        assert hasattr(self, 'dyn'), "Define the system dynamics first!"
        assert hasattr(self, 'path_cost'), "Define the running cost function first!"
        assert hasattr(self, 'final_cost'), "Define the final cost function first!"
        

        #-----------------------------------------construct the CasiADi solver-----------------------------------------##
        # J: The objective function
        # w: The state and control variables
        # w0: the initial guess of the state and control variables solutions
        # p: symbolic parameters of the problem input
        # g: The equality for multiple shooting constraints
        self.horizon=horizon
        # Start with an empty NLP
        w = []
        self.w0 = []
        self.lbw = []
        self.ubw = []
        J = 0
        g = []
        self.lbg = []
        self.ubg = []


        # for solver to receive the current state and control
        X=SX.sym('X',self.n_state,self.horizon+1)
        U=SX.sym('U',self.n_control,self.horizon)
        P=SX.sym('P',self.n_state+self.n_control+1)
        
        
        
        # "Lift" initial conditions
        w += [X[:,0]]
        self.lbw += self.state_lb
        self.ubw +=  self.state_ub
        self.w0 += [0.5 * (x + y) for x, y in zip(self.state_lb, self.state_ub)]
        
        # current state and control constraints
        # g for multiple shooting constraints
        g += [X[:,0]-P[0:self.n_state]]
        self.lbg += self.n_state * [0]
        self.ubg += self.n_state * [0]

        Ulast=P[self.n_state:]
        
        # Formulate the NLP
        for k in range(int(self.horizon)):
         
            w += [U[:,k]]

            # control constraints
            self.lbw += self.control_lb
            self.ubw += self.control_ub
            self.w0 += [0.5 * (x + y) for x, y in zip(self.control_lb, self.control_ub)]

            #calculate weight
            # self.t: traverse time

            weight = 600*casadi.exp(-10*(dt*k-P[-1])**2) #gamma should increase as the flight duration decreases
             
            # Integrate till the end of the interval
            Xnext = self.dyn_fn(X[:,k], U[:,k],auxvar_value)

            # weight*self.tra_cost_fn(Xk, auxvar_value) + 
            Ck = weight*self.tra_cost_fn(X[:,k], auxvar_value) + self.path_cost_fn(X[:,k], auxvar_value)\
                +self.input_cost_fn(U[:,k], auxvar_value) #+ 1*dot(Uk-Ulast,Uk-Ulast)
                                                               
            J = J + Ck

        
            w += [X[:,k+1]]
            self.lbw += self.state_lb
            self.ubw += self.state_ub
            self.w0 += [0.5 * (x + y) for x, y in zip(self.state_lb, self.state_ub)]
            Ulast = U[:,k]

            # Add equality constraint, multiple shooting
            g += [Xnext - X[:,k+1]]
            self.lbg += self.n_state * [0]
            self.ubg += self.n_state * [0]

        # Adding the final cost
        J = J + self.final_cost_fn(X[:,k+1], auxvar_value)

        # Create an NLP solver and solve it
        opts = {'ipopt.max_iter':100,
                'ipopt.print_level': print_level, 
                'ipopt.sb': 'yes', 
                'print_time': print_level}
       

       
        # 1. Create a solver
        
        prob = {'f': J, 
                'x': vertcat(*w), 
                'p': P,
                'g': vertcat(*g)}
        
        # # acquire the current directory
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # # build the path to the subdirectory
        # self.so_path  = os.path.join(current_dir, 'casadi_saved/mpc_v1.so')
        # # self.so_path = "./casadi_saved/mpc_v1.so" 
        # self.solver = nlpsol('solver', 'ipopt', prob, opts)
        # # # jit (just-in-time compilation)
        # print("Generating shared library........")
        # cname = self.solver.generate_dependencies("mpc_v1.c")  
        # system('gcc -fPIC -shared -O3 ' + cname + ' -o ' + self.so_path) # -O3

        # load the generated code
        self.solver = nlpsol('solver', 'ipopt', prob, opts)
        
        
    def ocSolver(self,
                current_state_control, 
                auxvar_value=1, 
                costate_option=0,
                t_tra=1.0):
        # ------------------------------------------ use the solver -------------------------------------------##
        # 2. use the solver to Solve the NLP

        # self.w0: The initial state and control( initial state means current drone state)
        t_ = time.time()
        sol = self.solver(x0=self.w0,
                     lbx=self.lbw, 
                     ubx=self.ubw, 
                     p=current_state_control+ [t_tra], 
                     lbg=self.lbg, 
                     ubg=self.ubg)
        
        w_opt = sol['x'].full().flatten()
        # print('solving time=',time.time()-t_)
        # take the optimal control and state
        sol_traj = numpy.concatenate((w_opt, self.n_control * [0]))
        sol_traj = numpy.reshape(sol_traj, (-1, self.n_state + self.n_control))
        state_traj_opt = sol_traj[:, 0:self.n_state]
        control_traj_opt = numpy.delete(sol_traj[:, self.n_state:], -1, 0)
        current_time = numpy.array([k for k in range(self.horizon + 1)])

        # Compute the costates using two options
        if costate_option == 0:
            # Default option, which directly obtains the costates from the NLP solver
            costate_traj_opt = numpy.reshape(sol['lam_g'].full().flatten(), (-1, self.n_state))
        else:
            # Another option, which solve the costates by the Pontryagin's Maximum Principle
            # The variable name is consistent with the notations used in the PDP paper
            dfx_fun = casadi.Function('dfx', [self.state, self.control, self.auxvar], [jacobian(self.dyn, self.state)])
            dhx_fun = casadi.Function('dhx', [self.state, self.auxvar], [jacobian(self.final_cost, self.state)])
            dcx_fun = casadi.Function('dcx', [self.state, self.control, self.auxvar],
                                      [jacobian(self.path_cost, self.state)])
            costate_traj_opt = numpy.zeros((self.horizon, self.n_state))
            costate_traj_opt[-1, :] = dhx_fun(state_traj_opt[-1, :], auxvar_value)
            for k in range(self.horizon - 1, 0, -1):
                costate_traj_opt[k - 1, :] = dcx_fun(state_traj_opt[k, :], control_traj_opt[k, :],
                                                     auxvar_value).full() + numpy.dot(
                    numpy.transpose(dfx_fun(state_traj_opt[k, :], control_traj_opt[k, :], auxvar_value).full()),
                    costate_traj_opt[k, :])

        # output
        opt_sol = {"state_traj_opt": state_traj_opt,
                   "control_traj_opt": control_traj_opt,
                   "costate_traj_opt": costate_traj_opt,
                   'auxvar_value': auxvar_value,
                   "time": current_time,
                   "horizon": self.horizon,
                   "cost": sol['f'].full()}

        return opt_sol 

    def AcadosOcSolverInit(self, horizon=20, auxvar_value=1,
                            print_level=0, 
                            dt = 0.1,
                            costate_option=0,
                            SQP_RTI_OPTION=False,
                            USE_PREV_SOLVER=False
                            ):
        """
        This function is to define the optimal control problem using ACADOS
        """
       
        
        assert hasattr(self, 'state'), "Define the state variable first!"
        assert hasattr(self, 'control'), "Define the control variable first!"
        # assert hasattr(self, 'dyn'), "Define the system dynamics first!"
        assert hasattr(self, 'path_cost'), "Define the running cost function first!"
        assert hasattr(self, 'final_cost'), "Define the final cost function first!"
        # Start with an empty NLP
        self.horizon=horizon
        self.SQP_RTI_OPTION=SQP_RTI_OPTION

        # predict horizon in seconds
        T=horizon*dt
        self.n_nodes = horizon
        self.state_traj_opt = np.zeros((self.n_nodes+1,self.n_state))
        self.control_traj_opt = np.zeros((self.n_nodes,self.n_control))

        w = []
        self.w0 = []
        self.lbw = []
        self.ubw = []
        J = 0
        g = []
        self.lbg = []
        self.ubg = []


        ###############################################################
        ###############################################################
        ##----------model dynamic symbolic expression----------------##
        ###############################################################
        ###############################################################
        model=AcadosModel()
        
        
        ############################################################### 
        ##----------------- mapping CasADi to ACADOS ----------------##
        ###############################################################
        model.name="ACADOS_model" 

        # symbolical variables has already been defined in the quad_model.py
        """
        # define the state of the quadrotor
        rx, ry, rz = SX.sym('rx'), SX.sym('ry'), SX.sym('rz')
        self.r_I = vertcat(rx, ry, rz)
        vx, vy, vz = SX.sym('vx'), SX.sym('vy'), SX.sym('vz')
        self.v_I = vertcat(vx, vy, vz)
        
        # quaternions attitude of B w.r.t. I
        q0, q1, q2, q3 = SX.sym('q0'), SX.sym('q1'), SX.sym('q2'), SX.sym('q3')
        self.q = vertcat(q0, q1, q2, q3)
        wx, wy, wz = SX.sym('wx'), SX.sym('wy'), SX.sym('wz')
        self.w_B = vertcat(wx, wy, wz)
        
        # define the quadrotor input
        f1, f2, f3, f4 = SX.sym('f1'), SX.sym('f2'), SX.sym('f3'), SX.sym('f4')
        self.T_B = vertcat(f1, f2, f3, f4)


        ## define desire traverse pose
        self.des_tra_r_I = vertcat(SX.sym('des_tra_rx'), SX.sym('des_tra_ry'), SX.sym('des_tra_rz'))
        self.des_tra_q = vertcat(SX.sym('des_tra_q0'), SX.sym('des_tra_q1'), SX.sym('des_tra_q2'), SX.sym('des_tra_q3'))

        """
        # explicit model
        model.f_expl_expr=self.dyn_fn_acados(self.state,self.control,self.auxvar)

        # implicit model
        x_dot=casadi.SX.sym('x_dot',self.n_state)
        model.f_impl_expr=x_dot-self.dyn_fn_acados(self.state,self.control,self.auxvar)

        # self.state = vertcat(self.r_I, self.v_I, self.q, self.w_B)
        model.x=self.state
        model.xdot=x_dot

        #  self.control = self.T_B
        model.u=self.control
        
        ## parameters: received after solver initialization, includes:
        # goal state: 
        # Ulast: current state can be set as constraints 
        # desired traverse pose: des_tra_r_I, des_tra_q
        # tra_cost: weight
       
        P=casadi.SX.sym('p',self.n_state+\
                        self.n_control+\
                        self.des_tra_r_I.numel()\
                        +self.des_tra_q.numel()+1)
        model.p=P
        
        ###############################################################
        ###############################################################
        ##-------------------------optimizer-------------------------##
        ###############################################################
        ###############################################################


        ############################################################### 
        ##------------------set the environment path-----------------##
        ###############################################################     
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        ## get the ACAODS path
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)

        ###############################################################  
        ##------------------build the ACADOS ocp   ------------------##
        ############################################################### 
        ocp=AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        


        ############################################################### 
        ##------------------ setting the ocp model ------------------##
        ############################################################### 
        ocp.model = model
        ocp.dims.N = self.n_nodes    # number of nodes 
        ocp.solver_options.tf = T # horizon length T in seconds
        ocp.dims.np = self.n_state+self.n_control+self.des_tra_r_I.numel()+self.des_tra_q.numel()+1    # number of parameters for solver input, here is the current state and control
        ocp.parameter_values = np.zeros(self.n_state+self.n_control+self.des_tra_r_I.numel()+self.des_tra_q.numel()+1) 



        ############################################################### 
        ##------------ setting the cost function---------------------##
        ############################################################### 


        #-------------------------external cost-------------------------##
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        Ulast=ocp.model.p[self.n_state:self.n_state+self.n_control]
        goal_state=ocp.model.p[0:self.n_state]  
        des_tra_pos=ocp.model.p[self.n_state+self.n_control : self.n_state+self.n_control+3]
        des_tra_q=ocp.model.p[self.n_state+self.n_control+3 : self.n_state+self.n_control+7]
    

        # # setting the cost function
        # weight = 60*casadi.exp(-10*(dt*k-model.p[-1])**2) #gamma should increase as the flight duration decreases
        # ocp.model.cost_expr_ext_cost_custom_hess/cost_expr_ext_cost
        ocp.model.cost_expr_ext_cost = self.path_cost_fn(ocp.model.x,goal_state,self.auxvar)\
            +self.final_cost_fn(ocp.model.x,goal_state,self.auxvar)\
            +ocp.model.p[-1]*self.tra_cost_fn(ocp.model.x, des_tra_pos,des_tra_q, self.auxvar)\
            +self.wInputDiff*dot(ocp.model.u-Ulast,ocp.model.u-Ulast) \
            +self.input_cost_fn(ocp.model.u,self.auxvar)
        
        # end cost
        ocp.model.cost_expr_ext_cost_e = self.final_cost_fn(ocp.model.x,goal_state,self.auxvar)

        Ulast=ocp.model.u
        ############################################################### 
        ##----------------- setting the constraints -----------------##
        ###############################################################  

           
        ##------------------control constraints----------------------##
        # # will set this initial value for all N nodes states
        x_init = np.zeros((self.n_state))
        x_init[6] = 1.0 # w for the quaternion
        ocp.constraints.x0 = x_init

        # 4x1
        ocp.constraints.lbu = np.array(self.control_lb) 
        ocp.constraints.ubu = np.array(self.control_ub)
        ocp.constraints.idxbu = np.array([i for i in range(self.n_control)])
        
        
        ##------------------ state constraints ----------------------##
        # # constraint for position ( no constraints for the state)
        ocp.constraints.lbx = np.array(self.state_lb) #([])#
        ocp.constraints.ubx = np.array(self.state_ub) #([])#
        ocp.constraints.idxbx = np.array([i for i in range(self.n_state)]) #([])#i for i in range(self.n_state)]
        



        ##------------------ setting the solver ------------------##
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'# FULL_CONDENSING_HPIPM PARTIAL_CONDENSING_HPIPM  FULL_CONDENSING_QPOASES PARTIAL_CONDENSING_OSQP
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # GAUSS_NEWTON, EXACT
        ocp.solver_options.regularize_method = 'CONVEXIFY'#'CONVEXIFY', PROJECT_REDUC_HESS
        ocp.solver_options.integrator_type = 'ERK' # ERK (explicit Runge-Kutta integrator) or IRK (Implicit Runge-Kutta integrator)
        ocp.solver_options.sim_method_num_stages = 4 # default 4
        ocp.solver_options.print_level = 0
        ocp.solver_options.levenberg_marquardt = 1e-5 # small value for gauss newton method, large value for gradient descent method
        
        if SQP_RTI_OPTION: 
            # for deployment
            ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        else:
            ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI or SQP
        # ocp.solver_options.qp_solver_warm_start=2
        # ocp.solver_options.nlp_solver_max_iter = 100
        ##------------------ setting the code generation ------------------##
        # compile acados ocp
        json_file = os.path.join('./'+model.name+'_acados_ocp.json')

        # load solver from json file
        build=True
        generate=True
        if USE_PREV_SOLVER:
            build=False
            generate=False
        self.acados_solver = AcadosOcpSolver(ocp,generate=generate,build=build, json_file=json_file)
 


    def AcadosOcSolver(self, 
                    current_state_control, 
                    goal_pos,
                    goal_ori,
                    auxvar_value=1, 
                    costate_option=0,
                    dt=0.1,
                    tra_pos=np.array([0,0,1.5]),
                    tra_q=np.array([1,0,0,0]),
                    t_tra=1.0,
                    max_tra_w=60):
        """
        This function is to solve the optimal control problem using ACADOS
        """
        # #---------------------for linear cost---------------------##
        # # #set desired ref state
        desired_goal_vel=np.array([0, 0, 0])
        desired_goal_ori = np.array(goal_ori)

        goal_state=np.concatenate((np.array(goal_pos),desired_goal_vel,desired_goal_ori))#,desired_goal_w))
        
        # set the desired state-control at 0->N-1 nodes
        for i in range(self.n_nodes):
            
            # set the current input
            current_input = np.array(current_state_control[self.n_state:])
            # current_input = np.array([0,0,0,0])
            gamma=self.config_dict['learning_agile']['traverse_weight_span']
            # weight = max_tra_w*casadi.exp(-gamma*(dt*i-t_tra)**2) #gamma should increase as the flight duration decreases
            weight=max_tra_w*np.exp(-gamma*(dt*i-t_tra)**2) #gamma should increase as the flight duration decreases
            self.acados_solver.set(i, 'p',np.concatenate((goal_state,
                                                          current_input,# current is dummy
                                                          np.concatenate((tra_pos,tra_q)),
                                                          np.array([weight]))))
            if i==10:
                weight_vis=weight
            

        # set the last state-control as the initial guess for the last node
        self.acados_solver.set(self.n_nodes, "x", self.state_traj_opt[-1,:])

        # set the end desired goal
        weight = 0.0*casadi.exp(-10*(dt*self.n_nodes-t_tra)**2) #gamma should increase as the flight duration decreases
        self.acados_solver.set(self.n_nodes, "p",np.concatenate((goal_state,
                                                                 current_input,
                                                                 np.concatenate((tra_pos,tra_q)),
                                                                 np.array([weight]))))

        # set initial condition aligned with the current state
        self.acados_solver.set(0, "lbx", np.array(current_state_control[0:self.n_state]))
        self.acados_solver.set(0, "ubx", np.array(current_state_control[0:self.n_state]))
        
       
        NO_SOLUTION_FLAG=False
        # solve ocp
       
        status = self.acados_solver.solve()

        if status != 0:
            NO_SOLUTION_FLAG=True
        
        #-------------take the optimal control and state sequences
        #self.n_nodes
        for i in range(self.n_nodes):
            self.state_traj_opt[i,:]=self.acados_solver.get(i, "x")
            self.control_traj_opt[i,:]=self.acados_solver.get(i, "u")
        self.state_traj_opt[-1,:]=self.acados_solver.get(self.n_nodes, "x")
        
       
            
        # output
        opt_sol = {"state_traj_opt": self.state_traj_opt,
                "control_traj_opt": self.control_traj_opt,
                'auxvar_value': auxvar_value,
                "time": time,
                "horizon": self.horizon}
                #"cost": sol['f'].full()}
    

        return opt_sol,weight_vis ,NO_SOLUTION_FLAG   
    


    # def diffPMP(self):
    #     assert hasattr(self, 'state'), "Define the state variable first!"
    #     assert hasattr(self, 'control'), "Define the control variable first!"
    #     assert hasattr(self, 'dyn'), "Define the system dynamics first!"
    #     assert hasattr(self, 'path_cost'), "Define the running cost/reward function first!"
    #     assert hasattr(self, 'final_cost'), "Define the final cost/reward function first!"

    #     # Define the Hamiltonian function
    #     self.costate = casadi.SX.sym('lambda', self.state.numel())
    #     self.path_Hamil = self.path_cost + dot(self.dyn, self.costate)  # path Hamiltonian
    #     self.final_Hamil = self.final_cost  # final Hamiltonian

    #     # Differentiating dynamics; notations here are consistent with the PDP paper
    #     self.dfx = jacobian(self.dyn, self.state)
    #     self.dfx_fn = casadi.Function('dfx', [self.state, self.control, self.auxvar], [self.dfx])
    #     self.dfu = jacobian(self.dyn, self.control)
    #     self.dfu_fn = casadi.Function('dfu', [self.state, self.control, self.auxvar], [self.dfu])
    #     self.dfe = jacobian(self.dyn, self.auxvar)
    #     self.dfe_fn = casadi.Function('dfe', [self.state, self.control, self.auxvar], [self.dfe])

    #     # First-order derivative of path Hamiltonian
    #     self.dHx = jacobian(self.path_Hamil, self.state).T
    #     self.dHx_fn = casadi.Function('dHx', [self.state, self.control, self.costate, self.auxvar], [self.dHx])
    #     self.dHu = jacobian(self.path_Hamil, self.control).T
    #     self.dHu_fn = casadi.Function('dHu', [self.state, self.control, self.costate, self.auxvar], [self.dHu])

    #     # Second-order derivative of path Hamiltonian
    #     self.ddHxx = jacobian(self.dHx, self.state)
    #     self.ddHxx_fn = casadi.Function('ddHxx', [self.state, self.control, self.costate, self.auxvar], [self.ddHxx])
    #     self.ddHxu = jacobian(self.dHx, self.control)
    #     self.ddHxu_fn = casadi.Function('ddHxu', [self.state, self.control, self.costate, self.auxvar], [self.ddHxu])
    #     self.ddHxe = jacobian(self.dHx, self.auxvar)
    #     self.ddHxe_fn = casadi.Function('ddHxe', [self.state, self.control, self.costate, self.auxvar], [self.ddHxe])
    #     self.ddHux = jacobian(self.dHu, self.state)
    #     self.ddHux_fn = casadi.Function('ddHux', [self.state, self.control, self.costate, self.auxvar], [self.ddHux])
    #     self.ddHuu = jacobian(self.dHu, self.control)
    #     self.ddHuu_fn = casadi.Function('ddHuu', [self.state, self.control, self.costate, self.auxvar], [self.ddHuu])
    #     self.ddHue = jacobian(self.dHu, self.auxvar)
    #     self.ddHue_fn = casadi.Function('ddHue', [self.state, self.control, self.costate, self.auxvar], [self.ddHue])

    #     # First-order derivative of final Hamiltonian
    #     self.dhx = jacobian(self.final_Hamil, self.state).T
    #     self.dhx_fn = casadi.Function('dhx', [self.state, self.auxvar], [self.dhx])

    #     # second order differential of path Hamiltonian
    #     self.ddhxx = jacobian(self.dhx, self.state)
    #     self.ddhxx_fn = casadi.Function('ddhxx', [self.state, self.auxvar], [self.ddhxx])
    #     self.ddhxe = jacobian(self.dhx, self.auxvar)
    #     self.ddhxe_fn = casadi.Function('ddhxe', [self.state, self.auxvar], [self.ddhxe])

    # def getAuxSys(self, state_traj_opt, control_traj_opt, costate_traj_opt, auxvar_value=1):
    #     statement = [hasattr(self, 'dfx_fn'), hasattr(self, 'dfu_fn'), hasattr(self, 'dfe_fn'),
    #                  hasattr(self, 'ddHxx_fn'), \
    #                  hasattr(self, 'ddHxu_fn'), hasattr(self, 'ddHxe_fn'), hasattr(self, 'ddHux_fn'),
    #                  hasattr(self, 'ddHuu_fn'), \
    #                  hasattr(self, 'ddHue_fn'), hasattr(self, 'ddhxx_fn'), hasattr(self, 'ddhxe_fn'), ]
    #     if not all(statement):
    #         self.diffPMP()

    #     # Initialize the coefficient matrices of the auxiliary control system: note that all the notations used here are
    #     # consistent with the notations defined in the PDP paper.
    #     dynF, dynG, dynE = [], [], []
    #     matHxx, matHxu, matHxe, matHux, matHuu, matHue, mathxx, mathxe = [], [], [], [], [], [], [], []

    #     # Solve the above coefficient matrices
    #     for t in range(numpy.size(control_traj_opt, 0)):
    #         curr_x = state_traj_opt[t, :]
    #         curr_u = control_traj_opt[t, :]
    #         next_lambda = costate_traj_opt[t, :]
    #         dynF += [self.dfx_fn(curr_x, curr_u, auxvar_value).full()]
    #         dynG += [self.dfu_fn(curr_x, curr_u, auxvar_value).full()]
    #         dynE += [self.dfe_fn(curr_x, curr_u, auxvar_value).full()]
    #         matHxx += [self.ddHxx_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #         matHxu += [self.ddHxu_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #         matHxe += [self.ddHxe_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #         matHux += [self.ddHux_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #         matHuu += [self.ddHuu_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #         matHue += [self.ddHue_fn(curr_x, curr_u, next_lambda, auxvar_value).full()]
    #     mathxx = [self.ddhxx_fn(state_traj_opt[-1, :], auxvar_value).full()]
    #     mathxe = [self.ddhxe_fn(state_traj_opt[-1, :], auxvar_value).full()]

    #     auxSys = {"dynF": dynF,
    #               "dynG": dynG,
    #               "dynE": dynE,
    #               "Hxx": matHxx,
    #               "Hxu": matHxu,
    #               "Hxe": matHxe,
    #               "Hux": matHux,
    #               "Huu": matHuu,
    #               "Hue": matHue,
    #               "hxx": mathxx,
    #               "hxe": mathxe}
    #     return auxSys