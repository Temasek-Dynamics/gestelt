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

'''
# =============================================================================================================
# The OCSys class has multiple functionaries: 1) define an optimal control system, 2) solve the optimal control
# system, and 3) obtain the auxiliary control system.

# The standard form of the dynamics of an optimal control system is
# x_k+1= f（x_k, u_k, auxvar)
# The standard form of the cost function of an optimal control system is
# J = sum_0^(T-1) path_cost + final_cost,
# where path_cost = c(x, u, auxvar) and final_cost= h(x, auxvar).
# Note that in the above standard optimal control system form, "auxvar" is the parameter (which can be learned)
# If you don't need the parameter, e.g.m you just want to use this class to solve an optimal control problem,
# instead of learning the parameter, you can ignore setting this augment in your code.

# The procedure to use this class is fairly straightforward, just understand each method by looking at its name:
# Step 1: set state variable ----> setStateVariable
# Step 2: set control variable ----> setControlVariable
# Step 3: set parameter (if applicable) ----> setAuxvarVariable; otherwise you can ignore this step.
# Step 4: set dynamics equation----> setDyn
# Step 5: set path cost function ----> setPathCost
# Step 6: set final cost function -----> setFinalCost
# Step 7: solve the optimal control problem -----> ocSolver
# Step 8: differentiate the Pontryagin's maximum principle (if you have Step 3) -----> diffPMP
# Step 9: get the auxiliary control system (if have Step 3) ------> getAuxSys

# Note that if you are not wanting to learn the parameter in an optimal control system, you can ignore Step 3. 8. 9.
# Note that most of the notations used here are consistent with the notations defined in the PDP paper.
'''
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
        self.dyn = self.state + dt * f
        self.dyn_fn = casadi.Function('dynamics', [self.state, self.control, self.auxvar], [self.dyn],['x','u','p'],['x_next'])
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

    def setInputCost(self,input_cost):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert input_cost.numel() == 1, "input_cost must be a scalar function"        
        
        self.input_cost = input_cost
        self.input_cost_fn = casadi.Function('input_cost',[self.control, self.auxvar], [self.input_cost])

    def setInputDiffCost(self,input_diff_cost):   
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert input_diff_cost.numel() == 1, "input_diff_cost must be a scalar function"

        self.input_diff_cost = input_diff_cost
        self.input_diff_cost_fn = casadi.Function('input_diff_cost', [self.control, self.auxvar], [self.input_diff_cost])
    
    
    def setPathCost(self, path_cost):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert path_cost.numel() == 1, "path_cost must be a scalar function"

        self.path_cost = path_cost
        self.path_cost_fn = casadi.Function('path_cost', [self.state,  self.auxvar], [self.path_cost])
    
    
    def setFinalCost(self, final_cost):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert final_cost.numel() == 1, "final_cost must be a scalar function"

        self.final_cost = final_cost
        self.final_cost_fn = casadi.Function('final_cost', [self.state, self.auxvar], [self.final_cost])


    def setTraCost(self, 
                trav_cost):
                

        self.trav_cost = trav_cost
        self.trav_cost_fn = casadi.Function('trav_cost', [self.state,self.trav_auxvar], [self.trav_cost])

    def setInputDiffCostAllSym(self, Ulast,input_diff_cost_all_sym):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert input_diff_cost_all_sym.numel() == 1, "input_diff_cost must be a scalar function"

        self.Ulast = Ulast
        self.input_diff_cost_all_sym = input_diff_cost_all_sym
        self.input_diff_cost_all_sym_fn = casadi.Function('input_diff_cost', [self.control,self.Ulast, self.auxvar], [self.input_diff_cost_all_sym])

    def setPathCostAllSym(self, 
                    path_cost_all_sym,
                    goal_state_sym):
        
        " all symbolic path cost function, means the goal state is also a symbolic variable"

        self.goal_state_sym = goal_state_sym
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert path_cost_all_sym.numel() == 1, "path_cost must be a scalar function"

        self.path_cost_all_sym = path_cost_all_sym
        self.path_cost_all_sym_fn = casadi.Function('path_cost', [self.state,self.goal_state_sym,self.auxvar], [self.path_cost_all_sym])


    def setFinalCostAllSym(self, 
                     final_cost_all_sym,
                     goal_state_sym):
        " all symbolic path cost function, means the goal state is also a symbolic variable"

        self.goal_state_sym = goal_state_sym
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert final_cost_all_sym.numel() == 1, "final_cost must be a scalar function"

        self.final_cost_all_sym = final_cost_all_sym
        self.final_cost_all_sym_fn = casadi.Function('final_cost_all_sym', [self.state,self.goal_state_sym, self.auxvar], [self.final_cost_all_sym])
    
    def setTraCostAllSym(self, 
                trav_cost, 
                trav_auxvar,
                t_node):
                

        self.trav_cost = trav_cost
        self.trav_auxvar = trav_auxvar
        self.n_trav_auxvar = self.trav_auxvar.numel()
        self.t_node = t_node
        self.trav_cost_all_sym_fn = casadi.Function('trav_cost', [self.state,self.trav_auxvar,self.t_node], [self.trav_cost])



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

            # weight*self.trav_cost_fn(Xk, auxvar_value) + 
            Ck = weight*self.trav_cost_fn(X[:,k], auxvar_value) + self.path_cost_fn(X[:,k], auxvar_value)\
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
        # assert hasattr(self, 'path_cost'), "Define the running cost function first!"
        assert hasattr(self, 'final_cost_all_sym'), "Define the final cost function first!"
        # Start with an empty NLP
        self.horizon=horizon
        self.SQP_RTI_OPTION=SQP_RTI_OPTION

        # predict horizon in seconds
        T=horizon*dt
        self.n_nodes = horizon

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
        # Ulast_value: current state can be set as constraints 
        
        # replace:
            # desired traverse pose: des_tra_r_I, des_tra_q
            # trav_cost: weight
        # to
            # desired traverse pose: des_tra_r_I, des_tra_rodi_param  (Rodrigues param)
            # t_tra: traverse time
            # t_node: current node time

       
        P=casadi.SX.sym('p',self.n_state+\
                        self.n_control+\
                        self.trav_auxvar.numel()+1) # the last one is the current node time
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
        ocp.dims.np = self.n_state+self.n_control+self.trav_auxvar.numel()+1    # number of parameters for solver input, here is the current state and control
        ocp.parameter_values = np.zeros(self.n_state+self.n_control+self.trav_auxvar.numel()+1) 



        ############################################################### 
        ##------------ setting the cost function---------------------##
        ############################################################### 


        #-------------------------external cost-------------------------##
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        Ulast_value=ocp.model.p[self.n_state:self.n_state+self.n_control]
        goal_state=ocp.model.p[0:self.n_state]  
        # des_tra_pos=ocp.model.p[self.n_state+self.n_control : self.n_state+self.n_control+3]
        # des_tra_q=ocp.model.p[self.n_state+self.n_control+3 : self.n_state+self.n_control+7]
    
        # self.trav_auxvar = vertcat(self.des_tra_r_I, self.des_tra_rodi_param,self.des_t_tra)
        trav_auxvar_value=ocp.model.p[self.n_state+self.n_control:self.n_state+self.n_control+self.trav_auxvar.numel()]

        # current node time
        t_node=ocp.model.p[-1]

        # # setting the cost function
        # weight = 60*casadi.exp(-10*(dt*k-model.p[-1])**2) #gamma should increase as the flight duration decreases
        # ocp.model.cost_expr_ext_cost_custom_hess/cost_expr_ext_cost
        ocp.model.cost_expr_ext_cost = self.path_cost_all_sym_fn(ocp.model.x,goal_state,self.auxvar)\
            +self.final_cost_all_sym_fn(ocp.model.x,goal_state,self.auxvar)\
            +self.trav_cost_all_sym_fn(ocp.model.x, trav_auxvar_value, t_node)\
            +self.input_diff_cost_all_sym_fn(ocp.model.u,Ulast_value,self.auxvar) \
            +self.input_cost_fn(ocp.model.u,self.auxvar)
        
        # end cost
        ocp.model.cost_expr_ext_cost_e = self.final_cost_all_sym_fn(ocp.model.x,goal_state,self.auxvar)

        Ulast_value=ocp.model.u
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
                    tra_ang=np.array([0,0,0]),
                    t_tra=1.0,
                    max_tra_w=60):
        """
        This function is to solve the optimal control problem using ACADOS
        """
        self.state_traj_opt = np.zeros((self.n_nodes+1,self.n_state))
        self.control_traj_opt = np.zeros((self.n_nodes,self.n_control))
        self.costate_traj_opt = np.zeros((self.n_nodes,self.n_state))

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
          
            # weight = max_tra_w*casadi.exp(-gamma*(dt*i-t_tra)**2) #gamma should increase as the flight duration decreases
            # weight=max_tra_w*np.exp(-gamma*(dt*i-t_tra)**2) #gamma should increase as the flight duration decreases
            
            self.acados_solver.set(i, 'p',np.concatenate((goal_state,
                                                          current_input,# current is dummy
                                                          np.concatenate((tra_pos,tra_ang,np.array([t_tra]))),
                                                          np.array([dt*i]))))
            # if i==10:
            #     weight_vis=weight
            

        # set the last state-control as the initial guess for the last node
        self.acados_solver.set(self.n_nodes, "x", self.state_traj_opt[-1,:])

        # set the end desired goal
        weight = 0.0*casadi.exp(-10*(dt*self.n_nodes-t_tra)**2) #gamma should increase as the flight duration decreases
        self.acados_solver.set(self.n_nodes, "p",np.concatenate((goal_state,
                                                                 current_input,
                                                                 np.concatenate((tra_pos,tra_ang,np.array([t_tra]))),
                                                                 np.array([self.n_nodes*dt]))))

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
            self.costate_traj_opt[i,:]=self.acados_solver.get(i, "pi")

        self.state_traj_opt[-1,:]=self.acados_solver.get(self.n_nodes, "x")
        
       
            
        # output
        opt_sol = {"state_traj_opt": self.state_traj_opt,
                "control_traj_opt": self.control_traj_opt,
                "costate_traj_opt": self.costate_traj_opt,
                'auxvar_value': auxvar_value,
                "time": time,
                "horizon": self.horizon}
                #"cost": sol['f'].full()}
    

        return opt_sol,NO_SOLUTION_FLAG   
    

    
        
    def diffPMP(self):
        assert hasattr(self, 'state'), "Define the state variable first!"
        assert hasattr(self, 'control'), "Define the control variable first!"
        assert hasattr(self, 'dyn'), "Define the system dynamics first!"
        assert hasattr(self, 'path_cost'), "Define the running cost/reward function first!"
        assert hasattr(self, 'final_cost'), "Define the final cost/reward function first!"

        # Define the Hamiltonian function
        self.costate = casadi.SX.sym('lambda', self.state.numel())
        self.path_Hamil = self.path_cost \
                        + self.trav_cost\
                        + self.input_cost \
                        + self.input_diff_cost \
                        + dot(self.dyn, self.costate)  # path Hamiltonian
        
        self.final_Hamil = self.final_cost  # final Hamiltonian

        # Differentiating dynamics; notations here are consistent with the PDP paper
        self.dfx = jacobian(self.dyn, self.state)
        self.dfx_fn = casadi.Function('dfx', [self.state, self.control, self.trav_auxvar], [self.dfx])
        self.dfu = jacobian(self.dyn, self.control)
        self.dfu_fn = casadi.Function('dfu', [self.state, self.control, self.trav_auxvar], [self.dfu])
        self.dfe = jacobian(self.dyn, self.trav_auxvar)
        self.dfe_fn = casadi.Function('dfe', [self.state, self.control, self.trav_auxvar], [self.dfe])

        # First-order derivative of path Hamiltonian
        self.dHx = jacobian(self.path_Hamil, self.state).T
        self.dHx_fn = casadi.Function('dHx', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.dHx])
        self.dHu = jacobian(self.path_Hamil, self.control).T
        self.dHu_fn = casadi.Function('dHu', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.dHu])

        # Second-order derivative of path Hamiltonian
        self.ddHxx = jacobian(self.dHx, self.state)
        self.ddHxx_fn = casadi.Function('ddHxx', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHxx])
        self.ddHxu = jacobian(self.dHx, self.control)
        self.ddHxu_fn = casadi.Function('ddHxu', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHxu])
        self.ddHxe = jacobian(self.dHx, self.trav_auxvar)
        self.ddHxe_fn = casadi.Function('ddHxe', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHxe])
        self.ddHux = jacobian(self.dHu, self.state)
        self.ddHux_fn = casadi.Function('ddHux', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHux])
        self.ddHuu = jacobian(self.dHu, self.control)
        self.ddHuu_fn = casadi.Function('ddHuu', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHuu])
        self.ddHue = jacobian(self.dHu, self.trav_auxvar)
        self.ddHue_fn = casadi.Function('ddHue', [self.state, self.control, self.costate, self.t_node, self.trav_auxvar], [self.ddHue])

        # First-order derivative of final Hamiltonian
        self.dhx = jacobian(self.final_Hamil, self.state).T
        self.dhx_fn = casadi.Function('dhx', [self.state, self.trav_auxvar], [self.dhx])

        # second order differential of path Hamiltonian
        self.ddhxx = jacobian(self.dhx, self.state)
        self.ddhxx_fn = casadi.Function('ddhxx', [self.state, self.trav_auxvar], [self.ddhxx])
        self.ddhxe = jacobian(self.dhx, self.trav_auxvar)
        self.ddhxe_fn = casadi.Function('ddhxe', [self.state, self.trav_auxvar], [self.ddhxe])

    def getAuxSys(self,state_traj_opt, 
                   control_traj_opt, 
                   costate_traj_opt, 
                   auxvar_value=1):
        
        statement = [hasattr(self, 'dfx_fn'), hasattr(self, 'dfu_fn'), hasattr(self, 'dfe_fn'),
                     hasattr(self, 'ddHxx_fn'), \
                     hasattr(self, 'ddHxu_fn'), hasattr(self, 'ddHxe_fn'), hasattr(self, 'ddHux_fn'),
                     hasattr(self, 'ddHuu_fn'), \
                     hasattr(self, 'ddHue_fn'), hasattr(self, 'ddhxx_fn'), hasattr(self, 'ddhxe_fn'), ]
        if not all(statement):
            self.diffPMP()

        # Initialize the coefficient matrices of the auxiliary control system: note that all the notations used here are
        # consistent with the notations defined in the PDP paper.
        dynF, dynG, dynE = [], [], []
        matHxx, matHxu, matHxe, matHux, matHuu, matHue, mathxx, mathxe = [], [], [], [], [], [], [], []

        # Solve the above coefficient matrices
        for t in range(numpy.size(control_traj_opt, 0)):
            curr_x = state_traj_opt[t, :]
            curr_u = control_traj_opt[t, :]
            next_lambda = costate_traj_opt[t, :]
            dynF += [self.dfx_fn(curr_x, curr_u, auxvar_value).full()]
            dynG += [self.dfu_fn(curr_x, curr_u, auxvar_value).full()]
            dynE += [self.dfe_fn(curr_x, curr_u, auxvar_value).full()]
            matHxx += [self.ddHxx_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
            matHxu += [self.ddHxu_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
            matHxe += [self.ddHxe_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
            matHux += [self.ddHux_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
            matHuu += [self.ddHuu_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
            matHue += [self.ddHue_fn(curr_x, curr_u, next_lambda, t, auxvar_value).full()]
        mathxx = [self.ddhxx_fn(state_traj_opt[-1, :], auxvar_value).full()]
        mathxe = [self.ddhxe_fn(state_traj_opt[-1, :], auxvar_value).full()]

        auxSys = {"dynF": dynF,
                  "dynG": dynG,
                  "dynE": dynE,
                  "Hxx": matHxx,
                  "Hxu": matHxu,
                  "Hxe": matHxe,
                  "Hux": matHux,
                  "Huu": matHuu,
                  "Hue": matHue,
                  "hxx": mathxx,
                  "hxe": mathxe}
        return auxSys
    

'''
# =============================================================================================================
# The LQR class is mainly for solving (time-varying or time-invariant) LQR problems.
# The standard form of the dynamics in the LQR system is
# X_k+1=dynF_k*X_k+dynG_k*U_k+dynE_k,
# where matrices dynF_k, dynG_k, and dynE_k are system dynamics matrices you need to specify (maybe time-varying)
# The standard form of cost function for the LQR system is
# J=sum_0^(horizon-1) path_cost + final cost, where
# path_cost  = trace (1/2*X'*Hxx*X +1/2*U'*Huu*U + 1/2*X'*Hxu*U + 1/2*U'*Hux*X + Hue'*U + Hxe'*X)
# final_cost = trace (1/2*X'*hxx*X +hxe'*X)
# Here, Hxx, Huu, Hux, Hxu, Heu, Hex, hxx, hex are cost matrices you need to specify (maybe time-varying).
# Some of the above dynamics and cost matrices, by default, are zero (none) matrices
# Note that the variable X and variable U can be matrix variables.
# The above defined standard form is consistent with the auxiliary control system defined in the PDP paper
'''


class LQR:

    def __init__(self, project_name="LQR system"):
        self.project_name = project_name

    def setDyn(self, dynF, dynG, dynE=None):
        if type(dynF) is numpy.ndarray:
            self.dynF = [dynF]
            self.n_state = numpy.size(dynF, 0)
        elif type(dynF[0]) is numpy.ndarray:
            self.dynF = dynF
            self.n_state = numpy.size(dynF[0], 0)
        else:
            assert False, "Type of dynF matrix should be numpy.ndarray  or list of numpy.ndarray"

        if type(dynG) is numpy.ndarray:
            self.dynG = [dynG]
            self.n_control = numpy.size(dynG, 1)
        elif type(dynG[0]) is numpy.ndarray:
            self.dynG = dynG
            self.n_control = numpy.size(self.dynG[0], 1)
        else:
            assert False, "Type of dynG matrix should be numpy.ndarray  or list of numpy.ndarray"

        if dynE is not None:
            if type(dynE) is numpy.ndarray:
                self.dynE = [dynE]
                self.n_batch = numpy.size(dynE, 1)
            elif type(dynE[0]) is numpy.ndarray:
                self.dynE = dynE
                self.n_batch = numpy.size(dynE[0], 1)
            else:
                assert False, "Type of dynE matrix should be numpy.ndarray, list of numpy.ndarray, or None"
        else:
            self.dynE = None
            self.n_batch = None

    def setPathCost(self, Hxx, Huu, Hxu=None, Hux=None, Hxe=None, Hue=None):

        if type(Hxx) is numpy.ndarray:
            self.Hxx = [Hxx]
        elif type(Hxx[0]) is numpy.ndarray:
            self.Hxx = Hxx
        else:
            assert False, "Type of path cost Hxx matrix should be numpy.ndarray or list of numpy.ndarray, or None"

        if type(Huu) is numpy.ndarray:
            self.Huu = [Huu]
        elif type(Huu[0]) is numpy.ndarray:
            self.Huu = Huu
        else:
            assert False, "Type of path cost Huu matrix should be numpy.ndarray or list of numpy.ndarray, or None"

        if Hxu is not None:
            if type(Hxu) is numpy.ndarray:
                self.Hxu = [Hxu]
            elif type(Hxu[0]) is numpy.ndarray:
                self.Hxu = Hxu
            else:
                assert False, "Type of path cost Hxu matrix should be numpy.ndarray or list of numpy.ndarray, or None"
        else:
            self.Hxu = None

        if Hux is not None:
            if type(Hux) is numpy.ndarray:
                self.Hux = [Hux]
            elif type(Hux[0]) is numpy.ndarray:
                self.Hux = Hux
            else:
                assert False, "Type of path cost Hux matrix should be numpy.ndarray or list of numpy.ndarray, or None"
        else:
            self.Hux = None

        if Hxe is not None:
            if type(Hxe) is numpy.ndarray:
                self.Hxe = [Hxe]
            elif type(Hxe[0]) is numpy.ndarray:
                self.Hxe = Hxe
            else:
                assert False, "Type of path cost Hxe matrix should be numpy.ndarray or list of numpy.ndarray, or None"
        else:
            self.Hxe = None

        if Hue is not None:
            if type(Hue) is numpy.ndarray:
                self.Hue = [Hue]
            elif type(Hue[0]) is numpy.ndarray:
                self.Hue = Hue
            else:
                assert False, "Type of path cost Hue matrix should be numpy.ndarray or list of numpy.ndarray, or None"
        else:
            self.Hue = None

    def setFinalCost(self, hxx, hxe=None):

        if type(hxx) is numpy.ndarray:
            self.hxx = [hxx]
        elif type(hxx[0]) is numpy.ndarray:
            self.hxx = hxx
        else:
            assert False, "Type of final cost hxx matrix should be numpy.ndarray or list of numpy.ndarray"

        if hxe is not None:
            if type(hxe) is numpy.ndarray:
                self.hxe = [hxe]
            elif type(hxe[0]) is numpy.ndarray:
                self.hxe = hxe
            else:
                assert False, "Type of final cost hxe matrix should be numpy.ndarray, list of numpy.ndarray, or None"
        else:
            self.hxe = None

    def lqrSolver(self, ini_state, horizon):

        # Data pre-processing
        n_state = numpy.size(self.dynF[0], 1)
        if type(ini_state) is list:
            self.ini_x = numpy.array(ini_state, numpy.float64)
            if self.ini_x.ndim == 2:
                self.n_batch = numpy.size(self.ini_x, 1)
            else:
                self.n_batch = 1
                self.ini_x = self.ini_x.reshape(n_state, -1)
        elif type(ini_state) is numpy.ndarray:
            self.ini_x = ini_state
            if self.ini_x.ndim == 2:
                self.n_batch = numpy.size(self.ini_x, 1)
            else:
                self.n_batch = 1
                self.ini_x = self.ini_x.reshape(n_state, -1)
        else:
            assert False, "Initial state should be of numpy.ndarray type or list!"

        self.horizon = horizon

        if self.dynE is not None:
            assert self.n_batch == numpy.size(self.dynE[0],
                                              1), "Number of data batch is not consistent with column of dynE"

        # Check the time horizon
        if len(self.dynF) > 1 and len(self.dynF) != self.horizon:
            assert False, "time-varying dynF is not consistent with given horizon"
        elif len(self.dynF) == 1:
            F = self.horizon * self.dynF
        else:
            F = self.dynF

        if len(self.dynG) > 1 and len(self.dynG) != self.horizon:
            assert False, "time-varying dynG is not consistent with given horizon"
        elif len(self.dynG) == 1:
            G = self.horizon * self.dynG
        else:
            G = self.dynG

        if self.dynE is not None:
            if len(self.dynE) > 1 and len(self.dynE) != self.horizon:
                assert False, "time-varying dynE is not consistent with given horizon"
            elif len(self.dynE) == 1:
                E = self.horizon * self.dynE
            else:
                E = self.dynE
        else:
            E = self.horizon * [numpy.zeros(self.ini_x.shape)]

        if len(self.Hxx) > 1 and len(self.Hxx) != self.horizon:
            assert False, "time-varying Hxx is not consistent with given horizon"
        elif len(self.Hxx) == 1:
            Hxx = self.horizon * self.Hxx
        else:
            Hxx = self.Hxx

        if len(self.Huu) > 1 and len(self.Huu) != self.horizon:
            assert False, "time-varying Huu is not consistent with given horizon"
        elif len(self.Huu) == 1:
            Huu = self.horizon * self.Huu
        else:
            Huu = self.Huu

        hxx = self.hxx

        if self.hxe is None:
            hxe = [numpy.zeros(self.ini_x.shape)]

        if self.Hxu is None:
            Hxu = self.horizon * [numpy.zeros((self.n_state, self.n_control))]
        else:
            if len(self.Hxu) > 1 and len(self.Hxu) != self.horizon:
                assert False, "time-varying Hxu is not consistent with given horizon"
            elif len(self.Hxu) == 1:
                Hxu = self.horizon * self.Hxu
            else:
                Hxu = self.Hxu

        if self.Hux is None:  # Hux is the transpose of Hxu
            Hux = self.horizon * [numpy.zeros((self.n_control, self.n_state))]
        else:
            if len(self.Hux) > 1 and len(self.Hux) != self.horizon:
                assert False, "time-varying Hux is not consistent with given horizon"
            elif len(self.Hux) == 1:
                Hux = self.horizon * self.Hux
            else:
                Hux = self.Hux

        if self.Hxe is None:
            Hxe = self.horizon * [numpy.zeros((self.n_state, self.n_batch))]
        else:
            if len(self.Hxe) > 1 and len(self.Hxe) != self.horizon:
                assert False, "time-varying Hxe is not consistent with given horizon"
            elif len(self.Hxe) == 1:
                Hxe = self.horizon * self.Hxe
            else:
                Hxe = self.Hxe

        if self.Hue is None:
            Hue = self.horizon * [numpy.zeros((self.n_control, self.n_batch))]
        else:
            if len(self.Hue) > 1 and len(self.Hue) != self.horizon:
                assert False, "time-varying Hue is not consistent with given horizon"
            elif len(self.Hue) == 1:
                Hue = self.horizon * self.Hue
            else:
                Hue = self.Hue

        # Solve the Riccati equations: the notations used here are consistent with Lemma 4.2 in the PDP paper
        I = numpy.eye(self.n_state)
        PP = self.horizon * [numpy.zeros((self.n_state, self.n_state))]
        WW = self.horizon * [numpy.zeros((self.n_state, self.n_batch))]
        PP[-1] = self.hxx[0]
        WW[-1] = self.hxe[0]
        for t in range(self.horizon - 1, 0, -1):
            P_next = PP[t]
            W_next = WW[t]
            invHuu = numpy.linalg.inv(Huu[t])
            GinvHuu = numpy.matmul(G[t], invHuu)
            HxuinvHuu = numpy.matmul(Hxu[t], invHuu)
            A_t = F[t] - numpy.matmul(GinvHuu, numpy.transpose(Hxu[t]))
            R_t = numpy.matmul(GinvHuu, numpy.transpose(G[t]))
            M_t = E[t] - numpy.matmul(GinvHuu, Hue[t])
            Q_t = Hxx[t] - numpy.matmul(HxuinvHuu, numpy.transpose(Hxu[t]))
            N_t = Hxe[t] - numpy.matmul(HxuinvHuu, Hue[t])

            temp_mat = numpy.matmul(numpy.transpose(A_t), numpy.linalg.inv(I + numpy.matmul(P_next, R_t)))
            P_curr = Q_t + numpy.matmul(temp_mat, numpy.matmul(P_next, A_t))
            W_curr = N_t + numpy.matmul(temp_mat, W_next + numpy.matmul(P_next, M_t))

            PP[t - 1] = P_curr
            WW[t - 1] = W_curr

        # Compute the trajectory using the Raccti matrices obtained from the above: the notations used here are
        # consistent with the PDP paper in Lemma 4.2
        state_traj_opt = (self.horizon + 1) * [numpy.zeros((self.n_state, self.n_batch))]
        control_traj_opt = (self.horizon) * [numpy.zeros((self.n_control, self.n_batch))]
        costate_traj_opt = (self.horizon) * [numpy.zeros((self.n_state, self.n_batch))]
        state_traj_opt[0] = self.ini_x
        for t in range(self.horizon):
            P_next = PP[t]
            W_next = WW[t]
            invHuu = numpy.linalg.inv(Huu[t])
            GinvHuu = numpy.matmul(G[t], invHuu)
            A_t = F[t] - numpy.matmul(GinvHuu, numpy.transpose(Hxu[t]))
            M_t = E[t] - numpy.matmul(GinvHuu, Hue[t])
            R_t = numpy.matmul(GinvHuu, numpy.transpose(G[t]))

            x_t = state_traj_opt[t]
            u_t = -numpy.matmul(invHuu, numpy.matmul(numpy.transpose(Hxu[t]), x_t) + Hue[t]) \
                  - numpy.linalg.multi_dot([invHuu, numpy.transpose(G[t]), numpy.linalg.inv(I + numpy.dot(P_next, R_t)),
                                            (numpy.matmul(numpy.matmul(P_next, A_t), x_t) + numpy.matmul(P_next,
                                                                                                         M_t) + W_next)])

            x_next = numpy.matmul(F[t], x_t) + numpy.matmul(G[t], u_t) + E[t]
            lambda_next = numpy.matmul(P_next, x_next) + W_next

            state_traj_opt[t + 1] = x_next
            control_traj_opt[t] = u_t
            costate_traj_opt[t] = lambda_next
        time = [k for k in range(self.horizon + 1)]

        opt_sol = {'state_traj_opt': state_traj_opt,
                   'control_traj_opt': control_traj_opt,
                   'costate_traj_opt': costate_traj_opt,
                   'time': time}
        return opt_sol
