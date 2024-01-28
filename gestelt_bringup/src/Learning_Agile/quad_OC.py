##this file is to obtain the optimal solution

from casadi import *
import numpy
from scipy import interpolate
import casadi
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_template import AcadosModel
import time
import scipy
class OCSys:

    def __init__(self, project_name="my optimal control system"):
        self.project_name = project_name

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

        #self.dyn = casadi.Function('f',[self.state, self.control],[f])
        self.dyn = self.state + dt * f
        self.dyn_fn = casadi.Function('dynamics', [self.state, self.control, self.auxvar], [self.dyn],['x','u','p'],['x_next'])
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

    def setthrustcost(self, thrust_cost):
        if not hasattr(self, 'auxvar'):
            self.setAuxvarVariable()

        assert thrust_cost.numel() == 1, "thrust_cost must be a scalar function"        

        self.thrust_cost = thrust_cost
        self.thrust_cost_fn = casadi.Function('thrust_cost',[self.control, self.auxvar], [self.thrust_cost])

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
    
    def setTraCost(self, tra_cost, t = 3.0):
        self.t = t
        self.tra_cost = tra_cost
        self.tra_cost_fn = casadi.Function('tra_cost', [self.state, self.auxvar], [self.tra_cost])


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
        P=MX.sym('P',self.n_state+self.n_control)

        # "Lift" initial conditions
        Xk = MX.sym('X0', self.n_state)
        w += [Xk]
        self.lbw += self.state_lb
        self.ubw +=  self.state_ub
        self.w0 += [0.5 * (x + y) for x, y in zip(self.state_lb, self.state_ub)]
        
        # current state and control constraints
        # g for multiple shooting constraints
        g += [Xk-P[0:self.n_state]]
        self.lbg += self.n_state * [0]
        self.ubg += self.n_state * [0]

        Ulast=P[self.n_state:]
        
        # Formulate the NLP
        for k in range(int(self.horizon)):
            # New NLP variable for the control
            Uk = MX.sym('U_' + str(k), self.n_control) # 4 control variables
            
            w += [Uk]

            # control constraints
            self.lbw += self.control_lb
            self.ubw += self.control_ub
            self.w0 += [0.5 * (x + y) for x, y in zip(self.control_lb, self.control_ub)]

            #calculate weight
            # weight = 60*casadi.exp(-10*(dt*k-self.t)**2) #gamma should increase as the flight duration decreases
             
            # Integrate till the end of the interval
            Xnext = self.dyn_fn(Xk, Uk,auxvar_value)

            # weight*self.tra_cost_fn(Xk, auxvar_value) + 
            Ck = self.path_cost_fn(Xk, auxvar_value)\
                +self.thrust_cost_fn(Uk, auxvar_value) + 1*dot(Uk-Ulast,Uk-Ulast)
                                                               
            J = J + Ck

            # New NLP variable for state at end of interval
            Xk = MX.sym('X_' + str(k + 1), self.n_state)
            w += [Xk]
            self.lbw += self.state_lb
            self.ubw += self.state_ub
            self.w0 += [0.5 * (x + y) for x, y in zip(self.state_lb, self.state_ub)]
            Ulast = Uk

            # Add equality constraint, multiple shooting
            g += [Xnext - Xk]
            self.lbg += self.n_state * [0]
            self.ubg += self.n_state * [0]

        # Adding the final cost
        J = J + self.final_cost_fn(Xk, auxvar_value)

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
        self.solver = nlpsol('solver', 'ipopt', prob, opts)
        
        
    def ocSolver(self, current_state_control, auxvar_value=1, costate_option=0):
        # ------------------------------------------ use the solver -------------------------------------------##
        # 2. use the solver to Solve the NLP

        # self.w0: The initial state and control( initial state means current drone state)
        t_ = time.time()
        sol = self.solver(x0=self.w0,
                     lbx=self.lbw, 
                     ubx=self.ubw, 
                     p=current_state_control, 
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
                            w_tra_p=1,
                            w_tra_q=1,
                            w_thrust=1,
                            w_final_p=1,
                            w_final_v=1,
                            w_final_q=1,
                            w_final_w=1,
                            gazebo_sim=False
                            ):
        """
        This function is to define the optimal control problem using ACADOS
        """
       
        
        assert hasattr(self, 'state'), "Define the state variable first!"
        assert hasattr(self, 'control'), "Define the control variable first!"
        assert hasattr(self, 'dyn'), "Define the system dynamics first!"
        assert hasattr(self, 'path_cost'), "Define the running cost function first!"
        assert hasattr(self, 'final_cost'), "Define the final cost function first!"
        # Start with an empty NLP
        self.horizon=horizon

        # predict horizon in seconds
        self.T=horizon*dt
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

        # explicit model
        model.f_expl_expr=self.dyn_fn(self.state,self.control,self.auxvar)

        # implicit model
        x_dot=casadi.SX.sym('x_dot',self.n_state)
        model.f_impl_expr=x_dot-self.dyn_fn(self.state,self.control,self.auxvar)

        model.x=self.state
        model.xdot=x_dot
        model.u=self.control
        
        # Ulast
        P=casadi.SX.sym('p',self.n_state+self.n_control)
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

        self.n_stats_input=self.n_state+self.n_control


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
        ocp.solver_options.tf = self.T # horizon length T in seconds
        ocp.dims.np = self.n_state+self.n_control      # number of parameters for solver input, here is the current state and control
        ocp.parameter_values = np.zeros(self.n_state+self.n_control) # initial guess for the current state and control



        ############################################################### 
        ##------------ setting the cost function---------------------##
        ############################################################### 

         #---------------------for linear cost---------------------##
        # setting the cost function weight
        # Q_tra_p = np.diag([1, 1, 1])*w_tra_p
        # Q_tra_q = np.diag([1, 1, 1])*w_tra_q

        # Q_final_p = np.diag([1, 1, 1])*w_final_p#*0.1
        # Q_final_v = np.diag([1, 1, 1])*w_final_v#*0.1#*10
        # Q_final_q = np.diag([1, 1, 1, 1])*w_final_q#*0
        # Q_final_w = np.diag([1, 1, 1])*w_final_w#*10

        # R_thrust = np.diag([1, 1, 1, 1])*w_thrust

        # Q_state = scipy.linalg.block_diag(Q_final_p, Q_final_v, Q_final_q, Q_final_w)
        # ocp.cost.W = scipy.linalg.block_diag(Q_final_p, Q_final_v, Q_final_q, Q_final_w, R_thrust)
        # ocp.cost.W_e = scipy.linalg.block_diag(Q_final_p, Q_final_v, Q_final_q, Q_final_w)

        # ocp.cost.cost_type = 'LINEAR_LS'
        # ocp.cost.cost_type_e = 'LINEAR_LS'

        # # #mapping from x,u to y
        # ocp.cost.Vx = np.zeros((self.n_state+self.n_control, self.n_state))
        # ocp.cost.Vx[:self.n_state, :self.n_state] = np.eye(self.n_state)
        
        # ocp.cost.Vu = np.zeros((self.n_state+self.n_control, self.n_control))
        # ocp.cost.Vu[-self.n_control:,-self.n_control:] = np.eye(self.n_control)
        
        # ocp.cost.Vx_e = np.eye(self.n_state)

       
        # initial constraints, and desired values, will be updated later
        # x_ref= np.zeros((self.n_state))
        # u_ref= np.zeros((self.n_control))

        # ### 0--N-1
        # ocp.cost.yref = np.concatenate((x_ref, u_ref))
        # ### N
        # ocp.cost.yref_e = x_ref


        #-------------------------external cost-------------------------##
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        Ulast=ocp.model.p[self.n_state:]
        goal_state=ocp.model.p[0:self.n_state]
        
        # setting the cost function
        ocp.model.cost_expr_ext_cost = self.path_cost_fn(ocp.model.x,self.auxvar)\
            +self.thrust_cost_fn(ocp.model.u,self.auxvar)\
            +100*dot(ocp.model.u-Ulast,ocp.model.u-Ulast)
        ocp.model.cost_expr_ext_cost_e = self.path_cost_fn(ocp.model.x,self.auxvar)

        ############################################################### 
        ##----------------- setting the constraints -----------------##
        ###############################################################  

           
        ##------------------control constraints----------------------##
        # # will set this initial value for all N nodes states
        x_init = np.zeros((self.n_state))
        x_init[3] = 1.0
        ocp.constraints.x0 = x_init

        # 4x1
        ocp.constraints.lbu = np.array(self.control_lb) 
        ocp.constraints.ubu = np.array(self.control_ub)
        ocp.constraints.idxbu = np.array([i for i in range(self.n_control)])
        
        
        ##------------------ state constraints ----------------------##
        # # constraint for position ( no constraints for the state)
        ocp.constraints.lbx = np.array([])#(self.state_lb) #([])#
        ocp.constraints.ubx = np.array([])#(self.state_ub) #([])#
        ocp.constraints.idxbx = np.array([])#([i for i in range(self.n_state)]) #([])#i for i in range(self.n_state)]
        



        ##------------------ setting the solver ------------------##
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # GAUSS_NEWTON, EXACT
        ocp.solver_options.integrator_type = 'ERK' # ERK (explicit Runge-Kutta integrator) or IRK (Implicit Runge-Kutta integrator)
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI or SQP
        ocp.solver_options.sim_method_num_steps = 3
        # ocp.solver_options.nlp_solver_max_iter = 200
        ##------------------ setting the code generation ------------------##
        # compile acados ocp
        json_file = os.path.join('./'+model.name+'_acados_ocp.json')

        # load solver from json file
        build=True
        generate=True
        if gazebo_sim:
            build=False
            generate=False
        self.acados_solver = AcadosOcpSolver(ocp,generate=generate,build=build, json_file=json_file)

    def AcadosOcSolver(self, current_state_control, goal_pos,auxvar_value=1, costate_option=0):
        """
        This function is to solve the optimal control problem using ACADOS
        """
 
        #---------------------for linear cost---------------------##
        # #set desired ref state
        desired_goal_vel=np.array([0, 0, 0])
        desired_goal_ori = np.array([1, 0, 0, 0])
        desired_goal_w=np.array([0, 0, 0])
        desired_thrust = np.ones(self.n_control)*0.0
        
        goal_state=np.concatenate((np.array(goal_pos),desired_goal_vel,desired_goal_ori,desired_goal_w))
        goal_state_middle=np.concatenate((np.array(goal_pos),desired_goal_vel,desired_goal_ori,desired_goal_w,desired_thrust ))
        
        
        # set the desired state at the N-th(final) node (only x, no u)
        # self.acados_solver.set(self.n_nodes, "yref",goal_state)
        

        # set the desired state-control at 0->N-1 nodes
        for i in range(self.n_nodes):
            # self.acados_solver.set(i, "yref",goal_state_middle)
            # self.acados_solver.set(i, "yref_e",goal_state)
            
            # set the desired goal and current input
            self.acados_solver.set(i, "p",np.concatenate((goal_state,np.array(current_state_control[self.n_state:]))))
        
        # set initial condition aligned with the current state
        self.acados_solver.set(0, "lbx", np.array(current_state_control[0:self.n_state]))
        self.acados_solver.set(0, "ubx", np.array(current_state_control[0:self.n_state]))
        
       

        # solve ocp
        status = self.acados_solver.solve()

        if status != 0:
            raise Exception('acados returned status {}. Exiting.'.format(status))
        sol_u=self.acados_solver.get(0, "u")
        sol_x=self.acados_solver.get(1, "x")

        w_opt = sol_x

        # take the optimal control and state
        # sol_traj = numpy.concatenate((w_opt, self.n_control * [0]))
        # sol_traj = numpy.reshape(sol_traj, (-1, self.n_state + self.n_control))
        # state_traj_opt = sol_traj[:, 0:self.n_state]
        # control_traj_opt = numpy.delete(sol_traj[:, self.n_state:], -1, 0)
        # time = numpy.array([k for k in range(self.horizon + 1)])
        state_traj_opt = w_opt.reshape(-1, self.n_state)
        control_traj_opt = sol_u.reshape(-1, self.n_control)
        # output
        opt_sol = {"state_traj_opt": state_traj_opt,
                   "control_traj_opt": control_traj_opt,
                   'auxvar_value': auxvar_value,
                   "time": time,
                   "horizon": self.horizon}
                   #"cost": sol['f'].full()}

        return opt_sol 
    

    def sys_dynamics(self, dt):
        M = 4       # refinement
        DT = dt/M
        X0 = casadi.SX.sym("X", self._s_dim)
        U = casadi.SX.sym("U", self._u_dim)
        # #
        X = X0
        for _ in range(M):
            # --------- RK4------------
            k1 =DT*self.f(X, U)
            k2 =DT*self.f(X+0.5*k1, U)
            k3 =DT*self.f(X+0.5*k2, U)
            k4 =DT*self.f(X+k3, U)
            #
            X = X + (k1 + 2*k2 + 2*k3 + k4)/6        
        # Fold
        F = casadi.Function('F', [X0, U], [X])
        return F


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