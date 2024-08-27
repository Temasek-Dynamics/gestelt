import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import scipy.linalg

from helper import *
from params import *
from sys_dynamics import SysDyn

class AcadosCustomOcp:
  def __init__(self, track_mngr):

    self.nx = 0
    self.nu = 0
    self.ny = 0
    self.ns = 0

    self.ocp = None         # AcadosOcp()
    self.solver = None      # AcadosOcpSolver()
    self.integrator = None  # AcadosSimSolver()
    self.sysModel = None    # System dynamics model

    self.zeta_0 = zeta_0_default  # Initial state
    self.zeta_N = None  # States for intermediate shooting nodes
    self.u_N = None     # Control inputs for intermediate shooting nodes

    self.track_mngr = track_mngr

  def setInitialCondition(self, zeta_0):
    """Set initial condition
    """
    self.zeta_0 = zeta_0 
    self.zeta_N = ca.repmat(np.reshape(zeta_0, (self.nx, 1)), 1, N+1 )

    self.ocp.constraints.x0 = zeta_0

  def setupAcadosOCP(self):
    """Formulate the OCP 
    """

    # Create CASADI symbolic expressions
    self.sysModel = SysDyn()

    zeta_f, dyn_f, u, proj_constr, dyn_fn = self.sysModel.setupODE(self.track_mngr)

    #####
    # Create acados model
    #####
    ocp = AcadosOcp()
    model_ac = AcadosModel()
    model_ac.f_expl_expr = dyn_f  # explict expression
    model_ac.x = zeta_f           # state variables
    model_ac.u = u                # control inputs
    model_ac.name = "quadrotor"
    ocp.model = model_ac

    #####
    # Set dimensions
    #####
    ocp.dims.N = N
    self.nx = model_ac.x.size()[0]
    self.nu = model_ac.u.size()[0]
    # ny = self.nx + self.nu 

    self.zeta_N = ca.repmat(np.reshape(self.zeta_0, (self.nx, 1)), 1, N+1 )
    self.u_N = ca.repmat(U_REF, 1, N)

    #####
    # continuity on constraints
    #####
    ocp.constraints.x0 = self.zeta_0

    # formulate cost function
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(model_ac.x, model_ac.u)
    # yref:  reference at intermediate shooting nodes (1 to N-1).
    ocp.cost.yref = np.array([ 1.0, 0, 0,   # s, n, b
                                1, 0, 0, 0, # Quaternion
                                0, 0, 0,    # sDot, nDot, bDot
                                0, 0, 0,    # (angular vel) wr, wp, wy
                                0, 0, 0,    # (linear vel) vx, vy, vz
                              U_HOV, U_HOV, U_HOV, U_HOV, # ohm1, ohm2, ohm3, ohm4
                              0, 0, 0, 0])  # (control input) alpha1, alpha2, alpha3, alpha4
    ocp.cost.W = scipy.linalg.block_diag(Q, R)

    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr_e = model_ac.x
    # yref_e:  reference at terminal shooting node (N).
    ocp.cost.yref_e = np.array([ 1.0, 0, 0,
                                  1, 0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  U_HOV, U_HOV, U_HOV, U_HOV])
    ocp.cost.W_e = Qn

    #####
    # formulate inquality constraints
    #####
    # constrain AGV dynamics : acceleration, angular velocity (convex ?, Non-linear)
    dyn_constr_eqn = []
    ineq_constr_eqn = ca.vertcat(dyn_constr_eqn,
                              proj_constr)

    # con_h_expr: Nonlinear constraints on the initial shooting node
    # lh <= h(x, u) <= uh
    model_ac.con_h_expr = ineq_constr_eqn
    model_ac.con_h_expr_e = ineq_constr_eqn

    # inequality bounds
    nh = model_ac.con_h_expr.shape[0] # Number of rows in h(x, u)

    # constrain controls
    # lbu = [0] * self.nu;      ubu = [0] * self.nu

    # # Control bounds ( Affects horizon quality before switch)
    # lbu[0] = OHM_MIN;         ubu[0] = OHM_MAX
    # lbu[1] = OHM_MIN;         ubu[1] = OHM_MAX

    # ocp.constraints.lbu = np.array(lbu)
    # ocp.constraints.ubu = np.array(ubu)
    # ocp.constraints.idxbu = np.array([0, 1])

    # Bounds on path constraints (inequality)
    lh = np.zeros(nh);        uh = np.zeros(nh)
    lh[:] = -INF;             uh[:] = 1

    ocp.constraints.lh = lh
    ocp.constraints.uh = uh

    ocp.constraints.lh_e = lh
    ocp.constraints.uh_e = uh

    #####
    # configure itegrator and QP solver
    #####


    # configure itegrator and QP solver
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.tf = Tf
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    # ocp.solver_options.collocation_type = 'GAUSS_RADAU_IIA'
    # ocp.solver_options.time_steps = time_steps
    # ocp.solver_options.shooting_nodes = shooting_nodes

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"#"PARTIAL_CONDENSING_HPIPM" #"FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx =  "GAUSS_NEWTON"#"EXACT",
    # ocp.solver_options.cost_discretization ="INTEGRATOR"
    ocp.solver_options.qp_solver_cond_N = int(N/2)
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tol = 1e-3
    ocp.qp_solver_tol = 1e-3

    # create solver
    solve_json = "planner_ocp.json"
    self.ocp = ocp
    self.solver = AcadosOcpSolver(ocp, json_file = solve_json)
    self.integrator = AcadosSimSolver(ocp, json_file = solve_json) 

    return True

  def solve_and_sim(self):
    """Solve the OCP with multiple shooting, and forward simulate with RK4
    """

    # status indication
    #   # 0 – success
    #   # 1 – failure
    #   # 2 – maximum number of iterations reached
    #   # 3 – minimum step size in QP solver reached
    #   # 4 – qp solver failed

    # Integrate ODE model to get CL estimate (no measurement noise)
    u_0 = self.solver.solve_for_x0(self.zeta_0)
    self.zeta_0 = self.integrator.simulate( x = self.zeta_0,
                                            u = u_0)
    
    self.zeta_N = np.reshape(self.solver.get(0, "x"), (self.nx, 1)) 
    for i in range(1, N+1): # for each intermediate shooting node
      zeta_i = np.reshape(self.solver.get(i, "x"), (self.nx, 1))
      self.zeta_N = np.concatenate((self.zeta_N, zeta_i), axis = 1) # Concatenate vertically

    self.u_N[:, 0] = u_0  # Set first control input
    
  def cost_update_ref(self, zeta_0):
    """Update cost reference

    Args:
        zeta_0 (_type_): Initial condition

    Returns:
        bool: Indicates if path has ended
    """
    s0 = float(zeta_0[0])
    if self.track_mngr.isPathEnded(s0):
      return True # Path has ended
    
    sref = s0 + Tf  # Reference at end of time horizon
    # srefDot = S_DOT / Tf  # Rate of change of reference point
    srefDot = 1.0   # Rate of change of reference point
    for j in range(N):  # For each intermediate shooting node
      sref_j = s0 + (sref - s0) * j / N
      yref = np.array([sref_j, 0, 0, 
                        0, 0, 0, 0, 
                        srefDot, 0, 0, 
                        0, 0, 0, 
                        0, 0, 0, 
                        U_HOV, U_HOV, U_HOV, U_HOV, 
                        0, 0, 0, 0])
      self.solver.set(j, "yref", yref)

    # Terminal shooting node
    yref = np.array([sref_j, 0, 0, 
                     0, 0, 0, 0, 
                     srefDot, 0, 0, 
                     0, 0, 0, 
                     0, 0, 0, 
                     U_HOV, U_HOV, U_HOV, U_HOV])
    self.solver.set(N, "yref", yref)

  def get_cost(self) -> float:
    """Get current cost of the current solution.

    Returns:
        float: cost value of 
    """
    cost = self.solver.get_cost()
    return cost