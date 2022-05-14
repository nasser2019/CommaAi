#!/usr/bin/env python3
import os
import numpy as np

from casadi import SX, vertcat, sin, cos

from common.realtime import sec_since_boot
from selfdrive.controls.lib.drive_helpers import LAT_MPC_N as N
from selfdrive.modeld.constants import T_IDXS

if __name__ == '__main__':  # generating code
  from pyextra.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from selfdrive.car.body.body_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

LAT_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LAT_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LAT_MPC_DIR, "acados_ocp_body.json")
X_DIM = 4
#P_DIM = 2
MODEL_NAME = 'body'
ACADOS_SOLVER_TYPE = 'SQP_RTI'

def gen_body_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  theta = SX.sym('theta')
  theta_rate = SX.sym('theta_rate')
  model.x = vertcat(x_ego, v_ego, theta, theta_rate)

  # controls
  torque = SX.sym('torque')
  model.u = vertcat(torque)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  theta_ego_dot = SX.sym('theta_ego_dot')
  theta_rate_ego_dot = SX.sym('theta_rate_ego_dot')

  model.xdot = vertcat(x_ego_dot, v_ego_dot, theta_ego_dot, theta_rate_ego_dot)

  # dynamics model
  g = 9.81  # gravity
  mt = 8.25  # total mass
  mw = 1.162  # wheel mass
  mb = mt - 2*mw  # non-wheel mass
  r = .163/2  # wheel radius
  cog_l = .242 - r  # cog above axle
  J = 0.5 * mw * (r**2)  # Mass Moment of Inertia of wheel, modeled as solid cylinder
  I2 = 0.5 * mw * (2 * cog_l**2)  # Mass Moment of Inertia of pendulum, modeled as hollow cylinder with wall radius cog_l
  a = mb * cog_l
  Io = I2 + mb * cog_l**2
  mo = mt + J / (r**2)
  d1 = Io * mo - (a**2) * cos(theta)**2
  f_expl = vertcat(v_ego,
                   (1/d1) * (a * Io * (theta_rate ** 2) * sin(theta)
                             - (a**2) * g * sin(theta) * cos(theta)
                             + torque * (Io/r + a * cos(theta))),
                   theta_rate,# torque)
                   (1/d1) * (- (a**2) * (theta_rate ** 2) * sin(theta) * cos(theta)
                             + a * mo * g * sin(theta)
                             - torque * (mo + (a/r) * cos(theta))))
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_body_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_body_model()

  Tf = np.array(T_IDXS)[N]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  Q = np.diag([0.0, 0.0])
  QR = np.diag([0.0, 0.0, 0.0])

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  v_ego, theta = ocp.model.x[1], ocp.model.x[2]
  torque= ocp.model.u[0]

  #ocp.parameter_values = np.zeros((P_DIM, ))

  ocp.cost.yref = np.zeros((3, ))
  ocp.cost.yref_e = np.zeros((2, ))
  # TODO hacky weights to keep behavior the same
  ocp.model.cost_y_expr = vertcat(v_ego, theta, torque)
  ocp.model.cost_y_expr_e = vertcat(v_ego, theta)

  # set constraints
  ocp.constraints.constr_type = 'BGH'
  #ocp.constraints.idxbx = np.array([2,3])
  #ocp.constraints.ubx = np.array([np.radians(90), np.radians(50)])
  #ocp.constraints.lbx = np.array([-np.radians(90), -np.radians(50)])
  x0 = np.zeros((X_DIM,))
  ocp.constraints.x0 = x0

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_solver_cond_N = 10

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = np.array(T_IDXS)[:N+1]

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class BodyMpc():
  def __init__(self, x0=np.zeros(X_DIM)):
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset(x0)

  def reset(self, x0=np.zeros(X_DIM)):
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.yref = np.zeros((N+1, 3))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:2])

    # Somehow needed for stable init
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.solver.constraints_set(0, "lbx", x0)
    self.solver.constraints_set(0, "ubx", x0)
    self.solver.solve()
    self.solution_status = 0
    self.solve_time = 0.0
    self.cost = 0

  def set_weights(self, path_weight, heading_weight, steer_rate_weight):
    W = np.asfortranarray(np.diag([1.0, 1.0, 1.0]))
    for i in range(N):
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', W[:2,:2])

  def run(self, x0):
    x0_cp = np.copy(x0)
    #p_cp = np.copy(p)
    self.solver.constraints_set(0, "lbx", x0_cp)
    self.solver.constraints_set(0, "ubx", x0_cp)
    self.yref[:,0] = 0.0
    self.yref[:,1] = 0.0
    self.yref[:,2] = 0.0
    #v_ego = p_cp[0]
    # rotation_radius = p_cp[1]
    #self.yref[:,1] = heading_pts*(v_ego+5.0)
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
      #self.solver.set(i, "p", p_cp)
    #self.solver.set(N, "p", p_cp)
    self.solver.cost_set(N, "yref", self.yref[N][:2])

    t = sec_since_boot()
    self.solution_status = self.solver.solve()
    self.solve_time = sec_since_boot() - t

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')
    self.cost = self.solver.get_cost()


if __name__ == "__main__":
  ocp = gen_body_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
