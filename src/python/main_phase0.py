from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from casadi import SX, vertcat, sin, cos, Function
import numpy as np
import scipy.linalg
from CarModelPhase0 import CarModelPhase0
import time
import matplotlib.pyplot as plt
import datetime
import os
from utils_phase0 import *
from tqdm import tqdm

Tf = 0.5  # prediction horizon
N = round(Tf*500)  # number of discretization steps
T = 5.0  # maximum simulation time[s]

mb = 4
mw = 2
Rw = 0.17
Iw = (mw*(Rw**2))

car_model = CarModelPhase0(mb, mw, Iw, Rw, 0.5, Tf/N)
model = car_model.model

ocp = AcadosOcp()
ocp.model = model
print(model)

# set dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

ocp.dims.N = N

# set cost
Q = np.diag([ 0, 100, 1, 0])*100

R = np.diag([0.01])
Qe = np.diag([ 0, 100, 1, 0])*100

ocp.cost.cost_type = "LINEAR_LS"
ocp.cost.cost_type_e = "LINEAR_LS"
unscale = N / Tf

ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
ocp.cost.W_e = Qe / unscale

Vx = np.zeros((ny, nx))
Vx[:nx, :nx] = np.eye(nx)
ocp.cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[-nu:-nu+1, :] = np.eye(nu)
ocp.cost.Vu = Vu

Vx_e = np.zeros((ny_e, nx))
Vx_e[:nx, :nx] = np.eye(nx)
ocp.cost.Vx_e = Vx_e

# set intial references
target_phase1 = np.array([0.0, 1e15, 0, 0, -20]) # target standing position

yref = target_phase1
yref_e = target_phase1[:4]
ocp.cost.yref = yref
ocp.cost.yref_e = yref_e

# setting constraints
lower_X = np.array([-1e15, 0, -1e15, -1e15])
upper_X = np.array([1e15, 1e15, 1e15, 1e15])

ocp.constraints.lbx = lower_X
ocp.constraints.ubx = upper_X
ocp.constraints.idxbx = np.array([0, 1, 2, 3])

lower_U = np.array([-10])
ocp.constraints.lbu = lower_U

upper_U = np.array([10])
ocp.constraints.ubu = upper_U
ocp.constraints.idxbu = np.array([0])

#  Set 
# print(f"Const shape: {model.con_h_expr.shape}")
model.con_h_expr = None

# lower_ground = np.array([0, 0])
# upper_ground = np.array([1e15, 0])

# lower_flying = np.array([-1e15, -1e15])
# upper_flying = np.array([1e15, 1e15])

# ocp.constraints.lh = lower_ground
# ocp.constraints.uh = upper_ground

# set intial condition
v0 = 7
x0 = np.array([-2/Rw, 0.0, v0/Rw, 0.0])
ocp.constraints.x0 = x0

# set QP solver and integration
ocp.solver_options.tf = Tf
ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
#ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
ocp.solver_options.nlp_solver_type = "SQP_RTI"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
ocp.solver_options.integrator_type = "DISCRETE"
ocp.solver_options.sim_method_num_stages = 4
ocp.solver_options.sim_method_num_steps = 1
ocp.solver_options.qp_solver_iter_max = 1000
ocp.solver_options.nlp_solver_max_iter = 1000
#ocp.solver_options.nlp_solver_tol_eq = 1e-3
#ocp.solver_options.nlp_solver_tol_ineq = 1e-3
#ocp.solver_options.qp_solver_tol_eq = 1e-3
#ocp.solver_options.qp_solver_tol_ineq = 1e-3

# create solver
acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

# Create log file
time_now = datetime.datetime.now()
folder = time_now.strftime("%Y_%m_%d_%H:%M:%S")
os.mkdir('results/' + folder)
with open('results/'+folder+'/data.txt', 'w') as f:
    print(f"# {os.getcwd().split('/')[-1]}", file=f)
    print(f'Tf = {Tf}', file=f)
    print(f'x0 = {x0}', file=f)
    print(f'Q = {Q}', file=f)
    print(f'R = {R}', file=f)
    print(f'Qe = {Qe}', file=f)
    print(f'qp_solver = {ocp.solver_options.qp_solver}', file=f)
    print(f'nlp_solver_type = {ocp.solver_options.nlp_solver_type}', file=f)
    print(f'qp_solver_iter_max = {ocp.solver_options.qp_solver_iter_max}', file=f)
    print(f'nlp_solver_max_iter = {ocp.solver_options.nlp_solver_max_iter}', file=f)

Nsim = round(T * N / Tf)
# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
simX_horizon = np.ndarray((Nsim, N, nx))

tcomp_sum = 0
tcomp_max = 0
time_iterations = np.zeros(Nsim)
cost_integral = 0

# simulate
for i in tqdm(range(Nsim)):

    actual_time = i* (T/Nsim)

    # update reference
    for j in range(N):
        acados_solver.set(j, "yref", yref)
        step_time = actual_time + j*(T/Nsim)
        acados_solver.set(j, 'y_ref', target_phase1)
        
    step_time = actual_time + N*(T/Nsim)
    acados_solver.set(N, 'y_ref', target_phase1[:4])
        
    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    #print(acados_solver.get_residuals())
    #acados_solver.print_statistics()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    elapsed = time.time() - t
    time_iterations[i] = elapsed

    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed
    
    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")
    
    cost_integral += np.matmul(u0, u0.T)*Tf / N
    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]
    for j in range(N):
        simX_horizon[i, j, :] = acados_solver.get(j, 'x')

    
    # update initial condition
    x0 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)


with open('results/'+folder+'/data.txt', 'a') as f:
    print(f'min_time = {np.min(time_iterations)}', file=f)
    print(f'max_time = {np.max(time_iterations)}', file=f)
    print(f'mean_time = {np.mean(time_iterations)}', file=f)
    print(f'std_time = {np.std(time_iterations)}', file=f)
    print(f'cost integral = {cost_integral}', file=f)

max_index = np.argmin(np.abs(simX[:,2]))
print(simX[:,2])
print(max_index)
stop_state = simX[max_index]
simX = simX[:max_index]
simU = simU[:max_index]
t = np.linspace(0.0, Nsim * Tf / N, Nsim)
t = t[:max_index]

print(t.shape)
print(simX.shape)
plotRes(simX,t)

plt.savefig('results/' + folder + "/state_plots.eps")
plt.savefig('results/' + folder + "/state_plots.png", dpi=300)

plotU(simU,t)
plt.savefig('results/' + folder + "/control_plots.eps")
plt.savefig('results/' + folder + "/control_plots.png", dpi=300)

#plt.show()
with open('results/' + folder + "/simX.npy", 'wb') as f:
    np.save(f, simX)
with open('results/' + folder + "/simU.npy", 'wb') as f:
    np.save(f, simU)
with open('results/' + folder + "/simX_horizon.npy", 'wb') as f:
    np.save(f, simX_horizon)
with open('results/' + folder + "/t.npy", 'wb') as f:
    np.save(f, t)

ext_state = get_extended_state(simX, Rw)
# THIS IS A BIT SLOW
renderVideo(ext_state, simX_horizon, t, folder)

print(stop_state)
print(simX[-1])