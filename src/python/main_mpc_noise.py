from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from casadi import SX, vertcat, sin, cos, Function
import numpy as np
import scipy.linalg
from CarModel_mpc_noise import CarModel_mpc_noise
import time
import matplotlib.pyplot as plt
import datetime
import os
from utils import *
from tqdm import tqdm

Tf = 1.5  # prediction horizon
N = round(Tf*20)  # number of discretization steps
T = 20.0  # maximum simulation time[s]

mb = 4
mw = 2
mt = mb+mw
Rw = 0.17
Iw = (mw*(Rw**2))
g = 9.81

car_model = CarModel_mpc_noise(mb, mw, Iw, Rw, Tf/N)
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
Q =  np.diag([ 1, 1, 10, 0, 0, 0])*100
R = np.diag([1, 1])
Qe = np.diag([ 1, 1, 10, 0, 0, 0])*100


ocp.cost.cost_type = "LINEAR_LS"
ocp.cost.cost_type_e = "LINEAR_LS"
unscale = N / Tf

ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
ocp.cost.W_e = Qe / unscale

Vx = np.zeros((ny, nx))
Vx[:nx, :nx] = np.eye(nx)
ocp.cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[-nu:] = np.eye(nu)
ocp.cost.Vu = Vu

Vx_e = np.zeros((ny_e, nx))
Vx_e[:nx, :nx] = np.eye(nx)
ocp.cost.Vx_e = Vx_e

# set intial references
#target_position = np.array([0., 0.8, 0, 0, 0, 0]) # target standing position
target_position = np.array([-2./Rw, 0.8, 0, 0, 0, 0, 0, mb*g]) # target standing position

yref = target_position
yref_e = target_position[:6]
ocp.cost.yref = yref
ocp.cost.yref_e = yref_e

# parameters
ocp.parameter_values = np.array([])

# setting constraints
# setting constraints
lower_X = np.array([2*Rw,   -np.pi/2])
upper_X = np.array([1+2*Rw, np.pi/2])

ocp.constraints.lbx = lower_X
ocp.constraints.ubx = upper_X
ocp.constraints.idxbx = np.array([1, 2])

lower_U = np.array([-10, -200])
upper_U = np.array([10, 200])

ocp.constraints.lbu = lower_U
ocp.constraints.ubu = upper_U
ocp.constraints.idxbu = np.array([0, 1])


#  Set 
model.con_h_expr = None

# set intial condition
v0 = 0
x0 = np.array([ 0., 0.5, np.pi/8, 0., 0., 0.])
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

# create solver
acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

# Create log file
time_now = datetime.datetime.now()
folder = time_now.strftime("MPC_%Y_%m_%d_%H:%M:%S")
os.mkdir('results/' + folder)
with open('results/'+folder+'/data.txt', 'w') as f:
    print(f"# {os.getcwd().split('/')[-1]}", file=f)
    print(f'Tf = {Tf}', file=f)
    print(f'N = {N}', file=f)
    print(f'x0 = {x0}', file=f)
    print(f'target_position = {target_position}', file=f) 
    print(f'Q = {Q}', file=f)
    print(f'R = {R}', file=f)
    print(f'Qe = {Qe}', file=f)
    print(f'qp_solver = {ocp.solver_options.qp_solver}', file=f)
    print(f'nlp_solver_type = {ocp.solver_options.nlp_solver_type}', file=f)
    print(f'qp_solver_iter_max = {ocp.solver_options.qp_solver_iter_max}', file=f)
    print(f'nlp_solver_max_iter = {ocp.solver_options.nlp_solver_max_iter}', file=f)

Nsim = round(T * N / Tf)

# initialize data structs
simX = simX_new = np.zeros((Nsim,6))#simX_correct)#np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, 2))
#simX_horizon = np.ndarray((Nsim, N, nx))

tcomp_sum = 0
tcomp_max = 0
time_iterations = np.zeros(Nsim)
cost_integral = 0

noise_std = 0.00
# simulate
for i in tqdm(range(Nsim)):

    x_noise = x0 + np.random.normal(0, noise_std, x0.shape)
    acados_solver.set(0, "lbx", x_noise)
    acados_solver.set(0, "ubx", x_noise)
    acados_solver.set(0, "x", x_noise)

    for j in range(N):
        acados_solver.set(j, "yref", yref)
    
    acados_solver.set(N, 'yref', yref_e)

    status = acados_solver.solve()
    if status != 0:
        raise Exception("acados returned status {} in closed loop iteration {}.".format(status, i))
    
    # get solution
    u0 = acados_solver.get(0, "u")

    for j in range(2):
        simU[i, j] = u0[j]
    for j in range(nx):
        simX[i, j] = x0[j]

    # update initial condition
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    acados_solver.set(0, "x", x0)

    acados_solver.set(0, "lbu", u0)
    acados_solver.set(0, "ubu", u0)
    #x_noise = x0 + np.random.normal(0, noise_std, x0.shape)

    status = acados_solver.solve()
    if status != 0:
        raise Exception("acados returned status {} in closed loop iteration {}.".format(status, i))

    x0 = acados_solver.get(1, "x")

    



with open('results/'+folder+'/data.txt', 'a') as f:

    print(f'noise_std = {noise_std}', file=f)
    
    print(f'min_time = {np.min(time_iterations)}', file=f)
    print(f'max_time = {np.max(time_iterations)}', file=f)
    print(f'mean_time = {np.mean(time_iterations)}', file=f)
    print(f'std_time = {np.std(time_iterations)}', file=f)
    print(f'cost integral = {cost_integral}', file=f)

t = np.linspace(0.0, Nsim * Tf / N, Nsim)

simX_new=np.zeros((simX.shape[0],10))
simX_new[:,0]=simX[:,0]*Rw
simX_new[:,1]=Rw
simX_new[:,2:5]=simX[:,0:3]
simX_new[:,5]=simX[:,3]*Rw
simX_new[:,6]=0
simX_new[:,7:10]=simX[:,3:6]

simU_new=np.zeros((simU.shape[0],4))
simU_new[:,0:2]=simU[:,0:2]

plotRes(simX_new,t)
plt.savefig('results/' + folder + "/state_plots.eps")
plt.savefig('results/' + folder + "/state_plots.png", dpi=300)

plotU(simU_new,t)
plt.savefig('results/' + folder + "/control_plots.eps")
plt.savefig('results/' + folder + "/control_plots.png", dpi=300)

#plt.show()
with open('results/' + folder + "/simX.npy", 'wb') as f:
    np.save(f, simX)
with open('results/' + folder + "/simU.npy", 'wb') as f:
    np.save(f, simU)
#with open('results_pd/' + folder + "/simX_horizon.npy", 'wb') as f:
#    np.save(f, simX_horizon)
with open('results/' + folder + "/t.npy", 'wb') as f:
    np.save(f, t)

#plt.show()
# THIS IS A BIT SLOW
#renderVideo(simX_new, None, True, t, folder, "mpc")