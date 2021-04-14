from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from casadi import SX, vertcat, sin, cos, Function
import numpy as np
import scipy.linalg
from CarModel_pd import CarModel_pd
import time
import matplotlib.pyplot as plt
import datetime
import os
from utils import *
from tqdm import tqdm

Tf = 2  # prediction horizon
N = round(Tf*20)  # number of discretization steps
T = 20.0  # maximum simulation time[s]

mb = 4
mw = 2
mt = mb+mw
Rw = 0.17
Iw = (mw*(Rw**2))
g = 9.81

car_model = CarModel_pd(mb, mw, Iw, Rw, Tf/N)
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
Q =  np.diag([ 1, 1, 1, 1, 1, 1])*1
R = np.diag([])
Qe = np.diag([ 1, 1, 1, 1, 1, 1])*1


ocp.cost.cost_type = "LINEAR_LS"
ocp.cost.cost_type_e = "LINEAR_LS"
unscale = N / Tf

ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
ocp.cost.W_e = Qe / unscale

Vx = np.zeros((ny, nx))
Vx[:nx, :nx] = np.eye(nx)
ocp.cost.Vx = Vx

Vu = np.zeros((ny, nu))
#Vu[-nu:] = np.eye(nu)
ocp.cost.Vu = Vu

Vx_e = np.zeros((ny_e, nx))
Vx_e[:nx, :nx] = np.eye(nx)
ocp.cost.Vx_e = Vx_e

# set intial references
#target_position = np.array([0., 0.8, 0, 0, 0, 0]) # target standing position
target_position = np.array([-2./Rw, 0.8, 0, 0, 0, 0]) # target standing position

yref = target_position
yref_e = target_position[:6]
ocp.cost.yref = yref
ocp.cost.yref_e = yref_e

# parameters
ocp.parameter_values = np.array([0, mb*g])

# setting constraints
# setting constraints
lower_X = np.array([])
upper_X = np.array([])

ocp.constraints.lbx = lower_X
ocp.constraints.ubx = upper_X
ocp.constraints.idxbx = np.array([])

lower_U = np.array([])
upper_U = np.array([])

ocp.constraints.lbu = lower_U
ocp.constraints.ubu = upper_U
ocp.constraints.idxbu = np.array([])


#  Set 
model.con_h_expr = None

# set intial condition
v0 = 0
x0 = np.array([ 0., 0.5, np.pi/8, 0., 0., 0.])
ocp.constraints.x0 = x0

# set QP solver and integration
ocp.solver_options.tf = Tf
#ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
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
folder = time_now.strftime("PD_%Y_%m_%d_%H:%M:%S")
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

Kp_theta = 20
Kd_theta = 4
Kp_phi = 0.125
Kd_phi = 0.2
Kp_l = 20
Kd_l = 20

noise_std = 0.0
# simulate
for i in tqdm(range(Nsim)):

    actual_time = i* (T/Nsim)

    x_noise = x0 + np.random.normal(0, noise_std, x0.shape)

    tau = -Kp_theta*(target_position[2]-x_noise[2]) -Kd_theta*(target_position[5]-x_noise[5]) - Kp_phi*(target_position[0]-x_noise[0])-Kd_phi*(target_position[3]-x_noise[3])
    f = mb*g*np.cos(x_noise[2]) + Kp_l*(target_position[1]-x_noise[1]) + Kd_l*(target_position[4]-x_noise[4])

    # update reference
    for j in range(N):
        acados_solver.set(j, "p", np.array([tau, f]))
    
    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    if status != 0:
        raise Exception("acados returned status {} in closed loop iteration {}.".format(status, i))

    elapsed = time.time() - t
    time_iterations[i] = elapsed

    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed
    
    
    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")

    u_exp = np.array([tau, f])

    cost_integral += np.matmul(u0, u0.T)*Tf / N
    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(2):
        simU[i, j] = u_exp[j]
    #for j in range(N):
    #    simX_horizon[i, j, :] = acados_solver.get(j, 'x')

    # update initial condition
    x0 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)


with open('results/'+folder+'/data.txt', 'a') as f:
    print(f'Kp_theta = {Kp_theta}', file=f)
    print(f'Kd_theta = {Kd_theta}', file=f)
    print(f'Kp_phi = {Kp_phi}', file=f)
    print(f'Kd_phi = {Kd_phi}', file=f)
    print(f'Kp_l = {Kp_l}', file=f)
    print(f'Kd_l = {Kd_l}', file=f)
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
renderVideo(simX_new, None, True, t, folder, "pd")