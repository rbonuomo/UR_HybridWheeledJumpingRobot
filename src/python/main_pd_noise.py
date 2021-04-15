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
T = 8.0  # maximum simulation time[s]

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
x0_start = x0
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

max_noise = 0.2
num_exp = 20

sigma_values = np.arange(0, max_noise+0.01, 0.02)
errors_array = np.zeros((sigma_values.shape[0], num_exp))

for noise_index, noise_std in enumerate(sigma_values): 
    
    n_exp = 0
    while n_exp<num_exp:
        print(f"Noise value: {noise_std}   {noise_index}/{len(sigma_values)}     n_exp: {n_exp}/{num_exp}")
        # simulate
        acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        simX = np.zeros((Nsim,6))#simX_correct)#np.ndarray((Nsim, nx))
        simU = np.ndarray((Nsim, 2))

        x0 = x0_start
        for i in range(Nsim):

            x_noise = x0 + np.diag([1, 0.1, 0.2, 1, 0.1, 0.2])@np.random.normal(0, noise_std, x0.shape)

            tau = -Kp_theta*(target_position[2]-x_noise[2]) -Kd_theta*(target_position[5]-x_noise[5]) - Kp_phi*(target_position[0]-x_noise[0])-Kd_phi*(target_position[3]-x_noise[3])
            f = mb*g*np.cos(x_noise[2]) + Kp_l*(target_position[1]-x_noise[1]) + Kd_l*(target_position[4]-x_noise[4])

            if np.abs(tau)>10:
                tau = np.sign(tau)*10
            if np.abs(f)>200:
                f = np.sign(f)*200

            # update reference
            for j in range(N):
                acados_solver.set(j, "p", np.array([tau, f]))
            
            # solve ocp
            status = acados_solver.solve()
            if status != 0:
                raise Exception("acados returned status {} in closed loop iteration {}.".format(status, i))
                        
            # get solution
            x0 = acados_solver.get(0, "x")
            u0 = acados_solver.get(0, "u")

            u_exp = np.array([tau, f])

            for j in range(nx):
                simX[i, j] = x0[j]
            for j in range(2):
                simU[i, j] = u_exp[j]

            # update initial condition
            x0 = acados_solver.get(1, "x")
            acados_solver.set(0, "lbx", x0)
            acados_solver.set(0, "ubx", x0)
        
        e = np.linalg.norm(target_position-x0)
        if not np.isnan(e).any():
            errors_array[noise_index, n_exp] = e
            n_exp += 1
        # t = np.linspace(0.0, Nsim * Tf / N, Nsim)

        # simX_new=np.zeros((simX.shape[0],10))
        # simX_new[:,0]=simX[:,0]*Rw
        # simX_new[:,1]=Rw
        # simX_new[:,2:5]=simX[:,0:3]
        # simX_new[:,5]=simX[:,3]*Rw
        # simX_new[:,6]=0
        # simX_new[:,7:10]=simX[:,3:6]

        # simU_new=np.zeros((simU.shape[0],4))
        # simU_new[:,0:2]=simU[:,0:2]

        # plotRes(simX_new,t)
        # plt.savefig('results/' + folder + f"/state_plots_{noise_std}_{n_exp}.eps")
        # plt.savefig('results/' + folder + f"/state_plots_{noise_std}_{n_exp}.png", dpi=300)

        # plotU(simU_new,t)
        # plt.savefig('results/' + folder + f"/control_plots_{noise_std}_{n_exp}.eps")
        # plt.savefig('results/' + folder + f"/control_plots_{noise_std}_{n_exp}.png", dpi=300)


with open('results/'+folder+'/data.txt', 'a') as f:
    print(f'Kp_theta = {Kp_theta}', file=f)
    print(f'Kd_theta = {Kd_theta}', file=f)
    print(f'Kp_phi = {Kp_phi}', file=f)
    print(f'Kd_phi = {Kd_phi}', file=f)
    print(f'Kp_l = {Kp_l}', file=f)
    print(f'Kd_l = {Kd_l}', file=f)
    print(f'noise_std = {noise_std}', file=f)
    
    
print(errors_array)

mean_array = np.mean(errors_array, axis=1)
print(mean_array)
std_array = np.std(errors_array, axis=1)
print(std_array)

plt.plot(sigma_values, mean_array)
plt.fill_between(sigma_values,mean_array-std_array,mean_array+std_array,alpha=.1)
plt.ylabel(r'Final State L2-distance to x*',fontsize ='16')
plt.xlabel(r'Noise Strength ($\sigma$)',fontsize ='16')
plt.grid(True)
plt.savefig('results/' + folder + f"/noise_plot.eps")
plt.savefig('results/' + folder + f"/noise_plot.png", dpi=300)

with open('results/' + folder + "/errors_array.npy", 'wb') as f:
    np.save(f, errors_array)
with open('results/' + folder + "/mean_array.npy", 'wb') as f:
    np.save(f, mean_array)
with open('results/' + folder + "/std_array.npy", 'wb') as f:
    np.save(f, std_array)
#plt.show()
# THIS IS A BIT SLOW
#renderVideo(simX_new, None, True, t, folder, "pd")