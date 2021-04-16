from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from casadi import SX, vertcat, sin, cos, Function
import numpy as np
import scipy.linalg
from CarModel_mpc_noise import CarModel_mpc_noise
from CarModel_pd import CarModel_pd
import time
import matplotlib.pyplot as plt
import datetime
import os
from utils import *
from tqdm import tqdm

Tf = 1.5  # prediction horizon
N = round(Tf*20)  # number of discretization steps
T = 12.0  # maximum simulation time[s]

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
Q =  np.diag([ 5, 50, 0.5, 1, 50, 500])
R = np.diag([10, 1])
Qe = np.diag([ 5, 50, 0.5, 1, 10, 500])


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

Tf_par = 2  # prediction horizon
N_par = round(Tf_par*20)  # number of discretization steps
T_par = 8.0  # maximum simulation time[s]

mb = 4
mw = 2
mt = mb+mw
Rw = 0.17
Iw = (mw*(Rw**2))
g = 9.81

car_model_par = CarModel_pd(mb, mw, Iw, Rw, Tf_par/N_par)
model_par = car_model_par.model

ocp_par = AcadosOcp()
ocp_par.model = model_par
print(model_par)

# set dimensions
nx_par = model_par.x.size()[0]
nu_par = model_par.u.size()[0]
ny_par = nx_par + nu_par
ny_e_par = nx_par

ocp_par.dims.N = N_par

# set cost
Q_par =  np.diag([ 1, 1, 1, 1, 1, 1])*1
R_par = np.diag([])
Qe_par = np.diag([ 1, 1, 1, 1, 1, 1])*1


ocp_par.cost.cost_type = "LINEAR_LS"
ocp_par.cost.cost_type_e = "LINEAR_LS"
unscale_par = N_par / Tf_par

ocp_par.cost.W = unscale_par * scipy.linalg.block_diag(Q_par, R_par)
ocp_par.cost.W_e = Qe_par / unscale_par

Vx_par = np.zeros((ny_par, nx_par))
Vx_par[:nx_par, :nx_par] = np.eye(nx_par)
ocp_par.cost.Vx = Vx_par


Vu_par = np.zeros((ny_par, nu_par))
#Vu[-nu:] = np.eye(nu)
ocp_par.cost.Vu = Vu_par

Vx_e_par = np.zeros((ny_e_par, nx_par))
Vx_e_par[:nx_par, :nx_par] = np.eye(nx_par)
ocp_par.cost.Vx_e = Vx_e_par

# set intial references
#target_position = np.array([0., 0.8, 0, 0, 0, 0]) # target standing position
target_position_par = np.array([-2./Rw, 0.8, 0, 0, 0, 0]) # target standing position

yref_par = target_position_par
yref_e_par = target_position_par[:6]
ocp_par.cost.yref = yref_par
ocp_par.cost.yref_e = yref_e_par

# parameters
ocp_par.parameter_values = np.array([0, mb*g])


# setting constraints
# setting constraints
lower_X_par = np.array([])
upper_X_par = np.array([])

ocp_par.constraints.lbx = lower_X_par
ocp_par.constraints.ubx = upper_X_par
ocp_par.constraints.idxbx = np.array([])

lower_U_par = np.array([])
upper_U_par = np.array([])

ocp_par.constraints.lbu = lower_U_par
ocp_par.constraints.ubu = upper_U_par
ocp_par.constraints.idxbu = np.array([])


#  Set 
model_par.con_h_expr = None

# set intial condition
v0 = 0
x0_par = np.array([ 0., 0.5, np.pi/8, 0., 0., 0.])
x0_start_par = x0_par
ocp_par.constraints.x0 = x0_par

# set QP solver and integration
ocp_par.solver_options.tf = Tf_par
#ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ocp_par.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
ocp_par.solver_options.nlp_solver_type = "SQP_RTI"
ocp_par.solver_options.hessian_approx = "GAUSS_NEWTON"
ocp_par.solver_options.integrator_type = "DISCRETE"
ocp_par.solver_options.sim_method_num_stages = 4
ocp_par.solver_options.sim_method_num_steps = 1
ocp_par.solver_options.qp_solver_iter_max = 1000
ocp_par.solver_options.nlp_solver_max_iter = 1000

# create solver
acados_solver_par = AcadosOcpSolver(ocp_par, json_file="acados_ocp_par.json")

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

max_noise = 0.4
num_exp = 20

sigma_values = np.arange(0, max_noise+0.01, 0.02)
errors_array = np.zeros((sigma_values.shape[0], num_exp))

x0_start = x0
for noise_index, noise_std in enumerate(sigma_values): 
    
    n_exp = 0
    while n_exp<num_exp:
        print(f"Noise value: {noise_std}   {noise_index}/{len(sigma_values)}     n_exp: {n_exp}/{num_exp}")
        # simulate
        acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        acados_solver_par = AcadosOcpSolver(ocp_par, json_file="acados_ocp_par.json")
        
        simX = np.zeros((Nsim,6))#simX_correct)#np.ndarray((Nsim, nx))
        simU = np.ndarray((Nsim, 2))

        x0 = x0_start
        try:
            # simulate
            for i in tqdm(range(Nsim)):

                #print(x0)
                x_noise = x0 + np.diag([1, 0.1, 0.2, 1, 0.1, 0.2])@np.random.normal(0, noise_std, x0.shape)
                #print(x_noise)
                acados_solver.set(0, "lbx", x_noise)
                acados_solver.set(0, "ubx", x_noise)
                #acados_solver.set(0, "x", x_noise)
                for j in range(N):
                    acados_solver.set(j, "yref", yref)
                
                acados_solver.set(N, 'yref', yref_e)
                status = acados_solver.solve()
                if status != 0:
                    raise Exception("acados returned status {} in closed loop iteration {}.".format(status, i))
                
                # get solution
                u0 = acados_solver.get(0, "u")
                #print(u0)

                for j in range(2):
                    simU[i, j] = u0[j]
                for j in range(nx):
                    simX[i, j] = x0[j]

                # update initial condition
                acados_solver_par.set(0, "lbx", x0)
                acados_solver_par.set(0, "ubx", x0)
                #acados_solver.set(0, "x", x0)

                for j in range(N):
                    acados_solver_par.set(j, "p", u0)
                
                #x_noise = x0 + np.random.normal(0, noise_std, x0.shape)

                status = acados_solver_par.solve()
                if status != 0:
                    print("acados returned status {} in closed loop iteration {}.".format(status, i))

                x0 = acados_solver_par.get(1, "x")
                #print(x0)
                if np.isnan(x0).any():
                    pippo
            e = np.linalg.norm(target_position[:6]-x0)
            if not np.isnan(e).any():
                errors_array[noise_index, n_exp] = e
                n_exp += 1
        except:
            pass
                
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