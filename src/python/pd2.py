import numpy as np
import scipy.linalg
import time
import matplotlib.pyplot as plt
import datetime
import os
from math import pi
from utils import *

from CarModel_pdmodel import CarModel_pd

mb = 4
mw = 2
Rw = 0.17
Iw = (mw*(Rw**2))
g = 9.81

Tf = 0.5  # prediction horizon
N = round(Tf*20)  # number of discretization steps

x0 = np.array([0,0.5,pi/8,0,0,0])

car_model = CarModel_pd(mb, mw, Iw, Rw, Tf/N, x0)

# u = np.array([5, 5])
# lam = np.array([1,1])

# next_state = car_model.next_state(u,lam,Tf/N)

# print(next_state)

t = 0

xdes = np.array([0,0.5,0,0,0,0])
f = 0
ddq = np.array([0,0,0])

e = xdes[2]-x0[2]

xHistory = []
uHistory = []
t=[]
t_now=0
while (t_now) < 10:

    tau = -15*(xdes[2]-x0[2])-2*(xdes[5]-x0[5]) #+1.25/Rw*(xdes[0]-x0[0])+2/Rw*(xdes[3]-x0[3])
    #f = mb*g + 1*(mb*g-f)
    f=mb*g*np.cos(x0[2])
    #f=mb*g*np.cos(x0[2]) + 2*(xdes[1]-x0[1]) + 2*(xdes[4]-x0[4])
    

    u = np.array([tau,f])

    next_state = car_model.next_state(u,(Tf/N))

    e = xdes[2]-next_state[2]
    print(e)

    t_now = t_now + Tf/N
    t.append(t_now)

    
    x0 = next_state
    xHistory.append(x0)
    uHistory.append(u)
   # dX = car_model.return_continuous_model(u, t, x0)


simX=np.array([xHistory])
simX=simX.reshape((simX.shape[1],simX.shape[2]))
print(simX.shape)

simU=np.array([uHistory])
simU=simU.reshape((simU.shape[1],simU.shape[2]))
print(simU.shape)

simX_new=np.zeros((simX.shape[0],10))
simX_new[:,0]=simX[:,0]*Rw
simX_new[:,1]=Rw
simX_new[:,2:5]=simX[:,0:3]
simX_new[:,5]=simX[:,3]*Rw
simX_new[:,6]=0
simX_new[:,7:10]=simX[:,3:6]

simU_new=np.zeros((simU.shape[0],4))
simU_new[:,0:2]=simU[:,0:2]

time_now = datetime.datetime.now()
folder = time_now.strftime("%Y_%m_%d_%H:%M:%S")
os.mkdir('results/' + folder)

plotU(simU_new,t)
plt.savefig('results/' + folder + "/control_plots.eps")
plt.savefig('results/' + folder + "/control_plots.png", dpi=300)

plotRes(simX_new,t)
plt.savefig('results/' + folder + "/state_plots.eps")
plt.savefig('results/' + folder + "/state_plots.png", dpi=300)

renderVideo(simX_new, None, True, t, folder, "pd")