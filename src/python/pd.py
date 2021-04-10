import numpy as np
import scipy.linalg
import time
import matplotlib.pyplot as plt
import datetime
import os
from math import pi

from CarModel_pd import CarModel_pd

mb = 4
mw = 2
Rw = 0.17
Iw = (mw*(Rw**2))
g = 9.81

Tf = 0.5  # prediction horizon
N = round(Tf*500)  # number of discretization steps

x0 = np.array([0,Rw,0,0.5,pi/8,0,0,0,0,0])

car_model = CarModel_pd(mb, mw, Iw, Rw, Tf/N, x0)

# u = np.array([5, 5])
# lam = np.array([1,1])

# next_state = car_model.next_state(u,lam,Tf/N)

# print(next_state)

t = 0

xdes = np.array([0,Rw,0,0.5,0,0,0,0,0,0])
f = 0
ddq = np.array([0,0,0,0,0])

e = xdes[4]-x0[4]

xHistory = []
uHistory = []

while abs(e) > 0.2:

    tau = -15*(xdes[4]-x0[4])-2*(xdes[9]-x0[9])-1.25*(xdes[0]-x0[0])-2*(xdes[5]-x0[5])
    f = mb*g + 1*(mb*g-f)

    u = np.array([tau,f])

    lam = car_model.return_lambda(u,x0,ddq)

    next_state = car_model.next_state(u,lam,Tf/N)

    e = xdes[4]-next_state[4]

    t = t + Tf/N

    x0 = next_state
    dX = car_model.return_continuous_model(u, lam, t, x0)
    ddq = dX[5:]
