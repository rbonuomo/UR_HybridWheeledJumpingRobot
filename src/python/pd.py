import numpy as np
import scipy.linalg
import time
import matplotlib.pyplot as plt
import datetime
import os

from CarModel_pd import CarModel_pd

mb = 4
mw = 2
Rw = 0.17
Iw = (mw*(Rw**2))

Tf = 0.5  # prediction horizon
N = round(Tf*500)  # number of discretization steps

x0 = np.array([1,2,3,4,5,6,7,8,9,10])

car_model = CarModel_pd(mb, mw, Iw, Rw, Tf/N, x0)

u = np.array([5, 5])
lam = np.array([1,1])

next_state = car_model.next_state(u,lam,Tf/N)

print(next_state)