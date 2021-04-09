import matplotlib.pyplot as plt
import math
import matplotlib
import os
from tqdm import tqdm
import numpy as np

def plotRes(simX, t):
    # plot results
    plt.figure(figsize=(16, 8))
    plt.subplot(2, 5, 1)
    plt.plot(t, simX[:,0])
    plt.ylabel(r'$x$', fontsize ='16')
    plt.xlabel(r'$t$ $(s)$', fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 2)
    plt.plot(t, simX[:,1])
    plt.ylabel(r'$z$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 3)
    plt.plot(t, simX[:,2])
    plt.ylabel(r'$\phi$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 4)
    plt.plot(t, simX[:,3])
    plt.ylabel(r'$l$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 5)
    plt.plot(t, simX[:,4])
    plt.ylabel(r'$\theta$',fontsize ='16')
    plt.xlabel(r'$t (s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 6)
    plt.plot(t, simX[:,5])
    plt.ylabel(r'$\dot{x}$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$', fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 7)
    plt.plot(t, simX[:,6])
    plt.ylabel(r'$\dot{z}$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 8)
    plt.plot(t, simX[:,7])
    plt.ylabel(r'$\dot{\phi}$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 9)
    plt.plot(t, simX[:,8])
    plt.ylabel(r'$\dot{l}$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 10)
    plt.plot(t, simX[:,9])
    plt.ylabel(r'$\dot{\theta}$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.tight_layout()

def plotU(simU, t):
    plt.figure(figsize=(12.8, 9.6))
    plt.subplot(2, 2, 1)
    plt.plot(t, simU[:,0])
    plt.ylabel(r'$\tau$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(t, simU[:,1])
    plt.ylabel(r'$f$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(t, simU[:,2])
    plt.ylabel(r'$\lambda_x$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(t, simU[:,3])
    plt.ylabel(r'$\lambda_z$',fontsize ='16')
    plt.xlabel(r'$t$ $(s)$',fontsize ='16')
    plt.grid(True)

    plt.tight_layout()

def drawRobot(state, i, folder):
    plt.figure(figsize=(10, 10))
    plt.xlim((-1, 3))
    plt.ylim((-1, 3))

    rect = matplotlib.patches.Rectangle((state[0]+math.cos(-state[4])*0.1, state[1]+math.sin(-state[4])*0.1), state[3], 0.2, angle=90-state[4]*180/math.pi)
    plt.gca().add_patch(rect)

    circle = plt.Circle((state[0], state[1]), 0.17, color='r')
    plt.gca().add_patch(circle)

    x_values = [state[0], state[0]+math.cos(-state[2]+math.pi/2)*0.17]
    y_values = [state[1], state[1]+math.sin(-state[2]+math.pi/2)*0.17]
    plt.plot(x_values, y_values, color='k')

    gap_minus = 1.5
    gap_plus = 2.

    x_values = [-10, gap_minus]
    y_values = [0, 0]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_plus, 10]
    y_values = [0, 0]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_minus, gap_minus]
    y_values = [0, -10]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_plus, gap_plus]
    y_values = [0, -10]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    plt.savefig('results/' + folder + "/%04d" % i +".png")
    if i==0:
        plt.savefig('results/' + folder + "/%04d" % i +".eps")

    #plt.show()
    plt.close()

def drawRobotWithHorizon(state, horizon, i, folder):
    plt.figure(figsize=(10, 10))
    plt.xlim((-1, 3))
    plt.ylim((-1, 3))

    #print(state)
    #print(horizon)
    
    for j in range(horizon.shape[0]-1, -1, -1): 
        state = horizon[j]
        if j==0:
            alpha=1
        else:
            alpha = 0.02

        rect = matplotlib.patches.Rectangle((state[0]+math.cos(-state[4])*0.1, state[1]+math.sin(-state[4])*0.1), state[3], 0.2, angle=90-state[4]*180/math.pi, alpha=alpha)
        plt.gca().add_patch(rect)

        circle = plt.Circle((state[0], state[1]), 0.17, color='r', alpha=alpha)
        plt.gca().add_patch(circle)

        x_values = [state[0], state[0]+math.cos(-state[2]+math.pi/2)*0.17]
        y_values = [state[1], state[1]+math.sin(-state[2]+math.pi/2)*0.17]
        plt.plot(x_values, y_values, color='k', alpha=alpha)

    gap_minus = 1.5
    gap_plus = 2.

    x_values = [-10, gap_minus]
    y_values = [0, 0]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_plus, 10]
    y_values = [0, 0]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_minus, gap_minus]
    y_values = [0, -10]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    x_values = [gap_plus, gap_plus]
    y_values = [0, -10]
    plt.plot(x_values, y_values, color='k', linewidth=0.5)

    plt.savefig('results/' + folder + "/%04d" % i +"_hor.png")
    
    #plt.show()
    plt.close()

def renderVideo(simX, simX_horizon, phase0, t, folder, name):
    
    if phase0:
        for i in tqdm(range(0, simX.shape[0])):
            state = simX[i, :]
            drawRobot(state, i, folder)
    else:
        for i in tqdm(range(simX.shape[0])):
            state = simX[i, :]
            drawRobot(state, i, folder)
            horizon = simX_horizon[i]
            drawRobotWithHorizon(state, horizon, i, folder)
    period = np.mean(np.diff(t))
    fr = int(np.around(1/period, decimals=0))
    os.chdir('results/' + folder)
    os.system(f"ffmpeg -framerate {fr}"+" -i %04d.png -r 30 -pix_fmt yuv420p "+f"{name}.mp4")
    for i in tqdm(range(simX.shape[0]), desc="Removing temp files"):
        os.system('rm %04d.png' %i)
    if not phase0:
        os.system(f"ffmpeg -framerate {fr}"+" -i %04d_hor.png -r 30 -pix_fmt yuv420p "+f"{name}_with_horizon.mp4")
        for i in tqdm(range(simX.shape[0]), desc="Removing temp files"):
            os.system('rm %04d_hor.png' %i)
    os.chdir('../..')

def get_extended_state(simX, Rw):
    print(simX.shape)
    ext_state = np.zeros((simX.shape[0], 10))
    ext_state[:, 0] = simX[:, 0]*Rw
    ext_state[:, 1] = Rw
    ext_state[:, 2] = simX[:, 0]
    ext_state[:, 3] = 0.5
    ext_state[:, 4] = simX[:, 1]-np.pi/2
    ext_state[:, 5] = simX[:, 2]*Rw
    ext_state[:, 6] = 0
    ext_state[:, 7] = simX[:, 2]
    ext_state[:, 8] = 0
    ext_state[:, 9] = simX[:, 3]
    return ext_state

def get_extended_controls(simU):
    ext_U = np.zeros((simU.shape[0], 4))
    ext_U[:, 0] = simU[:, 0]
    ext_U[:, 1] = 0
    ext_U[:, 2] = 0
    ext_U[:, 3] = 0
    return ext_U