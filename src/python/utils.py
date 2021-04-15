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
    plt.plot(t, simX[:,0], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$x$ $[m]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 2)
    plt.plot(t, simX[:,1], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$z$ $[m]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 3)
    plt.plot(t, simX[:,2], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\phi$ $[rad]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 4)
    plt.plot(t, simX[:,3], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$l$ $[m]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 5)
    plt.plot(t, simX[:,4], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\theta$ $[rad]$',fontsize ='16')
    plt.xlabel(r'$t [s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 6)
    plt.plot(t, simX[:,5], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{x}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 7)
    plt.plot(t, simX[:,6], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{z}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 8)
    plt.plot(t, simX[:,7],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{\phi}$ $[rad/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 9)
    plt.plot(t, simX[:,8], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{l}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 5, 10)
    plt.plot(t, simX[:,9], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{\theta}$ $[rad/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.tight_layout()

def plotU(simU, t):
    plt.figure(figsize=(12.8, 9.6))
    plt.subplot(2, 2, 1)
    plt.plot(t, simU[:,0], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\tau$ $[Nm]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(t, simU[:,1],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$f$ $[N]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(t, simU[:,2], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\lambda_x$ $[N]$ ',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(t, simU[:,3],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\lambda_z$ $[N]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)

    plt.tight_layout()


def plotState(simX,t):
    plt.figure()
    plt.plot(t, simX[:,0])
    plt.plot(t, simX[:,1])
    plt.plot(t, simX[:,2])
    plt.plot(t, simX[:,3])
    plt.plot(t, simX[:,4])
    plt.xlim(t[0],t[-1])
    plt.title('state variables')
    plt.legend([r'$x$',r'$z$', r'$\phi$',r'$l$', r'$\theta$'])
    plt.grid(True)

def plotStateDot(simX,t):
    plt.figure()
    plt.plot(t, simX[:,5])
    plt.plot(t, simX[:,6])
    plt.plot(t, simX[:,7])
    plt.plot(t, simX[:,8])
    plt.plot(t, simX[:,9])
    plt.xlim(t[0],t[-1])
    plt.title('derivative of state variables')
    plt.legend([r'$\dot{x}$',r'$\dot{z}$',r'$\dot{\phi}$',r'$\dot{l}$',r'$\dot{\theta}$'])
    plt.grid(True)

def plotControl(simU,t):
    plt.figure()
    plt.plot(t, simU[:,0])
    plt.plot(t, simU[:,1])
    plt.plot(t, simU[:,2])
    plt.plot(t, simU[:,3])
    plt.xlim(t[0],t[-1])
    plt.title('control variables')
    plt.legend([r'$\tau$',r'$f$', r'$\lambda_x$',r'$\lambda_z$'])
    plt.grid(True)

def single_plots(simX,simU,t,path):
    #x variable
    plt.figure()
    plt.title(r'x variable',fontsize='16')
    plt.plot(t, simX[:,0], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$x$ $[m]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_x.eps")
    plt.savefig(path + "/var_x.png", dpi=300)  

    #z variable
    plt.figure()
    plt.title(r'z variable',fontsize='16')
    plt.plot(t, simX[:,1], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$z$ $[m]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_z.eps")
    plt.savefig(path + "/var_z.png", dpi=300)  

    # #phi variable
    plt.figure()
    plt.title(r'$\phi$ variable',fontsize='16')
    plt.plot(t, simX[:,2], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\phi$ $[rad]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_phi.eps")
    plt.savefig(path + "/var_phi.png", dpi=300)  

    #l variable
    plt.figure()
    plt.title(r'l variable',fontsize='16')
    plt.plot(t, simX[:,3], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$l$ $[m]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_l.eps")
    plt.savefig(path + "/var_l.png", dpi=300)  

    #theta variable
    plt.figure()
    plt.title(r'$\theta$ variable',fontsize='16')
    plt.plot(t, simX[:,4], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\theta$ $[rad]$', fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_theta.eps")
    plt.savefig(path + "/var_theta.png", dpi=300)  

    #xdot variable
    plt.figure()
    plt.title(r'$\dot{x}$ variable',fontsize='16')
    plt.plot(t, simX[:,5], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{x}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$', fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_xdot.eps")
    plt.savefig(path + "/var_xdot.png", dpi=300)  

    #zdot variable
    plt.figure()
    plt.title(r'$\dot{z}$ variable',fontsize='16')
    plt.plot(t, simX[:,6], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{z}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_zdot.eps")
    plt.savefig(path + "/var_zdot.png", dpi=300)  

    #phidot variable
    plt.figure()
    plt.title(r'$\dot{\phi}$ variable',fontsize='16')
    plt.plot(t, simX[:,7],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{\phi}$ $[rad/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_phidot.eps")
    plt.savefig(path + "/var_phidot.png", dpi=300)  

    #ldot variable
    plt.figure()
    plt.title(r'$\dot{l}$ variable',fontsize='16')
    plt.plot(t, simX[:,8], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{l}$ $[m/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_ldot.eps")
    plt.savefig(path + "/var_ldot.png", dpi=300)  

    #thetadot variable
    plt.figure()
    plt.title(r'$\dot{\theta}$ variable',fontsize='16')
    plt.plot(t, simX[:,9], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\dot{\theta}$ $[rad/s]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_thetadot.eps")
    plt.savefig(path + "/var_thetadot.png", dpi=300)  

    #tau variable
    plt.figure()
    plt.title(r'$\tau$ variable',fontsize='16')
    plt.plot(t, simU[:,0], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\tau$ $[Nm]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_tau.eps")
    plt.savefig(path + "/var_tau.png", dpi=300)  

    #f variable
    plt.figure()
    plt.title(r'$f$ variable',fontsize='16')
    plt.plot(t, simU[:,1],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$f$ $[N]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_f.eps")
    plt.savefig(path + "/var_f.png", dpi=300)  

    plt.figure()
    plt.title(r'$\lambda_x$ variable',fontsize='16')
    plt.plot(t, simU[:,2], linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\lambda_x$ $[N]$ ',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_lambdax.eps")
    plt.savefig(path + "/var_lambdax.png", dpi=300)  

    plt.figure()
    plt.title(r'$\lambda_z$ variable',fontsize='16')   
    plt.plot(t, simU[:,3],linewidth='2')
    plt.xlim(t[0],t[-1])
    plt.ylabel(r'$\lambda_z$ $[N]$',fontsize ='16')
    plt.xlabel(r'$t$ $[s]$',fontsize ='16')
    plt.grid(True)
    plt.savefig(path + "/var_lambdaz.eps")
    plt.savefig(path + "/var_lambdaz.png", dpi=300)  
 

def drawRobot(state, i, folder):
    plt.figure(figsize=(20, 10))
    plt.xlim((-4, 4))
    plt.ylim((-1, 3))

    rect = matplotlib.patches.Rectangle((state[0]+math.cos(-state[4])*0.1, state[1]+math.sin(-state[4])*0.1), state[3], 0.20, angle=90-state[4]*180/math.pi, color='darkgrey')
    plt.gca().add_patch(rect)

    rect1 = matplotlib.patches.Rectangle((state[0]+math.cos(-state[4])*0.11, state[1]+math.sin(-state[4])*0.11), 0.3, 0.22, angle=90-state[4]*180/math.pi,color='black')
    plt.gca().add_patch(rect1)

    circle = plt.Circle((state[0], state[1]), 0.17, color='darkred')
    plt.gca().add_patch(circle)

    circle1 = plt.Circle((state[0], state[1]), 0.10, color='white')
    plt.gca().add_patch(circle1)

    circle2 = plt.Circle((state[0]+state[3]*math.sin(state[4]), state[1]+state[3]*math.cos(state[4])), 0.17, color='darkred')
    plt.gca().add_patch(circle2)

    circle3 = plt.Circle((state[0]+state[3]*math.sin(state[4]), state[1]+state[3]*math.cos(state[4])), 0.10, color='white')
    plt.gca().add_patch(circle3)

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