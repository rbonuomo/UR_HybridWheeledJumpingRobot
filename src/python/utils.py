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
    plt.ylabel(r'$x$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 2)
    plt.plot(t, simX[:,1])
    plt.ylabel(r'$z$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 3)
    plt.plot(t, simX[:,2])
    plt.ylabel(r'$\phi$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 4)
    plt.plot(t, simX[:,3])
    plt.ylabel(r'$l$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 5)
    plt.plot(t, simX[:,4])
    plt.ylabel(r'$\theta$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 6)
    plt.plot(t, simX[:,5])
    plt.ylabel(r'$\dot{x}$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 7)
    plt.plot(t, simX[:,6])
    plt.ylabel(r'$\dot{z}$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 8)
    plt.plot(t, simX[:,7])
    plt.ylabel(r'$\dot{\phi}$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 9)
    plt.plot(t, simX[:,8])
    plt.ylabel(r'$\dot{l}$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 5, 10)
    plt.plot(t, simX[:,9])
    plt.ylabel(r'$\dot{\theta}$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.tight_layout()

def plotU(simU, t):
    plt.figure(figsize=(12.8, 9.6))
    plt.subplot(2, 2, 1)
    plt.plot(t, simU[:,0])
    plt.ylabel(r'$\tau$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(t, simU[:,1])
    plt.ylabel(r'$f$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(t, simU[:,2])
    plt.ylabel(r'$\lambda_x$')
    plt.xlabel(r'$t$')
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(t, simU[:,3])
    plt.ylabel(r'$\lambda_z$')
    plt.xlabel(r'$t$')
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
    if i==0:
        plt.savefig('results/' + folder + "/%04d" % i +"_hor.eps")

    #plt.show()
    plt.close()

def renderVideo(simX, simX_horizon, t, folder):
    

    for i in tqdm(range(simX.shape[0])):
        state = simX[i, :]
        horizon = simX_horizon[i]
        drawRobot(state, i, folder)
        drawRobotWithHorizon(state, horizon, i, folder)

    
    period = np.mean(np.diff(t))
    fr = int(np.around(1/period, decimals=0))
    os.chdir('results/' + folder)
    os.system(f"ffmpeg -framerate {fr}"+" -i %04d.png -r 30 -pix_fmt yuv420p video.mp4")
    for i in tqdm(range(simX.shape[0]), desc="Removing temp files"):
        os.system('rm %04d.png' %i)
    os.system(f"ffmpeg -framerate {fr}"+" -i %04d_hor.png -r 30 -pix_fmt yuv420p video_with_horizon.mp4")
    for i in tqdm(range(simX.shape[0]), desc="Removing temp files"):
        os.system('rm %04d_hor.png' %i)
    os.chdir('../..')