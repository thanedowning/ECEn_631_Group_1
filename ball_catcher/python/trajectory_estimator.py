#!/usr/bin/env python3
from estimate_trajectory_interface import TrajectoryEstimator
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style("whitegrid")
from IPython.core.debugger import Pdb

if __name__ == '__main__':
    trj = TrajectoryEstimator()
    DISPLAY = False
    PLOT = True
    trj.init(DISPLAY)
    
    pts = np.zeros((100,3));
    for i in range(100):
        pts[i] = trj.run(i)
    
    begin = 32
    end = 63
    x = pts[:,0][begin:end]
    y = pts[:,1][begin:end]
    z = pts[:,2][begin:end]
    
    vz = (z[-1] - z[-6]) / 5 * 60  # velocity in inches/sec
    t = -z[-1] / vz                # time to reach z = 0

    vx = (x[-1] - x[-6]) / 5 * 60
    x0 = vx * t + x[-1]

    vy = (y[-1] - y[-6]) / 5 * 60
    y0 = vy * t + y[-1]
    
    xp = np.array([x[-1], x0])
    yp = np.array([y[-1], y0])
    zp = np.array([z[-1], 0])

    ### Plotting
    if PLOT:
        fig1 = plt.figure()
        plt.plot(x, z, label='measured')
        plt.plot(xp, zp, label='predicted')
        plt.legend(loc='upper right')
        plt.xlabel('x')
        plt.ylabel('z')

        fig2 = plt.figure()
        plt.plot(y, z, label='measured')
        plt.plot(yp, zp, label='predicted')
        plt.legend(loc='upper right')
        plt.xlabel('y')
        plt.ylabel('z')
        plt.show()