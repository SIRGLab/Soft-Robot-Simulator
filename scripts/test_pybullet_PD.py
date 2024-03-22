import numpy as np
from pybullet_env.BasicEnvironment import SoftRobotBasicEnvironment
from visualizer.visualizer import ODE
import time
import numpy as np

import matplotlib.pyplot as plt
from softRobotCtrl.robotControlOpenLoop import SoftRobotControl


def get_ref(gt,traj_name='Circle'):
    
        if traj_name == 'Rose':
            k = 4
            T  = 200
            w  = 2*np.pi/T
            a = 0.025
            r  = a * np.cos(k*w*gt)
            xd = (x0 + np.array((r*np.cos(w*gt),r*np.sin(w*gt),0.00*gt)))
            xd_dot = np.array((-r*w*np.sin(w*gt),r*w*np.cos(w*gt),0.00*gt))
        elif traj_name == 'Limacon':
            T  = 100
            w  = 2*np.pi/T
            radius = 0.02
            radius2 = 0.03
            shift = -0.02
            xd = (x0 + np.array(((shift+(radius+radius2*np.cos(w*gt))*np.cos(w*gt)),(radius+radius2*np.cos(w*gt))*np.sin(w*gt),0.00*gt)))
            xd_dot = np.array((radius*(-w*np.sin(w*(gt)-0.5*w*np.sin(w/2*(gt)))),radius*(w*np.cos(w*(gt)-0.5*radius2*np.cos(w/2*gt))),0.00))                            
        elif traj_name=='Circle':
            T  = 50*2
            w  = 2*np.pi/T
            radius = 0.02
            xd = (x0 + np.array((radius*np.sin(w*(gt)),radius*np.cos(w*(gt)),0.00*gt)))
            xd_dot = np.array((radius*w*np.cos(w*(gt)),-radius*w*np.sin(w*(gt)),0.00))
        elif traj_name=='Helix':
            T  = 50
            w  = 2*np.pi/T
            radius = 0.02
            xd = (x0 + np.array((radius*np.sin(w*(gt)),radius*np.cos(w*(gt)),0.0001*gt)))
            xd_dot = ( np.array((radius*w*np.cos(w*(gt)),-radius*w*np.sin(w*(gt)),0.0001)))
        elif traj_name=='Eight_Figure':
            T  = 25*2
            A  = 0.02
            w  = 2*np.pi/T
            xd = np.array((A*np.sin(w*gt) , A*np.sin((w/2)*gt),0.1))
            xd_dot = np.array((A*w*np.cos(w*gt),A*w/2*np.cos(w/2*gt),0.00))
        elif traj_name=='Moving_Eight_Figure':
            T  = 25*2
            A  = 0.03
            w  = 2*np.pi/T
            xd = np.array(x0+(A*np.sin(w*gt) , A*np.sin((w/2)*gt),0.0002*gt))
            xd_dot = np.array((A*w*np.cos(w*gt),A*w/2*np.cos(w/2*gt),0.0002))
        elif traj_name=='Square':        
            T  = 12.5*2
            tt = gt % (4*T)
            scale = 1

            if (tt<T):
                xd = (x0 + scale*np.array((-0.01+(0.02/T)*tt,0.01,0.0)))
                xd_dot = scale*np.array(((0.02/T),0,0))
            elif (tt<2*T):
                xd = (x0 + scale*np.array((0.01,0.01-((0.02/T)*(tt-T)),0.0)))
                xd_dot = scale*np.array((0,-(0.02/T),0))
            elif (tt<3*T):
                xd = (x0 + scale*np.array((0.01-((0.02/T)*(tt-(2*T))),-0.01,0.0)))
                xd_dot = scale*np.array((-(0.02/T),0,0))
            elif (tt<4*T):
                xd = (x0 + scale*np.array((-0.01,-0.01+((0.02/T)*(tt-(3*T))),0.0)))
                xd_dot = scale*np.array((0,+(0.02/T),0))
            else:
                # t0 = time.time()+5
                gt = 0
        elif traj_name=='Moveing_Square':        
            T  = 10.0
            tt = gt % (4*T)
            if (tt<T):
                xd = (x0 + 2*np.array((-0.01+(0.02/T)*tt,0.01,-0.02+0.0005*gt)))
                xd_dot = 2*np.array(((0.02/T),0,0.0005))
            elif (tt<2*T):
                xd = (x0 + 2*np.array((0.01,0.01-((0.02/T)*(tt-T)),-0.02+0.0005*gt)))
                xd_dot = 2*np.array((0,-(0.02/T),0.0005))
            elif (tt<3*T):
                xd = (x0 + 2*np.array((0.01-((0.02/T)*(tt-(2*T))),-0.01,-0.02+0.0005*gt)))
                xd_dot = 2*np.array((-(0.02/T),0,0.0005))
            elif (tt<4*T):
                xd = (x0 + 2*np.array((-0.01,-0.01+((0.02/T)*(tt-(3*T))),-0.02+0.0005*gt)))
                xd_dot = 2*np.array((0,+(0.02/T),0.0005))
              
        elif traj_name=='Triangle':        
            T  = 12.5 *2
            tt = gt % (4*T)
            scale = 2
            if (tt<T):
                xd = (x0 + scale*np.array((-0.01+(0.02/T)*tt,-0.01+(0.02/T)*tt,0.0)))
                xd_dot = scale*np.array(((0.02/T),(0.02/T),0))
            elif (tt<2*T):
                xd = (x0 + scale*np.array((0.01+(0.02/T)*(tt-(T)),0.01-((0.02/T)*(tt-(T))),0.0)))
                xd_dot = scale*np.array(((0.02/T),-(0.02/T),0))
            elif (tt<4*T):
                xd = (x0 + scale*np.array((0.03-((0.02/T)*(tt-(2*T))),-0.01,0.0)))
                xd_dot = scale*np.array((-(0.02/T),0,0))
            else:
                # t0 = time.time()+5
                gt = 0
        else: # circle
            T  = 50*2
            w  = 2*np.pi/T
            radius = 0.02
            xd = (x0 + np.array((radius*np.sin(w*(gt)),radius*np.cos(w*(gt)),0.00*gt)))
            xd_dot = np.array((radius*w*np.cos(w*(gt)),-radius*w*np.sin(w*(gt)),0.00))
            
        return xd,xd_dot




if __name__ == "__main__":

    saveLog  = True
    addNoise = True
    filteringObs = True
    filteringAct = False
    env = SoftRobotBasicEnvironment()
    ctrl = SoftRobotControl()
    # traj_name = 'Limacon'
    # traj_name = 'Rose'
    # traj_name = 'Eight_Figure'
    # traj_name = 'Moveing_Square'
    # traj_name = 'Square'
    traj_name = 'Helix'
    
    
    
    q = np.array([0.0, -0.0, 0.0])

    ts = 0.05
    tf = 150
    gt = 0
    x0 = np.array((0, 0, 0.1))
    endtip = np.array((0, 0, 0.1))
    actions = np.array((0, 0, 0))
    xc = env.move_robot(q)[:3]
    K = 1*np.diag((2.45, 2.45, 2.45))
    tp = time.time()
    t0 = tp
    ref = None
    
    timestr = time.strftime("%Y%m%d-%H%M%S")
    logFname = "logs/log_" + timestr+"_"+traj_name+".dat"
    logState = np.array([])

    for i in range(int(tf/ts)):
        t = time.time()
        dt = t - tp
        tp = t
        
        xd, xd_dot = get_ref(gt,traj_name)
       
        if ref is None:
            ref = np.copy(xd)
        else:
            ref = np.vstack((ref, xd))
   
        jac = ctrl.Jac(q).T
        pseudo_inverse = np.linalg.pinv(jac)
        qdot = pseudo_inverse @ (xd_dot + np.squeeze((K@(xd-xc)).T))
        q += (qdot * ts)

        ee = env.move_robot(action=q)

        if (filteringAct):
            if gt < 0.01:
                qp = np.copy(q)
            q = 0.75*qp + q * 0.3
            qp = np.copy(q)

        if (addNoise):
            mu, sigma = 0, 0.001
            xc = ee[:3]+np.squeeze(np.random.normal(mu, sigma, (1, 3)))
        else:
            xc = ee[:3]

        if (filteringObs):
            if gt < 0.01:
                xcp = np.copy(xc)
            xc = 0.9*xcp + xc * 0.1
            xcp = np.copy(xc)

        actions = np.vstack((actions, q))
        endtip = np.vstack((endtip, xc))

        if (saveLog):
            dummyLog = np.concatenate((np.array((gt, dt)), np.squeeze(xc), np.squeeze(xd), np.squeeze(
                xd_dot), np.squeeze(qdot), np.array((q[0], q[1], q[2])), np.squeeze(ee)))
            if logState.shape[0] == 0:
                logState = np.copy(dummyLog)
            else:
                logState = np.vstack((logState, dummyLog))

        gt += ts
        print(f"t:{gt:3.3f}\tdt:{dt:3.3f}")


    if (saveLog):
        with open(logFname, "w") as txt:  # creates empty file with header
            txt.write("#l,ux,uy,x,y,z\n")

        np.savetxt(logFname, logState, fmt='%.5f')
        print(f"log file has been saved: {logFname}")



