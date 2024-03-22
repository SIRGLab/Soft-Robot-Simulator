from cProfile import label
import matplotlib
import numpy as np
import numpy as np
from   scipy.integrate import solve_ivp
import scipy.sparse as sparse
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class SoftRobotControl():
    def __init__(self) -> None:
        """
        Initialize the SoftRobotControl class with default parameters.
        Sets up the initial state of the robot including its length, cable offsets, and initial orientation.
        """
        
        # initial length of robot
        self.l0 = 100e-3
        # cables offset
        self.d  = 7.5e-3
        # ode step time
        self.ds     = 0.005  

        r0 = np.array([0,0,0]).reshape(3,1)  
        R0 = np.eye(3,3)
        R0 = np.reshape(R0,(9,1))
        y0 = np.concatenate((r0, R0), axis=0)
        self.states = np.squeeze(np.asarray(y0))
        self.y0 = np.copy(self.states)

    def Jac(self, q, dq=np.array((1e-4,1e-4,1e-4))):
        """
        Calculates the Jacobian of the system for a given state q and small changes dq in the state.
        This is used for sensitivity analysis and control purposes.
        """
        f = self.runOdeForJac
        fx0 = f(q)
        n   = len(q)
        m   = len(fx0)
        jac = np.zeros((n, m))
        for j in range(m):  # through rows 
            if (j==0):
                Dq = np.array((dq[0]/2.0,0,0))
            elif (j==1):
                Dq = np.array((0,dq[1]/2.0,0))
            elif (j==2):
                Dq = np.array((0,0,dq[2]/2.0))

            jac [j,:] = (f(q+Dq) - f(q-Dq))/dq[j]
        return jac    

    def f(x):
        return np.array([np.sin(x[0]),np.cos(x[1]),np.sin(x[1]+np.cos(x[2]))])


    def odeFunction(self,s,y):
        """
        Defines the differential equations for the soft robot dynamics. It calculates the derivative of the state vector.
        """        
        dydt  = np.zeros(12)
        # % 12 elements are r (3) and R (9), respectively
        e3    = np.array([0,0,1]).reshape(3,1)              
        u_hat = np.array([[0,0,self.uy], [0, 0, -self.ux],[-self.uy, self.ux, 0]])
        r     = y[0:3].reshape(3,1)
        R     = np.array( [y[3:6],y[6:9],y[9:12]]).reshape(3,3)
        # % odes
        dR  = R @ u_hat
        dr  = R @ e3
        dRR = dR.T
        dydt[0:3]  = dr.T
        dydt[3:6]  = dRR[:,0]
        dydt[6:9]  = dRR[:,1]
        dydt[9:12] = dRR[:,2]
        return dydt.T

    def runOdeForJac(self,q):    
        """
        Solves the ODE to find the robot's state given the input parameters. This is specifically used to assist in calculating the Jacobian.
        """           
        l  = self.l0 + q[0]
        self.uy = (q[1]) / (l *self.d)
        self.ux = (q[2]) / -(l*self.d)

        cableLength          = (0,l)
        t_eval               = np.linspace(0, l, int(l/self.ds))
        sol                  = solve_ivp(self.odeFunction,cableLength,self.y0,t_eval=t_eval)
        self.states          = np.squeeze(np.asarray(sol.y[:,-1]))
        #print ("t: {0}, states: {1}".format(self.sim_time,self.states))
        return self.states[0:3]

    def visualize(self,state):
        """
        Visualizes the robot's trajectory in 3D space using matplotlib.
        """        
        fig = plt.figure()
        ax  = fig.add_subplot(projection='3d')
        ax.scatter (state[:,0],state[:,1],state[:,2], c = 'g', label='robot')
        ax.plot3D(state[:,0], state[:,1],state[:,2], c='r',lw=2)

        
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_xlim(-0.08, 0.08)
        ax.set_ylim(-0.06, 0.06)
        ax.set_zlim(-0.05, 0.15)
        
        ax.legend()
        
        plt.show()


class ODE():
    def __init__(self) -> None:
        self.l  = 0
        self.uy = 0
        self.ux = 0
        self.dp = 0
        self.err = np.array((0,0,0))
        self.errp = np.array((0,0,0))

        self.simCableLength  = 0
        # initial length of robot
        self.l0 = 100e-3
        # cables offset
        self.d  = 7.5e-3
        # ode step time
        self.ds     = 0.005 #0.0005  
        r0 = np.array([0,0,0]).reshape(3,1)  
        R0 = np.eye(3,3)
        R0 = np.reshape(R0,(9,1))
        y0 = np.concatenate((r0, R0), axis=0)
        self.states = np.squeeze(np.asarray(y0))
        self.y0 = np.copy(self.states)


    def updateAction(self,action):
        self.l  = self.l0 + action[0]
        # self.l  = action0
        
        self.uy = (action[1]) /  (self.l * self.d)
        self.ux = (action[2]) / -(self.l * self.d)


    def odeFunction(self,s,y):
        dydt  = np.zeros(12)
        # % 12 elements are r (3) and R (9), respectively
        e3    = np.array([0,0,1]).reshape(3,1)              
        u_hat = np.array([[0,0,self.uy], [0, 0, -self.ux],[-self.uy, self.ux, 0]])
        r     = y[0:3].reshape(3,1)
        R     = np.array( [y[3:6],y[6:9],y[9:12]]).reshape(3,3)
        # % odes
        dR  = R @ u_hat
        dr  = R @ e3
        dRR = dR.T
        dydt[0:3]  = dr.T
        dydt[3:6]  = dRR[:,0]
        dydt[6:9]  = dRR[:,1]
        dydt[9:12] = dRR[:,2]
        return dydt.T


    def odeStepFull(self):        
        cableLength          = (0,self.l)
        
        t_eval               = np.linspace(0, self.l, int(self.l/self.ds))
        sol                  = solve_ivp(self.odeFunction,cableLength,self.y0,t_eval=t_eval)
        self.states          = np.squeeze(np.asarray(sol.y[:,-1]))
        return sol.y


class softRobotVisualizer():
    def __init__(self,obsEn = False) -> None:
       
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.title = self.ax.set_title('Visualizer-1.01')
        self.xlabel = self.ax.set_xlabel("x (m)")
        self.ylabel = self.ax.set_ylabel("y (m)")
        self.zlabel = self.ax.set_zlabel("z (m)")
        self.ax.set_xlim([-0.08,0.08])
        self.ax.set_ylim([-0.08,0.08])
        self.ax.set_zlim([-0.0,0.15])
        self.speed = 1 
        
        self.actions = None
        self.endtips = None

        self.ode = ODE()       
        self.robot = self.ax.scatter([], [], [],marker='o',lw=5)
        self.robotBackbone, = self.ax.plot([], [], [],'r',lw=4)



    def update_graph(self,num):
        
        if self.actions is None:
            self.ode.updateAction(np.array((0+num/100,num/1000,num/1000)))
        else:
            self.ode.updateAction(self.actions[int(num*self.speed),:])

        self.sol = self.ode.odeStepFull()
        
        self.robot._offsets3d = (self.sol[0,:], self.sol[1,:], self.sol[2,:])
        self.robotBackbone.set_data(self.sol[0,:], self.sol[1,:])
        self.robotBackbone.set_3d_properties(self.sol[2,:])

        
        
        
if __name__ == "__main__":
    env = SoftRobotControl()
 
    q = np.array([0.0,-0.0,0.0])
    x0 = np.array([0, 0, 0.1])
    xdot = np.array([0.0,0.0,0.])

    ts = 0.05
    tf = 10
    logState = np.array([])
    refTraj = np.array([])
    

    for t in range(int(tf/ts)):        
        gt = t*ts
        T  = 10
        w  = 2*np.pi/T
        radius = 0.02
        xd = x0 + np.array([radius*np.cos(w*(gt)), radius*np.sin(w*(gt)),0.00])
        xd_dot = np.array([-radius*w*np.sin(w*(gt)),radius*w*np.cos(w*(gt)),0.00])
        
        jac = env.Jac(q).T
        pseudo_inverse = np.linalg.pinv(jac)
        qdot = pseudo_inverse @ xd_dot        
        q += (qdot * ts)
        
        if t==0:
            logState = np.copy(q)
            refTraj = np.copy(xd)
        else:
            logState =  np.vstack((logState,q))
            refTraj =  np.vstack((refTraj,xd))
            
        
        print (f"t = {gt:2.2f}")

    sfVis = softRobotVisualizer()
    sfVis.actions = logState
    
    ani = animation.FuncAnimation(sfVis.fig, sfVis.update_graph, logState.shape[0], interval=10, blit=False)

    plt.show()
        








