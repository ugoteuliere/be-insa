import numpy as np
from numpy.linalg import norm, pinv
from scipy.optimize import fmin_bfgs
from cop_des import CoPDes

# Computation of the trajectory of the center of mass by optimal control
class ComTrajectory(object):
    # time discretization step
    delta_t = 1e-2
    g = 9.81
    N = -1
    # Constructor
    #  - start, end projection in the horizontal plane of the initial and end position of
    #    the center of mass,
    #  - steps, list of 2D vectors representing the successive positions of the feet
    def __init__(self, start, steps, end, z_com):
        self.start = start
        self.end = end
        self.steps = steps
        self.z_com = z_com
 
    def compute(self):
        I2 = np.identity(2)
        n = len(self.steps)
        T = (1+n)*CoPDes.double_support_time + n * CoPDes.single_support_time
        N = int(T/self.delta_t) + 1
        self.N = N
        self.X = np.zeros(2*N)
        
        D = np.zeros((2*N+2,2*N))
        k=0
        for i in range(2*N) :
            for j in range(2*N) :
                if (i==k and j==k) :
                    k=k+1
                    D[i,j]=1
        k=0
        for i in range(2, 2*N + 3):
            for j in range(2*N) :
                if (i==k+2 and j==k) :
                    k=k+1
                    D[i,j]=-1
        

        d0 = np.zeros((2*N + 2, 1))
        d0[0:2, 0] = -start
        d0[-2:, 0] = end

        A = np.identity(2*N) + (self.z_com/(self.g*self.delta_t**2))*(D.T @ D)
        
        cop_des = CoPDes(self.start, self.steps, self.end)
        times = self.delta_t * np.arange(N)
        cop = np.array(list(map(cop_des, times)))
        cop = cop.ravel().reshape(-1, 1)
        b = cop - (self.z_com/(self.g*self.delta_t**2))*(D.T @ d0)

        rows = 2*N
        cols = 2*N - 4
        C = np.zeros((rows, cols))
        C[2:2+cols, :] = np.eye(cols)

        d = np.zeros((2*N, 1))
        d[0:2, 0] = start
        d[-2:, 0] = end
        
        AC = A @ C
        AC_plus = np.linalg.pinv(AC.T @ AC) @ AC.T

        X_barre = (AC_plus) @ (b - A @ d)
        self.X = C @ X_barre + d
        self.X = self.X.reshape(-1)

        return self.X

    # Return projection of center of mass on horizontal plane at time t
    def __call__(self, t):
        if self.N < 0:
            raise RuntimeError("You should call method compute first.")
        i = (int)(t/self.delta_t + .5)
        res  = np.zeros(3)
        res[2] = self.z_com
        if i <= 0:
            res[:2] = self.start
        elif i >= self.N+1:
            res[:2] = self.end
        else:
            res[:2] = self.X[2*i-2:2*i]
        return res

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    start = np.array([0.,0.])
    steps = [np.array([0, .2]), np.array([.4, -.2]), np.array([.8, .2]), np.array([1.2, -.2])]
    end = np.array([1.2,0.])    
    com_trajectory = ComTrajectory(start, steps, end, .7)
    X = com_trajectory.compute()
    times = 0.01 * np.arange(len(X)//2)
    com = np.array(list(map(com_trajectory, times)))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel("time")
    ax.set_ylabel("m")
    ax.plot(times, com[:,0], label="x_com")
    ax.plot(times, com[:,1], label="y_com")
    ax.legend()
    plt.show()
    # com_trajectory.solve()
    # com = np.array(list(map(com_trajectory, times)))
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.set_xlabel("time")
    # ax.set_ylabel("m")
    # ax.plot(times, com[:,0], label="x_com")
    # ax.plot(times, com[:,1], label="y_com")
    # ax.legend()
    # plt.show()
    
