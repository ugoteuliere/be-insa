import time
import numpy as np
from math import atan2, cos, sin
from pinocchio import forwardKinematics
from talos import Robot
from scipy.optimize import fmin_slsqp
from cop_des import CoPDes
from walking_motion import WalkingMotion

def normalized_quaternion(q):
	return np.linalg.norm(q[3:7]) - 1

class Bezier(object):
    """
    Bezier curve with any number of control points
    Evaluation is performed with de Casteljau algorithm.
    """
    def __init__(self, controlPoints):
        self.controlPoints = controlPoints

    def __call__(self, t):
        cp = self.controlPoints[:]
        while len(cp) > 1:
            cp1 = list()
            for p0, p1 in zip(cp, cp[1:]):
                cp1.append((1-t)*p0 + t*p1)
            cp = cp1[:]
        return cp[0]

    def derivative(self):
        """
        Return the derivative as a new Bezier curve
        """
        n = len(self.controlPoints) - 1
        cp = list()
        for P0, P1 in zip(self.controlPoints, self.controlPoints[1:]):
            cp.append(n*(P1-P0))
        return Bezier(cp)

def simpson(f, t_init, t_end, n_intervals):
    """
    Computation of an integral with Simpson formula
    """
    l = (t_end - t_init)/n_intervals
    t0 = t_init
    res = f(t0)/6
    for i in range(n_intervals):
        t1 = t0 + .5*l
        t2 = t0 + l
        res += 2/3*f(t1) + 1/3*f(t2)
        t0 = t2
    res -= f(t_end)/6
    res *= l
    return res

class Integrand(object):
    """
    Computes the integrand defining the integral cost for a given Bezier curve
    and a given parameter t as

         1     2           2
    I = --- (v   + alpha v  )
         2     T           N

    where
      - v  and v  are the tangent and normal velocities.
         T      N
    """
    alpha = 8
    def __init__(self, bezier):
        self.function = bezier
        self.derivative = bezier.derivative()

    def __call__(self, t):
        alpha = 8
        theta = self.function(t)[3]
        return (1/2) * ((np.dot((cos(theta),sin(theta),0),self.derivative(t)))**2 + alpha*(np.dot((-sin(theta),cos(theta),0),self.derivative(t)))**2 )

class SlidingMotion(object):
    """
    Defines a sliding motion of the robot using Bezier curve and minimizing
    an integral cost favoring forward motions
    """
    beta = 100
    stepLength = .25
    def __init__(self, robot, q0, end):
        """ Constructor

        - input: q0 initial configuration of the robot,
        - end: end configuration specified as (x, y, theta) for the position
                and orientation in the plane.
        """
        pass

    def cost(self, B):
        """
        Compute the cost of a trajectory represented by a Bezier curve
        """
        assert(len(B.shape) == 1)
        boundary_cost = 
        beta = 100
        return = simpson(Integrand(B),t) + beta*boundary_cost


    def boundaryConstraints(self, X):
        """
        Computes the scalar product of the x-y velocity at the beginning 
        (resp. at the end) of the trajectory with the unit vector normal to
        the initial (resp. end) orientation.
        """

    def solve(self):
        """
        Solve the optimization problem. Initialize with a straight line
        """
        control_points = fmin_slsqp(self.cost, q, f_eqcons=normalized_quaternion, iprint=-1, callback=None)
        return control_points

    def leftFootPose(self, pose):
        """
        Compute the desired pose of the left foot given the values (x,y,theta)
        contained in the input np.array pose.
        """
        res = np.zeros(3)
        return res

    def rightFootPose(self, pose):
        """
        Compute the desired pose of the right foot given the values (x,y,theta)
        contained in the input np.array pose.
        """
        res = np.zeros(3)
        return res


    def computeMotion(self):
        configs = list()
        return configs
        
if __name__ == '__main__':
    from talos import Robot
    robot = Robot()
    q0 = np.array([
        0.00000000e+00, 0.00000000e+00, 9.50023790e-01, 3.04115703e-04,
        0.00000000e+00, 0.00000000e+00, 9.99999957e-01, 0.00000000e+00,
        2.24440496e-02, -5.88127845e-01, 1.21572430e+00, -6.27580400e-01,
        -2.29184434e-02, 0.00000000e+00, -2.95804462e-02, -5.88175279e-01,
        1.21608861e+00, -6.27902977e-01, 2.91293666e-02, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, 2.00000000e-01, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, -2.00000000e-01, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, 0.00000000e+00])

    end = np.array([2, 1, 1.57])
    sm = SlidingMotion(robot, q0, end)
    configs = sm.computeMotion()
    for q in configs:
        time.sleep(1e-2)
        robot.display(q)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)
    times = 1e-2*np.arange(101)
    X = np.array(list(map(sm.slidingPath, times)))
    ax1.plot(X[:,0], X[:,1], label="x-y path")
    ax2.plot(times, X[:,2])
    plt.show()
