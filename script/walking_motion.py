import time
import numpy as np
from math import atan2, cos, sin
from pinocchio import centerOfMass, forwardKinematics
from cop_des import CoPDes
from com_trajectory import ComTrajectory
from inverse_kinematics import InverseKinematics
from tools import Constant, Piecewise
 
# Computes the trajectory of a swing foot.
#
# Input data are
#  - initial and final time of the trajectory,
#  - initial and final pose of the foot,
#  - maximal height of the foot, 
#
# The trajectory is polynomial with zero velocities at start and end.
# The orientation of the foot is kept as in intial pose.
class SwingFootTrajectory(object):
    def __init__(self, t_init, t_end, init, end, height):
        assert(init[2] == end[2])
        self.t_init = t_init
        self.t_end = t_end
        self.height = height
        self.init = np.array(init, dtype=float)
        self.end = np.array(end, dtype=float)

    def __call__(self, t):

        # Axe X
        T = np.array([
            [0, 0, 0, 1],
            [(self.t_end-self.t_init)**3, (self.t_end-self.t_init)**2, (self.t_end-self.t_init), 1],
            [0, 0, 1, 0],
            [3*(self.t_end-self.t_init)**2, 2**(self.t_end-self.t_init), 1, 0]
        ])
        X = np.array([self.init[0],self.end[0],0,0])
        A = np.linalg.inv(T)*X

        # Axe Y
        Y = np.array([self.init[1],self.end[1],0,0])
        B = np.linalg.inv(T)*Y

        # Axe Z
        t_mid=(self.t_end-self.t_init)/2
        t_end=self.t_end-self.t_init
        U = np.array([
            [0, 0, 0, 0, 1],
            [(t_end)**4, (t_end)**3, (t_end)**2, (t_end), 1],
            [0, 0, 0, 1, 0],
            [4*(t_end)**3, 3*(t_end)**2, 2**(t_end), 1, 0],
            [(t_mid)**4, (t_mid)**3, (t_mid)**2, (t_mid), 1]
        ])
        Z = np.array([self.init[2],self.end[2],0,0,self.height])
        C = np.linalg.inv(U)*Z

        composanteX = A[3]*(t-self.t_init)**3 + A[2]*(t-self.t_init)**2 + A[1]*(t-self.t_init) + A[0]
        composanteY = B[3]*(t-self.t_init)**3 + B[2]*(t-self.t_init)**2 + B[1]*(t-self.t_init) + B[0]
        composanteZ = C[4]*(t-self.t_init)**4 + C[3]*(t-self.t_init)**3 + C[2]*(t-self.t_init)**2 + C[1]*(t-self.t_init) + C[0]

        return np.array[composanteX,composanteY,composanteZ]

# Computes a walking whole-body motion
#
# Input data are
#  - an initial configuration of the robot,
#  - a sequence of step positions (x,y,theta) on the ground,
#  - a mapping from time to R corresponding to the desired orientation of the
#    waist. If not provided, keep constant orientation.
#
class WalkingMotion(object):
    step_height = 0.05
    single_support_time = .5
    double_support_time = .1 

    def __init__(self, robot):
        self.robot = robot
        

    def compute(self, q0, steps, waistOrientation = None):
        # Test input data
        if len(steps) < 4:
            raise RuntimeError("sequence of step should be of length at least 4 instead of " +
                               f"{len(steps)}")
        # Copy steps in order to avoid modifying the input list.
        steps_ = steps[:]
        # Compute offset between waist and center of mass since we control the center of mass
        # indirectly by controlling the waist.
        data = self.robot.model.createData()
        forwardKinematics(self.robot.model, data, q0)
        com = centerOfMass(self.robot.model, data, q0)
        start_com = np.array([0.,0.])
        end_com = np.array([1.2,0.])
        self.com_trajectory = ComTrajectory(start_com, steps_, end_com, 0.7)
        waist_pose = data.oMi[self.robot.waistJointId]
        com_offset = waist_pose.translation - com
        # Trajectory of left and right feet
        self.lf_traj = Piecewise()
        self.rf_traj = Piecewise()
        start_lf = np.array([0.,0.1,0.])
        start_rf = np.array([0.,-0.1,0.])
        t=0
        for i in range(len(steps_)):
            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,start_lf))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,start_rf))
            t=t+self.double_support_time

            self.lf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, start_lf, steps_[0], self.step_height))
            self.rf_traj.segments.append(Constant(t,t+self.single_support_time,start_rf))
            t=t+self.single_support_time

            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[0]))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,start_rf))
            t=t+self.double_support_time        

            self.lf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[0]))
            self.rf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, start_rf, steps_[1], self.step_height))
            t=t+self.single_support_time

            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[0]))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[1]))
            t=t+self.double_support_time   

            self.lf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[0], steps_[2], self.step_height))
            self.rf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[1]))
            t=t+self.single_support_time   

            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[2]))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[1]))
            t=t+self.double_support_time   

            self.lf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[2]))
            self.rf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[1], steps_[3], self.step_height))
            t=t+self.single_support_time

            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[2]))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[3]))
            t=t+self.double_support_time

            self.lf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[2], steps_[4], self.step_height))
            self.rf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[3]))
            t=t+self.single_support_time

            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[4]))
            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[3]))
            t=t+self.double_support_time

            self.lf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[4]))
            self.rf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[3], steps_[5], self.step_height))
            t=t+self.single_support_time

            self.lf_traj.segments.append(Constant(t,t+10,steps_[4]))
            self.rf_traj.segments.append(Constant(t,t+10,steps_[5]))
            t=t+10

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from talos import Robot
    from pinocchio import neutral
    import numpy as np
    from inverse_kinematics import InverseKinematics
    import eigenpy

    robot = Robot ()
    ik = InverseKinematics (robot)
    ik.rightFootRefPose.translation = np.array ([0, -0.1, 0.1])
    ik.leftFootRefPose.translation = np.array ([0, 0.1, 0.1])
    ik.waistRefPose.translation = np.array ([0, 0, 0.95])

    q0 = neutral (robot.model)
    q0 [robot.name_to_config_index["leg_right_4_joint"]] = .2
    q0 [robot.name_to_config_index["leg_left_4_joint"]] = .2
    q0 [robot.name_to_config_index["arm_left_2_joint"]] = .2
    q0 [robot.name_to_config_index["arm_right_2_joint"]] = -.2
    q = ik.solve (q0)
    robot.display(q)
    wm = WalkingMotion(robot)
    # First two values correspond to initial position of feet
    # Last two values correspond to final position of feet
    steps = [np.array([0, -.1, 0.]), np.array([0.4, .1, 0.]),
             np.array([.8, -.1, 0.]), np.array([1.2, .1, 0.]),
             np.array([1.6, -.1, 0.]), np.array([1.6, .1, 0.])]
    configs = wm.compute(q, steps)
    for q in configs:
        time.sleep(1e-2)
        robot.display(q)
    delta_t = wm.com_trajectory.delta_t
    times = delta_t*np.arange(wm.com_trajectory.N+1)
    lf = np.array(list(map(wm.lf_traj, times)))
    rf = np.array(list(map(wm.rf_traj, times)))
    cop_des = np.array(list(map(wm.com_trajectory.cop_des, times)))
    fig = plt.figure()
    ax1 = fig.add_subplot(311)
    ax2 = fig.add_subplot(312)
    ax3 = fig.add_subplot(313)
    ax1.plot(times, lf[:,0], label="x left foot")
    ax1.plot(times, rf[:,0], label="x right foot")
    ax1.plot(times, cop_des[:,0], label="x CoPdes")
    ax1.legend()
    ax2.plot(times, lf[:,1], label="y left foot")
    ax2.plot(times, rf[:,1], label="y right foot")
    ax2.plot(times, cop_des[:,1], label="y CoPdes")
    ax2.legend()
    ax3.plot(times, lf[:,2], label="z left foot")
    ax3.plot(times, rf[:,2], label="z right foot")
    ax3.legend()
    plt.show()

