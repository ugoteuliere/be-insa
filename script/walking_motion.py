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
        # On s'assure que les entrées sont des floats
        self.init = np.array(init, dtype=float)
        self.end = np.array(end, dtype=float)
        
        # Vérification Z
        assert(abs(self.init[2] - self.end[2]) < 1e-6)
        
        self.t_init = t_init
        self.t_end = t_end
        self.height = height

    def __call__(self, t):
        # On borne t pour éviter des divergences si on dépasse légèrement les temps
        if t < self.t_init: t = self.t_init
        if t > self.t_end: t = self.t_end
        
        dt = t - self.t_init
        T_duration = self.t_end - self.t_init

        # --- AXE X ---
        # Matrice pour polynôme degré 3 : [a3, a2, a1, a0]
        Tx = np.array([
            [0, 0, 0, 1],                         # Pos à t=0
            [T_duration**3, T_duration**2, T_duration, 1], # Pos à t=fin
            [0, 0, 1, 0],                         # Vitesse à t=0
            [3*T_duration**2, 2*T_duration, 1, 0] # Vitesse à t=fin
        ])
        X = np.array([self.init[0], self.end[0], 0, 0])
        
        # CORRECTION 1 : Utilisation de @ pour la multiplication matricielle
        A = np.linalg.inv(Tx) @ X 
        
        # CORRECTION 2 : Bon ordre des coefficients (A[0] est devant t^3)
        x_val = A[0]*dt**3 + A[1]*dt**2 + A[2]*dt + A[3]

        # --- AXE Y ---
        Y_vec = np.array([self.init[1], self.end[1], 0, 0])
        # On réutilise la même matrice Tx car les contraintes temporelles sont les mêmes
        B = np.linalg.inv(Tx) @ Y_vec
        y_val = B[0]*dt**3 + B[1]*dt**2 + B[2]*dt + B[3]

        # --- AXE Z (Degré 4 pour monter et descendre) ---
        t_mid = T_duration / 2
        Tz = np.array([
            [0, 0, 0, 0, 1],                      # Pos init
            [T_duration**4, T_duration**3, T_duration**2, T_duration, 1], # Pos fin
            [0, 0, 0, 1, 0],                      # Vit init
            [4*T_duration**3, 3*T_duration**2, 2*T_duration, 1, 0],       # Vit fin
            [t_mid**4, t_mid**3, t_mid**2, t_mid, 1]  # Passage par height au milieu
        ])
        Z_vec = np.array([self.init[2], self.end[2], 0, 0, self.height+0.1])
        
        C = np.linalg.inv(Tz) @ Z_vec
        
        z_val = C[0]*dt**4 + C[1]*dt**3 + C[2]*dt**2 + C[3]*dt + C[4]

        return np.array([x_val, y_val, z_val])

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
        start_rf = np.array ([0, -0.1, 0.1])
        start_lf = np.array ([0, 0.1, 0.1])
        # Compute offset between waist and center of mass since we control the center of mass
        # indirectly by controlling the waist.
        data = self.robot.model.createData()
        forwardKinematics(self.robot.model, data, q0)
        com = centerOfMass(self.robot.model, data, q0)
        start_com = np.array([(start_rf[0]+start_lf[0])/2,start_lf[0]])
        end_com = np.array([(steps_[-2][0]+steps_[-1][0])/2,0.])
        self.com_trajectory = ComTrajectory(start_com, steps_, end_com, 0.7)
        waist_pose = data.oMi[self.robot.waistJointId]
        com_offset = waist_pose.translation - com
        # Trajectory of left and right feet
        self.lf_traj = Piecewise()
        self.rf_traj = Piecewise()  
        
        t=0
        self.rf_traj.segments.append(Constant(t,t+self.double_support_time,start_rf))
        self.lf_traj.segments.append(Constant(t,t+self.double_support_time,start_lf))
        t=t+self.double_support_time

        self.rf_traj.segments.append(Constant(t,t+self.single_support_time,start_rf))
        self.lf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, start_lf, steps_[1], self.step_height))
        t=t+self.single_support_time
        
        for i in (0,len(steps_)-4):
            self.rf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[i], steps_[i+2], self.step_height))
            self.lf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[i+1]))
            t=t+self.single_support_time

            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[i+2]))
            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[i+1]))
            t=t+self.double_support_time        

            self.rf_traj.segments.append(Constant(t,t+self.single_support_time,steps_[i+2]))
            self.lf_traj.segments.append(SwingFootTrajectory(t, t+self.single_support_time, steps_[i+1], steps_[i+3], self.step_height))
            t=t+self.single_support_time

            self.rf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[i+2]))
            self.lf_traj.segments.append(Constant(t,t+self.double_support_time,steps_[i+3]))
            t=t+self.double_support_time 

        self.com_trajectory.compute()
        delta_t = self.com_trajectory.delta_t
        times = delta_t*np.arange(self.com_trajectory.N+1) 
        com_values = list(map(self.com_trajectory, times))
        configs = []
        
        ik = InverseKinematics(self.robot)
        q = q0.copy()
        initial_lf_rot = ik.leftFootRefPose.rotation.copy()
        initial_rf_rot = ik.rightFootRefPose.rotation.copy()
        initial_waist_rot = ik.waistRefPose.rotation.copy()

        # Boucle sur le temps
        for i, t in enumerate(times):
            
            # --- A. Récupération des cibles à l'instant t ---
            target_lf = self.lf_traj(t)
            target_rf = self.rf_traj(t)
            target_com_xy = com_values[i] 
            
            # --- B. Mise à jour des références IK ---
            # 1. Pied Gauche
            ik.leftFootRefPose.translation = target_lf
            ik.leftFootRefPose.rotation = initial_lf_rot
            # 2. Pied Droit
            ik.rightFootRefPose.translation = target_rf
            ik.rightFootRefPose.rotation = initial_rf_rot
            
            # 3. Taille (Waist)
            waist_translation = np.array([
                target_com_xy[0] + com_offset[0], 
                target_com_xy[1] + com_offset[1], 
                waist_pose.translation[2] # On garde la hauteur Z initiale de la taille
            ])
            ik.waistRefPose.translation = waist_translation
            ik.waistRefPose.rotation = initial_waist_rot # Ou mettre à jour selon waistOrientation si fourni

            # --- C. Résolution ---
            q = ik.solve(q)
            configs.append(q)

        return configs

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
    q = ik.solve(q0)
    robot.display(q)
    wm = WalkingMotion(robot)
    # First two values correspond to initial position of feet
    # Last two values correspond to final position of feet
    steps = [np.array([0, -.1, 0.1]), np.array([0.4, .1, 0.1]),
             np.array([.8, -.1, 0.1]), np.array([1.2, .1, 0.1]),
             np.array([1.6, -.1, 0.1]), np.array([1.6, .1, 0.1])]
    configs = wm.compute(q, steps, None)
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

