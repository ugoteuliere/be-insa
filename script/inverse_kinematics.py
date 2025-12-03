import numpy as np
import numpy.linalg
from scipy.optimize import fmin_slsqp
from pinocchio import forwardKinematics, log, neutral
import eigenpy

class CallbackLogger:
	def __init__(self):
		self.nfeval = 1
	def __call__(self,x):
		print('===CBK=== {0:4d}   {1}'.format(self.nfeval, x))
		self.nfeval += 1

def normalized_quaternion(q):
	return numpy.linalg.norm(q[3:7]) - 1

# This class computes inverse kinematics by numerical optimization
#
# The input to the inverse kinematic is provided by members
#  - leftFootRefPose
#  - rightFootRefPose
#  - waistRefPose
# that contain an element of type pinocchio SE3.
# Method solve computes a configuration in such a way that the poses of the
# end effector coincide with their reference poses.
class InverseKinematics (object):
	leftFootJoint = 'left_leg_6_joint'
	rightFootJoint = 'right_leg_6_joint'
	waistJoint = 'waist_joint'

	def __init__ (self, robot):
		self.robot = robot
		self.data = self.robot.model.createData()
		q = neutral (robot.model)
		self.fullConfigSize = len(q)
		forwardKinematics(robot.model, self.data, q)
		# Initialize references of feet and center of mass with initial values
		self.leftFootRefPose = self.data.oMi [robot.leftFootJointId].copy ()
		self.rightFootRefPose = self.data.oMi [robot.rightFootJointId].copy ()
		self.waistRefPose = self.data.oMi [robot.waistJointId].copy ()

	def cost (self, q):
		forwardKinematics (self.robot.model, self.data, q)
		errLeftFoot = log(self.leftFootRefPose.inverse() * self.data.oMi[self.robot.leftFootJointId])
		errRightFoot = log(self.rightFootRefPose.inverse() * self.data.oMi[self.robot.rightFootJointId])
		errWaist = log(self.waistRefPose.inverse() * self.data.oMi[self.robot.waistJointId])
		err = np.concatenate ((errLeftFoot.vector,errRightFoot.vector,errWaist.vector))
		return np.inner(err,err)

	def solve (self, q):
		result = q.copy()
		q_opt = fmin_slsqp(self.cost, q, f_eqcons=normalized_quaternion, callback=None)
		return q_opt

if __name__ == "__main__":
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
