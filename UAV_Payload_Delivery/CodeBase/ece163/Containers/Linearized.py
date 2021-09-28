"""
File contains the classes for linearized flight models in both transfer function and state space variants of the linearized
models. Models are created about a known trim condition, most often straight and level flight, but any combination of flight
path angle and constant radius turn can be used as a trim.

Perturbation inputs and states are about the nominal trim states, thus the deltaE is the change in elevator about the nominal
elevator angle for trim.
"""

import math
import ece163.Constants.VehiclePhysicalConstants as VPC

testingAbs_tol = 1e-6

class transferFunctions:
	def __init__(self, Va_trim = 0.0, alpha_trim = 0.0, beta_trim = 0.0, gamma_trim = 0.0, theta_trim = 0.0, phi_trim = 0.0, a_phi1 = 0.0, a_phi2 = 0.0, a_beta1 = 0.0, a_beta2 = 0.0, a_theta1 = 0.0, a_theta2 = 0.0, a_theta3 = 0.0, a_V1 = 0.0, a_V2 = 0.0, a_V3 = 0.0):
		"""
		Defines the parameters that will be used to implement the transfer function versions of control loops (using
		successive loop closure). See Beard Chapter 5 for details.
		"""
		self.Va_trim = Va_trim
		self.alpha_trim = alpha_trim
		self.beta_trim = beta_trim
		self.gamma_trim = gamma_trim
		self.theta_trim = theta_trim
		self.phi_trim = phi_trim
		self.a_phi1 = a_phi1
		self.a_phi2 = a_phi2
		self.a_beta1 = a_beta1
		self.a_beta2 = a_beta2
		self.a_theta1 = a_theta1
		self.a_theta2 = a_theta2
		self.a_theta3 = a_theta3
		self.a_V1 = a_V1
		self.a_V2 = a_V2
		self.a_V3 = a_V3
		return

	def __str__(self):
		return "{0.__name__}(Va={1.Va_trim}, alpha={1.alpha_trim}, beta={1.beta_trim}, gamma={1.gamma_trim}, " \
			   "theta={1.theta_trim}, phi={1.phi_trim}, a_phi1={1.a_phi1}, a_phi2={1.a_phi2}, a_beta1={1.a_beta1}," \
			   "a_beta2={1.a_beta2}, a_theta1={1.a_theta1}, a_theta2={1.a_theta2}, a_theta3={1.a_theta3}, a_V1={1.a_V1}, " \
			   "a_V2={1.a_V2}, a_V3={1.a_V3})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in ['Va_trim', 'alpha_trim', 'beta_trim',
																								'gamma_trim', 'theta_trim', 'phi_trim',
																								'a_phi1', 'a_phi2', 'a_beta1', 'a_beta2',
																								'a_theta1', 'a_theta2', 'a_theta3',
																								'a_V1', 'a_V2', 'a_V3']]):
				return False
			else:
				return True
		else:
			return NotImplemented


class stateSpace:
	def __init__(self):
		"""
		Defines the parameters that will be used to implement the state space versions of control loops (using LQR or pole
		placement techniques). See Beard Chapter 5 for details. Lateral state is [v, p, r, roll, yaw]^T and Longitudinal
		state is [u, w, q, pitch, height]. Height is used rather than down traditionally.
		"""
		self.Va_trim = 0.0
		self.alpha_trim = 0.0
		self.beta_trim = 0.0
		self.gamma_trim = 0.0
		self.theta_trim = 0.0
		self.phi_trim = 0.0
		self.A_longitudinal = [[0.0, 0.0, 0.0, 0.0, 0.0],
							   [0.0, 0.0, 0.0, 0.0, 0.0],
							   [0.0, 0.0, 0.0, 0.0, 0.0],
							   [0.0, 0.0, 0.0, 0.0, 0.0],
							   [0.0, 0.0, 0.0, 0.0, 0.0]]

		self.B_longitudinal = [[0.0, 0.0],
							   [0.0, 0.0],
							   [0.0, 0.0],
							   [0.0, 0.0],
							   [0.0, 0.0]]

		self.A_lateral = [[0.0, 0.0, 0.0, 0.0, 0.0],
						  [0.0, 0.0, 0.0, 0.0, 0.0],
						  [0.0, 0.0, 0.0, 0.0, 0.0],
						  [0.0, 0.0, 0.0, 0.0, 0.0],
						  [0.0, 0.0, 0.0, 0.0, 0.0]]

		self.B_lateral = [[0.0, 0.0],
						  [0.0, 0.0],
						  [0.0, 0.0],
						  [0.0, 0.0],
						  [0.0, 0.0]]
		return

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all([math.isclose(getattr(self, member), getattr(other, member)) for member in ['Va_trim', 'alpha_trim',
																								   'beta_trim', 'gamma_trim',
																								   'theta_trim', 'phi_trim']]):
				return False
			for myRow, otherRow in zip(self.A_longitudinal, other.A_longitudinal):
				if not all([math.isclose(x, y) for x, y in zip(myRow, otherRow)]):
					return False
			for myRow, otherRow in zip(self.B_longitudinal, other.B_longitudinal):
					if not all([math.isclose(x, y) for x, y in zip(myRow, otherRow)]):
						return False
			for myRow, otherRow in zip(self.A_lateral, other.A_lateral):
				if not all([math.isclose(x, y) for x, y in zip(myRow, otherRow)]):
					return False
			for myRow, otherRow in zip(self.B_lateral, other.B_lateral):
					if not all([math.isclose(x, y) for x, y in zip(myRow, otherRow)]):
						return False
			return True
		else:
			return NotImplemented










