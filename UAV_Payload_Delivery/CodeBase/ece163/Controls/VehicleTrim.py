"""
Vehicle Trim: for a flight vehicle, there is a climbing or descending spiral that is perfectly balanced, such that there
is no change in the derivative of the vehicle when at that state. The trim is defined by an input airspeed, Va*, an input
turn radius, R*, and an input climb angle, gamma*.

The cost function being minimized is: J = || \dot{x}* - f(x*,u*) || where f(x*, u*) are the derivative states directly
from the trim conditions. The minimization is to find angle of attack, alpha*, sideslip angle, beta*, and roll angle, phi*
corresponding to the trim states. Minimization is done with a simple gradient descent, and gradient is calculated
numerically.
"""

import math
import sys

from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations
from ece163.Constants import VehiclePhysicalConstants as VPC
import numpy
from scipy.optimize import minimize


class VehicleTrim():
	def __init__(self):
		"""
		Set of functions to compute the trim condition of the non-linear differential equations numerically. Class includes
		the trim states (and derivatives of the state) along with the trim controls within the structure of the class.
		"""
		self.VehicleTrimModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
		self.VehicleTrimModel.getWindModel().reset()
		self.ControlTrim = Inputs.controlInputs()
		return

	def getTrimState(self):
		"""
		Wrapper function to return the trim vehicle state (such that all of the derivatives are zero except for the two
		that are not (\dot{pd} and \dot{yaw}).

		:return: state: the trim state of the vehicle
		"""
		return self.VehicleTrimModel.getVehicleState()

	def getTrimControls(self):
		"""
		Wrapper function to return the trim control inputs

		:return: controlInputs
		"""
		return self.ControlTrim

	# def exportTrim(self, filepath=defaultTrimFilename):
	# 	"""
	# 	Wrapper function to export the trim state and trim control inputs to a pickle located in the filepath. Pickle will
	# 	contain the trim vehicle state, and the trim input controls. Use pickle.load(f) to load the state, and again to load
	# 	the inputs.
	#
	# 	:param filepath: path locates file for storing
	# 	:return: none
	# 	"""
	# 	trimState = self.VehicleTrimModel.getVehicleState()
	# 	trimControls = self.ControlTrim
	# 	with open(filepath, 'wb') as f:
	# 		pickle.dump((trimState, trimControls), f)
	# 	return

	def computeTrim(self, Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0):
		"""
		Function to compute trim and find the minimum using the inputs and the scipi optimization toolbox rather than do
		it by hand. Will need to put the entire thing into a numpy array for it to work (and need to use the minimization
		toolbox). We use kappa, the inverse of the radius, so that a straight line corresponds to kappa = 0, rather than
		R* = infinity. The state is contained inside the VehicleAerodynamicsModel.VehicleDynamicsModel.state.

		At the last step (once the trim optimization has been completed), the north, east, and down portions of the state
		are overwritten with those contained in VehiclePhysicalParameters file. Lastly, the control are checked against the
		min and max valid ranges also contained within VehiclePhysicalParameters as minControls and maxControls; if the trim
		values are out of range, the the function returns False.

		:param Vastar: trim airspeed in [m/s]
		:param kappastar: trim curvature (1/R*) in [1/m], use negative for CCW turn
		:param gammastar: trim climb angle [rad]
		:return: True or False (True if control inputs in range, False if not)
		"""

		self.VehicleTrimModel.getVehicleDynamicsModel().reset()
		self.VehicleTrimModel.getWindModel().reset()

		if any([math.isclose(Kappastar, 0.0), math.fabs(Kappastar) < (1/VPC.RstarMax)]):
			phistar = 0.0
			pstar = 0.0
			qstar = 0.0
			rstar = 0.0
			cons = ({'type': 'eq',
					 'fun': lambda x: numpy.array([
						 x[3] ** 2 + x[4] ** 2 + x[5] ** 2 - Vastar ** 2,  # magnitude of velocity vector is Va
						 x[4],  # v=0, force side slip angle to be zero
						 x[6],  # yaw = 0, force yaw to be zero
						 x[8],	# force roll angle to zero
						 x[9],  # force p to zero
						 x[10],	# force q to zero
						 x[11],	# force r to zero
						 x[13], # force ailerons to zero
						 x[15], # force rudder to zero
					 ]),
					 'jac': lambda x: numpy.array([
						 [0., 0., 0., 2 * x[3], 2 * x[4], 2 * x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],	#x[3]
						 [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],						#x[4]
						 [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],						#x[6]
						 [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],						#x[8]
						 [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],						#x[9]
						 [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],						#x[10]
						 [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],						#x[11]
						 [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],						#x[13]
						 [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],						#x[15]
					 ])
					 })
		else:
			phistar = math.copysign(math.atan(Vastar ** 2 * Kappastar * math.cos(Gammastar) / VPC.g0), Kappastar)
			pstar = -Vastar * Kappastar * math.sin(Gammastar)
			qstar = Vastar * Kappastar * math.sin(phistar) * math.cos(Gammastar)
			rstar = Vastar * Kappastar * math.cos(phistar) * math.cos(Gammastar)
			cons = ({'type': 'eq',
					 'fun': lambda x: numpy.array([
						 x[3] ** 2 + x[4] ** 2 + x[5] ** 2 - Vastar ** 2,  # magnitude of velocity vector is Va
						 x[4],  # v=0, force side slip angle to be zero
						 x[6],  # yaw = 0, force yaw to be zero
						 x[8] - phistar,  # force matching roll angle
						 x[9] + Vastar * Kappastar * math.sin(x[7]),  # force p to match in the case of turning
						 x[10] - Vastar * Kappastar * math.sin(x[8]) * math.cos(Gammastar),
						 # force q to match in the case of turning
						 x[11] - Vastar * Kappastar * math.cos(x[8]) * math.cos(Gammastar),
						 # force r to match in the case of turning
					 ]),
					 'jac': lambda x: numpy.array([
						 [0., 0., 0., 2 * x[3], 2 * x[4], 2 * x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
						 [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
						 [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
						 [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
						 [0., 0., 0., 0., 0., 0., 0., Vastar * Kappastar * math.cos(x[7]), 0., 1., 0., 0., 0., 0., 0.,
						  0.],
						 [0., 0., 0., 0., 0., 0., 0., 0., -Vastar * Kappastar * math.cos(Gammastar) * math.cos(x[8]),
						  0., 1., 0., 0., 0., 0., 0.],
						 [0., 0., 0., 0., 0., 0., 0., 0., Vastar * Kappastar * math.cos(Gammastar) * math.sin(x[8]), 0.,
						  0., 1., 0., 0., 0., 0.],
					 ])
					 })

		x0 = numpy.array([[self.VehicleTrimModel.getVehicleState().pn],	#pn [0]
					   [self.VehicleTrimModel.getVehicleState().pe],	#pe [1]
					   [self.VehicleTrimModel.getVehicleState().pd],	#pd [2]
					   [Vastar],											#u	[3]
					   [0.0],												#v	[4]
					   [0.0],												#w	[5]
					   [0.0],											#yaw	[6]
					   [Gammastar],										#pitch	[7]
					   [phistar],										#roll	[8]
					   [pstar],												#p	[9]
					   [qstar],												#q	[10]
					   [rstar],												#r	[11]
					   [self.ControlTrim.Throttle],					#Throttle	[12]
					   [self.ControlTrim.Aileron],					#Aileron	[13]
					   [self.ControlTrim.Elevator],					#Elevator	[14]
					   [self.ControlTrim.Rudder]])						#Rudder	[15]


		# solve the minimization problem to find the trim states and inputs
		res = minimize(self.trim_objective_fun, x0, method='SLSQP', args=(Vastar, Kappastar, Gammastar),
					   constraints=cons, options={'ftol': 1e-10, 'disp': False})
		self.MapArraytoClass(res.x)
		# replace parts of state with initial values
		self.VehicleTrimModel.getVehicleState().pn = VPC.InitialNorthPosition
		self.VehicleTrimModel.getVehicleState().pe = VPC.InitialEastPosition
		self.VehicleTrimModel.getVehicleState().pd = VPC.InitialDownPosition
		self.VehicleTrimModel.getVehicleState().yaw = VPC.InitialYawAngle
		# check input limits
		if any([self.ControlTrim.Throttle < VPC.minControls.Throttle,
				self.ControlTrim.Aileron < VPC.minControls.Aileron,
				self.ControlTrim.Elevator < VPC.minControls.Elevator,
				self.ControlTrim.Rudder < VPC.minControls.Rudder,
				self.ControlTrim.Throttle > VPC.maxControls.Throttle,
				self.ControlTrim.Aileron > VPC.maxControls.Aileron,
				self.ControlTrim.Elevator > VPC.maxControls.Elevator,
				self.ControlTrim.Rudder > VPC.maxControls.Rudder]):
			return False
		return True

	def trim_objective_fun(self, x, Vastar, Kappastar, Gammastar):
		"""
		Objective function for the minimization process. J = || f(x*,u*) - \dot(x)* ||^2. The function f(x*,u*) is the
		derivative given the state and control surface inputs.

		:param x: numpy array of state and controls concatenated in one column
		:param Vastar: desired trim airspeed [m/s]
		:param Kappastar: desired turn radius (1/R*) in [1/m], use negative kappa for CCW turns
		:param Gammastar: desired flight path angle [rad]
		:return: J, cost function
		"""
		# objective function to be minimized
		self.MapArraytoClass(x)
		Faero = self.VehicleTrimModel.updateForces(self.VehicleTrimModel.getVehicleState(),
							   self.ControlTrim,
							   wind = self.VehicleTrimModel.getWindModel().getWind())
		dot = self.VehicleTrimModel.getVehicleDynamicsModel().derivative(
			self.VehicleTrimModel.getVehicleState(), Faero)
		self.VehicleTrimModel.getVehicleDynamicsModel().setVehicleDerivative(dot)
		xdotstar_pd = -Vastar * math.sin(Gammastar)
		xdotstar_yaw = Vastar * Kappastar * math.cos(Gammastar)
		J = math.hypot(dot.pd - xdotstar_pd,
					   dot.u,
					   dot.v,
					   dot.w,
					   dot.roll,
					   dot.pitch,
					   dot.yaw - xdotstar_yaw,
					   dot.p,
					   dot.q,
					   dot.r)
		return J ** 2

	def MapArraytoClass(self, x):
		"""
		Helper function to map the state vector (numpy array) used in the minimization to the state and controlInput classes
		that are held in self.

		:param x: state + input vector (numpy array)
		:return: none
		"""
		self.VehicleTrimModel.getVehicleState().pn = x.item(0)
		self.VehicleTrimModel.getVehicleState().pe = x.item(1)
		self.VehicleTrimModel.getVehicleState().pd = x.item(2)
		self.VehicleTrimModel.getVehicleState().u = x.item(3)
		self.VehicleTrimModel.getVehicleState().v = x.item(4)
		self.VehicleTrimModel.getVehicleState().w = x.item(5)
		self.VehicleTrimModel.getVehicleState().yaw = x.item(6)
		self.VehicleTrimModel.getVehicleState().pitch = x.item(7)
		self.VehicleTrimModel.getVehicleState().roll = x.item(8)
		self.VehicleTrimModel.getVehicleState().R = \
			Rotations.euler2DCM(self.VehicleTrimModel.getVehicleState().yaw,
								self.VehicleTrimModel.getVehicleState().pitch,
								self.VehicleTrimModel.getVehicleState().roll)
		self.VehicleTrimModel.getVehicleState().p = x.item(9)
		self.VehicleTrimModel.getVehicleState().q = x.item(10)
		self.VehicleTrimModel.getVehicleState().r = x.item(11)

		self.VehicleTrimModel.getVehicleState().Va = math.hypot(x.item(3), x.item(4), x.item(5))  # Airspeed
		self.VehicleTrimModel.getVehicleState().alpha = math.atan2(x.item(5), x.item(3))  # angle of attack
		self.VehicleTrimModel.getVehicleState().beta = math.atan2(x.item(4), x.item(3))  # sideslip angle

		self.ControlTrim.Throttle = x.item(12)
		self.ControlTrim.Aileron = x.item(13)
		self.ControlTrim.Elevator = x.item(14)
		self.ControlTrim.Rudder = x.item(15)
		return

	def GenerateIdealPath(self, Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0, tFinal=100, tStep=0.1):
		"""
		Function to generate the expected trim trajectory for the climbing spiral. Generates points at each tStep until
		tFinal, with the exception that for straight lines (Kappastar = 0), only the first and last points are generated
		since that is all that is needed to draw the line. The path will start at the trim state initial position and
		orientation (yaw). Points are returned as a standard matrix (list of lists) in row, column major with the data
		represented as [[North, East, Down]_0, [North, East, Down]_2, ..., [North, East, Down]_n]

		:param Vastar: trim airspeed in [m/s]
		:param kappastar: trim curvature (1/R*) in [1/m], use negative for CCW turn
		:param gammastar: trim climb angle [rad]
		:param tFinal: final time to extend curve out to
		:param tStep: time step to generate points (one NED set at each tStep)
		:return: points: list of lists of NED points (e.g. matrix of points)
		"""



		if any([math.isclose(Kappastar, 0.0), math.fabs(Kappastar) < (1.0 / VPC.RstarMax)]):
			# straight line, only need end point
			points = [[0.0, 0.0, 0.0]]  # initialize points to origin
			points.append([Vastar * math.cos(Gammastar) * tFinal, 0.0, -Vastar * math.sin(Gammastar) * tFinal])
		else:
			# curved path, need to iterate and generate points
			Rstar = 1.0 / Kappastar
			points = [[0.0, -Rstar, 0.0]]  # initialize points to Radius
			down = 0.0
			psi = math.copysign(math.pi / 2.0, -Kappastar)
			VaKCosGam = Vastar * Kappastar * math.cos(Gammastar)
			VaSinGam = Vastar * math.sin(Gammastar)
			Rstar = math.fabs(Rstar)

			for i in range(int(tFinal / tStep)):
				psi += VaKCosGam * tStep
				down -= VaSinGam * tStep
				points.append([Rstar * math.cos(psi), Rstar * math.sin(psi), down])

		p0 = points[0]

		points = MatrixMath.offset(points, self.VehicleTrimModel.getVehicleState().pn - p0[0],
										 self.VehicleTrimModel.getVehicleState().pe - p0[1],
										 self.VehicleTrimModel.getVehicleState().pd - p0[2])
		cos_psi = math.cos(self.VehicleTrimModel.getVehicleState().yaw)
		sin_psi = math.sin(self.VehicleTrimModel.getVehicleState().yaw)
		R_psi = [[cos_psi, sin_psi, 0],[-sin_psi, cos_psi, 0],[0, 0, 1]]
		points = MatrixMath.multiply(points,R_psi)
		points = Rotations.ned2enu(points)
		return points

	def PathFollowingError(self, dT=VPC.dT, state=States.vehicleState(), Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0):
		"""
		Function to compute the error between the current state and where it should be at that point in time. Assumes that
		the state entered as a parameter and the trim state in self start out at the same point. The error is returned as
		an error state as defined in VehicleState. WARNING: Currently Unused.

		:param dT: time step since last comparison [s]
		:param state: input state from actual aircraft simulation, from Containers.States.vehicleState
		:param Vastar: trip airspeed in [m/s]
		:param Kappastar: trim curvature in [1/m] (1/R*), +CW, -CCW
		:param Gammastar: trim flight path angle in [rad]
		:return: errorState: the differences between the actual state and the true state
		"""
		errorState = States.vehicleState()
		Rstar = 1.0 / Kappastar
		AlphaStar = math.atan2(self.VehicleTrimModel.getVehicleState().w, self.VehicleTrimModel.getVehicleState().u)
		straightFlag = any([math.isclose(Kappastar, 0.0), math.fabs(Kappastar) < (1.0 / VPC.RstarMax)])

		if straightFlag:
			Betastar = 0.0
			Phistar = 0.0
		else:
			Betastar = math.atan2(self.VehicleTrimModel.getVehicleState().v, self.VehicleTrimModel.getVehicleState().u)
			Phistar = math.atan2(Vastar ** 2 * math.cos(Gammastar) * Kappastar, VPC.g0)

		self.VehicleTrimModel.getVehicleState().alpha = AlphaStar
		self.VehicleTrimModel.getVehicleState().beta = Betastar
		self.VehicleTrimModel.getVehicleState().Va = Vastar
		self.VehicleTrimModel.getVehicleState().roll = Phistar

		ustar = Vastar * math.cos(AlphaStar) * math.cos(Betastar)
		vstar = Vastar * math.sin(Betastar)
		wstar = Vastar * math.sin(AlphaStar) * math.cos(Betastar)
		Thetastar = AlphaStar + Gammastar

		if straightFlag:
			pstar = 0.0
			qstar = 0.0
			rstar = 0.0
			self.VehicleTrimModel.getVehicleState().pn += Vastar * math.cos(Gammastar) * \
															  math.cos(self.VehicleTrimModel.getVehicleState().yaw) * dT
			self.VehicleTrimModel.getVehicleState().pe += Vastar * math.cos(Gammastar) * \
															  math.sin(self.VehicleTrimModel.getVehicleState().yaw) * dT
		else:
			pstar = -Vastar * Kappastar * math.sin(Thetastar)
			qstar = Vastar * Kappastar * math.sin(Phistar) * math.cos(Thetastar)
			rstar = Vastar * Kappastar * math.cos(Phistar) * math.cos(Thetastar)
			self.VehicleTrimModel.getVehicleState().yaw += Vastar * Kappastar * math.cos(Gammastar) * dT
			self.VehicleTrimModel.getVehicleState().pn = Rstar * math.cos(self.VehicleTrimModel.getVehicleState().yaw)
			self.VehicleTrimModel.getVehicleState().pe = Rstar * math.sin(self.VehicleTrimModel.getVehicleState().yaw)

		self.VehicleTrimModel.getVehicleState().pd -= Vastar * math.sin(Gammastar) * dT
		self.VehicleTrimModel.getVehicleState().u = ustar
		self.VehicleTrimModel.getVehicleState().v = vstar
		self.VehicleTrimModel.getVehicleState().w = wstar
		self.VehicleTrimModel.getVehicleState().pitch = Thetastar
		self.VehicleTrimModel.getVehicleState().p = pstar
		self.VehicleTrimModel.getVehicleState().q = qstar
		self.VehicleTrimModel.getVehicleState().r = rstar
		self.VehicleTrimModel.getVehicleState().R = Rotations.euler2DCM(self.VehicleTrimModel.getVehicleState().yaw,
																			self.VehicleTrimModel.getVehicleState().pitch,
																			self.VehicleTrimModel.getVehicleState().roll)

		errorState.pn = state.pn - self.VehicleTrimModel.getVehicleState().pn
		errorState.pe = state.pn - self.VehicleTrimModel.getVehicleState().pe
		errorState.pd = state.pn - self.VehicleTrimModel.getVehicleState().pd
		errorState.u = state.u - self.VehicleTrimModel.getVehicleState().u
		errorState.v = state.v - self.VehicleTrimModel.getVehicleState().v
		errorState.w = state.w - self.VehicleTrimModel.getVehicleState().w
		errorState.yaw = state.yaw - self.VehicleTrimModel.getVehicleState().yaw
		errorState.pitch = state.pitch - self.VehicleTrimModel.getVehicleState().pitch
		errorState.roll = state.roll - self.VehicleTrimModel.getVehicleState().roll
		errorState.p = state.p - self.VehicleTrimModel.getVehicleState().p
		errorState.q = state.q - self.VehicleTrimModel.getVehicleState().q
		errorState.r = state.r - self.VehicleTrimModel.getVehicleState().r
		errorState.alpha = state.alpha - self.VehicleTrimModel.getVehicleState().alpha
		errorState.beta = state.beta - self.VehicleTrimModel.getVehicleState().beta
		errorState.R = MatrixMath.subtract(state.R, self.VehicleTrimModel.getVehicleState().R)

		return errorState




