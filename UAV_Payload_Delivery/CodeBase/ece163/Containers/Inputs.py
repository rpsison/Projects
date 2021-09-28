"""
File contains the classes that define the inputs to the various parts of the simulator. These classes are used
internally, and more will be added as needed. This file is differentiated from the States.py file which contains the
classes that are internal states tracked within the simulator.
"""

import math

testingAbs_tol = 1e-6

class forcesMoments:
	def __init__(self, Fx=0.0, Fy=0.0, Fz=0.0, Mx=0.0, My=0.0, Mz=0.0):
		"""
		Defines the forces [N] and moments [N-m] struct such that this can be passed around to the various functions that need to
		use them. Forces and moments are defined in the body-frame and assumed to be located at the center of mass.

		:param Fx: sum of forces in body-x direction [N]
		:param Fy: sum of forces in body-y direction [N]
		:param Fz: sum of forces in body-z direction [N]
		:param Mx: sum of moments about body-x direction [N-m]
		:param My: sum of moments about body-y direction [N-m]
		:param Mz: sum of moments about body-z direction [N-m]
		"""
		self.Fx = Fx
		self.Fy = Fy
		self.Fz = Fz
		self.Mx = Mx
		self.My = My
		self.Mz = Mz
		return

	def __repr__(self):
		return "{0.__name__}(Fx={1.Fx}, Fy={1.Fy}, Fz={1.Fz}, Mx={1.Mx}, My={1.My}, Mz={1.Mz})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in ['Fx', 'Fy', 'Fz',
																								'Mx', 'My', 'Mz']]):
				return False
			else:
				return True
		else:
			return NotImplemented


class controlInputs:
	def __init__(self, Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0):
		"""
		Defines the control inputs which are composed of throttle, ailerons, elevator, and rudder. Note that while the
		throttle is a PWM signal, the remaining ones are angles (in radians). These are all defined in body-frame

		:param Throttle: Throttle input [0 - 1], + Throttle -> + u (aircraft increases speed)
		:param Aileron: Aileron input [rad], + aileron -> + p (aircraft rolls right wind down)
		:param Elevator: Elevator input [rad], + elevator -> - q (aircraft pitches nose down)
		:param Rudder: Rudder input [rad], + rudder -> - r (aircraft yaws counter-clockwise from above)
		"""
		self.Throttle = Throttle
		self.Aileron = Aileron
		self.Elevator = Elevator
		self.Rudder = Rudder
		return

	def __repr__(self):
		return "{0.__name__}(Throttle={1.Throttle}, Aileron={1.Aileron}, Elevator={1.Elevator}, Rudder={1.Rudder})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member)) for member in ['Throttle', 'Aileron', 'Elevator', 'Rudder']]):
				return False
			else:
				return True
		else:
			return NotImplemented


class drydenParameters:
	def __init__(self, Lu=200.0, Lv=200.0, Lw=50.0, sigmau=1.06, sigmav=2.06, sigmaw=0.7):
		"""
		Defines the Dryden gust model parameters for use in wind modeling. Defaults are set to the Dryden low altitude
		light turbulence model.

		:param Lu: spatial frequency in forward
		:param Lv: spatial frequency sideways
		:param Lw: spatial frequency down
		:param sigmau: variance in forward
		:param sigmav: variance in sideways
		:param sigmaw: variance in down
		"""
		self.Lu = Lu
		self.Lv = Lv
		self.Lw = Lw
		self.sigmau = sigmau
		self.sigmav = sigmav
		self.sigmaw = sigmaw
		return

	def __repr__(self):
		return "{0.__name__}(Lu={1.Lu}, Lv={1.Lv}, Lw={1.Lw}, sigmau={1.sigmau}, sigmav={1.sigmav}, sigmaw={1.sigmaw})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member)) for member in ['Lu', 'Lv', 'Lw', 'sigmau', 'sigmav', 'sigmaw']]):
				return False
			else:
				return True
		else:
			return NotImplemented

