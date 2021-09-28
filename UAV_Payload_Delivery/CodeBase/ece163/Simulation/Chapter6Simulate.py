from . import Simulate
from ..Controls import VehicleClosedLoopControl
from ..Controls import PayloadDeliveryControl as PDC
from ..Containers.Controls import referenceCommands
from ..Containers.Controls import controlGains
from ..Containers.States import vehicleState
from ece163.Controls.VehicleTrim import VehicleTrim
from ..Constants import VehiclePhysicalConstants
import math

targetVa = 25
targetAlt = 100


class Chapter6Simulate(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['commandedCourse', 'commandedAltitude', 'commandedAirspeed'])
		self.underlyingModel = PDC.PayloadDeliveryControl(vehicleState(pn=1200,pe=1000))
		conGains = controlGains(kp_roll =3.0,
                                     kd_roll = 0.04,
                                     ki_roll = 0.001,
                                     kp_sideslip = 2.0,
                                     ki_sideslip = 2.0,
                                     kp_course = 5.0,
                                     ki_course = 2.0,
                                     kp_pitch = -10.0,
                                     kd_pitch = -0.8,
                                     kp_altitude = 0.08,
                                     ki_altitude = 0.03,
                                     kp_SpeedfromThrottle = 2.0,
                                     ki_SpeedfromThrottle = 1.0,
                                     kp_SpeedfromElevator = -0.5,
                                     ki_SpeedfromElevator = -0.1)


		initState = vehicleState(pn=0.0,
											pe=0.0,
											pd=-100.0,
											u=25.0,
											v=0.0,
											w=0.0,
											yaw=math.pi/4,
											pitch=0.0,
											roll=0.0,
											p=0.0,
											q=0.0,
											r=0.0)
		self.underlyingModel.target.setDeliveryAirSpeed(targetVa)
		self.underlyingModel.target.setDeliveryAltitude(targetAlt)
		self.underlyingModel.VCLC.setVehicleState(initState)

		self.underlyingModel.VCLC.setControlGains(conGains)
		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta', 'chi']))
		# self.dT = 1/50
		self.underlyingModel.Update(1)
		self.referenceInput = referenceCommands()

	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, referenceInput=None):
		self.time += VehiclePhysicalConstants.dT
		referenceInput = self.underlyingModel.referenceCommands
		self.underlyingModel.Update(0)
		self.recordData([referenceInput.commandedCourse, referenceInput.commandedAltitude, referenceInput.commandedAirspeed])
		return

	def reset(self):
		self.time = 0
		self.underlyingModel.reset()
		self.takenData.clear()