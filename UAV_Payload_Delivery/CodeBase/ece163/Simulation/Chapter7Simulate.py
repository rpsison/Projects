from . import Simulate
from ..Controls import VehicleClosedLoopControl
from ..Containers.Controls import referenceCommands
from ece163.Controls.VehicleTrim import VehicleTrim
from ..Constants import VehiclePhysicalConstants
from ..Sensors import SensorsModel

class Chapter7Simulate(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['commandedCourse', 'commandedAltitude', 'commandedAirspeed'])
		self.underlyingModel = VehicleClosedLoopControl.VehicleClosedLoopControl()
		self.sensorModel = SensorsModel.SensorsModel(self.underlyingModel.getVehicleAerodynamicsModel())

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta', 'chi']))
		# self.dT = 1/50

		self.referenceInput = referenceCommands()

	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, referenceInput=None):
		self.time += VehiclePhysicalConstants.dT
		if referenceInput is None:
			referenceInput = self.referenceInput
		self.underlyingModel.Update(referenceInput)
		self.sensorModel.update()
		self.recordData([referenceInput.commandedCourse, referenceInput.commandedAltitude, referenceInput.commandedAirspeed])
		return

	def reset(self):
		self.time = 0
		self.underlyingModel.reset()
		self.sensorModel.reset()
		self.takenData.clear()