import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.Chapter7Simulate
import ece163.Display.DataExport
import ece163.Display.doubleInputWithLabel
import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants
import ece163.Display.WindControl as WindControl
from ece163.Display.vehicleTrimWidget import vehicleTrimWidget
from ece163.Display.controlGainsWidget import controlGainsWidget
from ece163.Display.ReferenceControlWidget import ReferenceControlWidget

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'alpha', 'beta']

positionRange = 200

defaultTrimParameters = [('Airspeed', VehiclePhysicalConstants.InitialSpeed), ('Climb Angle', 0), ('Turn Radius', math.inf)]

class Chapter6(baseInterface.baseInterface):

	def __init__(self, parent=None):
		self.simulateInstance = ece163.Simulation.Chapter7Simulate.Chapter7Simulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Chapter 7")
		stateplotElements = [[x] for x in stateNamesofInterest]
		stateplotElements.append(['Va', 'Vg'])
		statetitleNames = list(stateNamesofInterest)
		statetitleNames.append('Va & Vg')
		legends = [False] * len(stateNamesofInterest) + [True]
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(5, 3, stateplotElements, titles=statetitleNames, useLegends=legends)
		self.outPutTabs.addTab(self.stateGrid, "States")

		ControlPlotNames = ['Course', 'Speed', 'Height', 'Pitch', 'Roll']
		controlplotElements = [['Reference', 'Actual'] for x in ControlPlotNames]
		trimPlotNames = ['Throttle', 'Aileron', 'Elevator', 'Rudder']
		controlplotElements.extend([['Trim', 'Actual'] for x in trimPlotNames])
		ControlPlotNames.extend(trimPlotNames)

		self.controlResponseGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(3, 4, controlplotElements, titles=ControlPlotNames, useLegends=True)
		self.afterUpdateDefList.append(self.updateControlResponsePlots)

		SensorPlotNames = ["Gyro X (deg/s)", "Gyro Y (deg/s)", "Gyro Z (deg/s)", "Baro (N/m^2)"]
		SensorPlotNames.extend(["Accel X (m/s^2)", "Accel Y (m/s^2)", "Accel Z (m/s^2)", "Pitot (N/m^2)"])
		SensorPlotNames.extend(["Mag X (nT)", "Mag Y (nT)", "Mag Z (nT)", "GPS Cog (deg)"])
		SensorPlotNames.extend(["GPS N (m)", "GPS E (m)", "GPS Alt (m)", "GPS Sog (m/s)"])

		sensorPlotElements = [['True', 'Noisy'] for x in SensorPlotNames]

		self.sensorsPlotGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(4, 4, sensorPlotElements, titles=SensorPlotNames, useLegends=True)
		self.afterUpdateDefList.append(self.updateSensorPlots)

		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance, 'Chapter7')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")



		self.trimCalcWidget = vehicleTrimWidget(self, self.trimCalcComplete)

		# self.inputTabs.

		self.gainCalcWidget = controlGainsWidget(self, self.gainCalcComplete, parent=self)
		self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)

		# self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)


		self.referenceControl = ReferenceControlWidget()
		self.inputTabs.addTab(self.referenceControl, "Reference Control")

		self.windControl = WindControl.WindControl(self.simulateInstance.underlyingModel.getVehicleAerodynamicsModel())
		self.inputTabs.addTab(self.windControl, WindControl.widgetName)

		self.simulateInstance.underlyingModel.setControlGains(self.gainCalcWidget.curGains)
		self.simulateInstance.underlyingModel.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		# self.playButton.setDisabled(True)

		# we wil handle the tab ordering at the end

		stateTab = self.outPutTabs.widget(2)
		stateText = self.outPutTabs.tabText(2)

		self.outPutTabs.removeTab(2)

		exportTab = self.outPutTabs.widget(2)
		exportText = self.outPutTabs.tabText(2)

		self.outPutTabs.addTab(self.trimCalcWidget, "Trim")
		self.outPutTabs.addTab(self.gainCalcWidget, "Gains")
		self.outPutTabs.addTab(stateTab, stateText)
		self.outPutTabs.addTab(self.sensorsPlotGrid, "Sensors")
		self.outPutTabs.addTab(self.controlResponseGrid, "Control Response")
		self.outPutTabs.addTab(exportTab, exportText)

		self.outPutTabs.setCurrentIndex(5)

		self.showMaximized()

		return

	def updateStatePlots(self, newState):
		stateList = list()
		for key in stateNamesofInterest:
			newVal = getattr(newState, key)
			if key in ['yaw', 'pitch', 'roll', 'p', 'q', 'r', 'alpha', 'beta']:
				newVal = math.degrees(newVal)
			stateList.append([newVal])
		stateList.append([newState.Va, math.hypot(newState.u, newState.v, newState.w)])

		self.stateGrid.addNewAllData(stateList, [self.simulateInstance.time]*(len(stateNamesofInterest) + 1))
		return

	def getVehicleState(self):
		return self.simulateInstance.underlyingModel.getVehicleState()

	def runUpdate(self):
		# inputControls = Inputs.controlInputs()
		self.simulateInstance.takeStep(self.referenceControl.currentReference)

		return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()
		self.vehicleInstance.reset(self.simulateInstance.underlyingModel.getVehicleState())
		self.updateNumericStateBox(self.simulateInstance.underlyingModel.getVehicleState())
		self.vehicleInstance.removeAllAribtraryLines()
		self.controlResponseGrid.clearDataPointsAll()

	def trimCalcComplete(self, **kwargs):
		"""
		if we have valid trim conditions we calculate the linear model
		"""
		self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)
		self.simulateInstance.underlyingModel.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		return

	def gainCalcComplete(self):
		self.simulateInstance.underlyingModel.setControlGains(self.gainCalcWidget.curGains)
		self.simulateInstance.underlyingModel.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		print(self.simulateInstance.underlyingModel.getControlGains())

	def updateControlResponsePlots(self):
		# ControlPlotNames = ['Course', 'Speed', 'Height', 'Pitch', 'Roll']
		# controlplotElements = [['Reference', 'Actual'] for x in ControlPlotNames]
		# trimPlotNames = ['Throttle', 'Aileron', 'Elevator', 'Rudder']
		# controlplotElements.extend([['Trim', 'Actual'] for x in trimPlotNames])
		# ControlPlotNames.extend(trimPlotNames)
		# self.stateGrid.addNewAllData(stateList, [self.simulateInstance.time] * (len(stateNamesofInterest) + 1))
		inputToGrid = list()

		Commanded = self.referenceControl.currentReference
		vehicleState = self.simulateInstance.getVehicleState()
		inputToGrid.append([math.degrees(Commanded.commandedCourse), math.degrees(vehicleState.chi)])  # Course
		inputToGrid.append([Commanded.commandedAirspeed, vehicleState.Va])  # Speed
		inputToGrid.append([Commanded.commandedAltitude, -vehicleState.pd])  # Height
		inputToGrid.append([math.degrees(x) for x in [Commanded.commandedPitch, vehicleState.pitch]])  # pitch
		inputToGrid.append([math.degrees(x) for x in [Commanded.commandedRoll, vehicleState.roll]])  # pitch
		ActualControl = self.simulateInstance.underlyingModel.getVehicleControlSurfaces()
		trimSettings = self.trimCalcWidget.currentTrimControls
		inputToGrid.append([trimSettings.Throttle, ActualControl.Throttle])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Aileron, ActualControl.Aileron]])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Elevator, ActualControl.Elevator]])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Rudder, ActualControl.Rudder]])  # Throttle
		# print(inputToGrid)
		self.controlResponseGrid.addNewAllData(inputToGrid, [self.simulateInstance.time]*len(inputToGrid))


		return

	def updateSensorPlots(self):
		inputToGrid = list()
		noisySensors = self.simulateInstance.sensorModel.getSensorsNoisy()
		trueSensors = self.simulateInstance.sensorModel.getSensorsTrue()

		inputToGrid.append([math.degrees(trueSensors.gyro_x), math.degrees(noisySensors.gyro_x)])
		inputToGrid.append([math.degrees(trueSensors.gyro_y), math.degrees(noisySensors.gyro_y)])
		inputToGrid.append([math.degrees(trueSensors.gyro_z), math.degrees(noisySensors.gyro_z)])
		inputToGrid.append([trueSensors.baro, noisySensors.baro])

		inputToGrid.append([trueSensors.accel_x, noisySensors.accel_x])
		inputToGrid.append([trueSensors.accel_y, noisySensors.accel_y])
		inputToGrid.append([trueSensors.accel_z, noisySensors.accel_z])
		inputToGrid.append([trueSensors.pitot, noisySensors.pitot])

		inputToGrid.append([trueSensors.mag_x, noisySensors.mag_x])
		inputToGrid.append([trueSensors.mag_y, noisySensors.mag_y])
		inputToGrid.append([trueSensors.mag_z, noisySensors.mag_z])
		inputToGrid.append([math.degrees(trueSensors.gps_cog), math.degrees(noisySensors.gps_cog)])

		inputToGrid.append([trueSensors.gps_n, noisySensors.gps_n])
		inputToGrid.append([trueSensors.gps_e, noisySensors.gps_e])
		inputToGrid.append([trueSensors.gps_alt, noisySensors.gps_alt])
		inputToGrid.append([trueSensors.gps_sog, noisySensors.gps_sog])


		self.sensorsPlotGrid.addNewAllData(inputToGrid, [self.simulateInstance.time]*len(inputToGrid))
		return

sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		# f.write(repr(exctype))
		# f.write('\n')
		# f.write(repr(value))
		# f.write('\n')
		traceback.print_exception(exctype, value, tracevalue, file=f)
		# traceback.print_tb(tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook



qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = Chapter6()
ourWindow.show()
qtApp.exec()