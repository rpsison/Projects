import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.Chapter5Simulate
import ece163.Display.DataExport
import ece163.Display.doubleInputWithLabel
import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants
import ece163.Display.WindControl as WindControl
from ece163.Display.vehicleTrimWidget import vehicleTrimWidget

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'alpha', 'beta']
systemInputs = [('Throttle', 0, 1, 0),
				('Aileron', -0.3, 0.3, 0),
				('Elevator', -0.3, 0.3, 0),
				('Rudder', -0.3, 0.3, 0)]

positionRange = 200

defaultTrimParameters = [('Airspeed', VehiclePhysicalConstants.InitialSpeed), ('Climb Angle', 0), ('Turn Radius', math.inf)]

class Chapter5(baseInterface.baseInterface):

	def __init__(self, parent=None):
		self.simulateInstance = ece163.Simulation.Chapter5Simulate.Chapter5Simulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Chapter 5")
		plotElements = [[x] for x in stateNamesofInterest]
		plotElements.append(['Va', 'Vg'])
		titleNames = list(stateNamesofInterest)
		titleNames.append('Va & Vg')
		legends = [False] * len(stateNamesofInterest) + [True]
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(5, 3, plotElements, titles=titleNames, useLegends=legends)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance, 'Chapter5')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")



		self.trimCalcWidget = vehicleTrimWidget(self, self.trimCalcComplete)
		self.inputTabs.addTab(self.trimCalcWidget, "Trim")
		# self.inputTabs.


		self.windControl = WindControl.WindControl(self.simulateInstance.underlyingModel)
		self.inputTabs.addTab(self.windControl, WindControl.widgetName)

		# self.playButton.setDisabled(True)
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
		self.simulateInstance.takeStep()

		return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()
		self.vehicleInstance.reset(self.simulateInstance.underlyingModel.getVehicleState())
		self.updateNumericStateBox(self.simulateInstance.underlyingModel.getVehicleState())
		self.vehicleInstance.removeAllAribtraryLines()

	def trimCalcComplete(self, Vastar, Kappastar, Gammastar,  **kwargs):
		# self.vehicleInstance.reset()
		self.ResetSimulation()
		newControlInput = self.trimCalcWidget.currentTrimControls
		self.simulateInstance.controlInput = newControlInput

		newStartState = self.trimCalcWidget.currentTrimState
		self.simulateInstance.underlyingModel.setVehicleState(newStartState)
		self.updateNumericStateBox(newStartState)
		self.vehicleInstance.reset(newStartState)
		self.vehicleInstance.removeAllAribtraryLines()
		self.vehicleInstance.addAribtraryLine(self.trimCalcWidget.trimInstance.GenerateIdealPath(Vastar, Kappastar, Gammastar), (0, 0, 1, 1))

		# self.vehicleInstance.openGLWindow.repaint()
		# self.vehicleInstance.clearAribtraryLine()
		# time.sleep(1)

		return


sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		traceback.print_exception(exctype, value, tracevalue, file=f)
		# traceback.print_tb(tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook



qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = Chapter5()
ourWindow.show()
qtApp.exec()