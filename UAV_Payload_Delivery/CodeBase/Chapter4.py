import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Containers.Inputs as Inputs
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.Chapter4Simulate
import ece163.Display.DataExport
import ece163.Display.WindControl as WindControl

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'alpha', 'beta']
systemInputs = [('Throttle', 0, 1, 0),
				('Aileron', -0.3, 0.3, 0),
				('Elevator', -0.3, 0.3, 0),
				('Rudder', -0.3, 0.3, 0)]

positionRange = 200

class Chapter4(baseInterface.baseInterface):
	def __init__(self, parent=None):
		self.simulateInstance = ece163.Simulation.Chapter4Simulate.Chapter4Simulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Chapter 4")
		plotElements = [[x] for x in stateNamesofInterest]
		plotElements.append(['Va', 'Vg'])
		titleNames = list(stateNamesofInterest)
		titleNames.append('Va & Vg')
		legends = [False] * len(stateNamesofInterest) + [True]
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(5, 3, plotElements, titles=titleNames, useLegends=legends)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance,'Chapter4')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")

		self.inputControlsWidget = QtWidgets.QWidget()
		gridSquish = QtWidgets.QVBoxLayout()
		self.inputGrid = QtWidgets.QGridLayout()
		gridSquish.addLayout(self.inputGrid)
		gridSquish.addStretch()
		self.inputControlsWidget.setLayout(gridSquish)
		self.inputTabs.addTab(self.inputControlsWidget, "Control Inputs")
		# self.inputLayout.addLayout(self.inputGrid)
		self.windControl = WindControl.WindControl(self.simulateInstance.underlyingModel)
		self.inputTabs.addTab(self.windControl, WindControl.widgetName)

		resetSlidersButton = QtWidgets.QPushButton("Reset Sliders")
		gridSquish.addWidget(resetSlidersButton)
		resetSlidersButton.clicked.connect(self.resetSliders)

		self.inputLayout.addStretch()
		self.inputSliders = list()

		for row in range(2):
			for col in range(2):
				index = col+row*2
				name, minValue, maxValue, startValue = systemInputs[index]
				newSlider = ece163.Display.SliderWithValue.SliderWithValue(name, minValue, maxValue, startValue)
				self.inputSliders.append(newSlider)
				self.inputGrid.addWidget(newSlider, row, col)

		# self.playButton.setDisabled(True)
		self.showMaximized()

		return

	def resetSliders(self):
		for slider in self.inputSliders:
			slider.resetSlider()
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
		inputControls = Inputs.controlInputs()
		for control in self.inputSliders:
			setattr(inputControls, control.name, control.curValue)
		self.simulateInstance.takeStep(inputControls)

		return

	# def sliderChangeResponse(self, newValue, name):
	# 	if name in ['yaw', 'pitch', 'roll']:
	# 		setattr(self.vehicleState, name, math.radians(newValue))
	# 	else:
	# 		if name == 'z':
	# 			newValue = -1*newValue
	# 		setattr(self.vehicleState, name, newValue)
	# 	self.runSimulation()
	# 	return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()
		self.vehicleInstance.reset(self.simulateInstance.underlyingModel.getVehicleState())

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
ourWindow = Chapter4()
ourWindow.show()
qtApp.exec()