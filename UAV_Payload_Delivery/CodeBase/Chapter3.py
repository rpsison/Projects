import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Containers.Inputs as Inputs
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.Chapter3Simulate
import ece163.Display.DataExport

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r']
systemInputs = [('Fx', -10, 10, 0),
				('Fy', -10, 10, 0),
				('Fz', -10, 10, 0),
				('Mx', -0.1, 0.1, 0),
				('My', -0.1, 0.1, 0),
				('Mz', -0.1, 0.1, 0)]

positionRange = 200

class Chapter3(baseInterface.baseInterface):
	def __init__(self, parent=None):
		# self.vehicleState = vehicleState.vehicleState()
		self.simulateInstance = ece163.Simulation.Chapter3Simulate.Chapter3Simulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Chapter 3")
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(4, 3, [[x] for x in stateNamesofInterest], titles=stateNamesofInterest)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance,'Chapter3')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")

		self.inputGrid = QtWidgets.QGridLayout()
		self.inputLayout.addLayout(self.inputGrid)

		resetSlidersButton = QtWidgets.QPushButton("Reset Sliders")
		self.inputLayout.addWidget(resetSlidersButton)
		resetSlidersButton.clicked.connect(self.resetSliders)

		self.inputLayout.addStretch()
		self.inputSliders = list()

		for row in range(2):
			for col in range(3):
				index = col+row*3
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
			if key in ['yaw', 'pitch', 'roll', 'p', 'q', 'r']:
				newVal = math.degrees(newVal)
			stateList.append([newVal])

		self.stateGrid.addNewAllData(stateList, [self.simulateInstance.time]*len(stateNamesofInterest))
		return

	def getVehicleState(self):
		return self.simulateInstance.underlyingModel.getVehicleState()

	def runUpdate(self):
		forceInputs = Inputs.forcesMoments()
		for control in self.inputSliders:
			setattr(forceInputs, control.name, control.curValue)
		self.simulateInstance.takeStep(forceInputs)

		return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()

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
ourWindow = Chapter3()
ourWindow.show()
qtApp.exec()