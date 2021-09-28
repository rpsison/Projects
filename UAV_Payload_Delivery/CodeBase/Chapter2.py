import math
import sys

# import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Containers.States as vehicleState
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll']

positionRange = 10

class testInterface(baseInterface.baseInterface):
	def __init__(self, parent=None):
		self.vehicleState = vehicleState.vehicleState()
		self.t = 0
		super().__init__(parent)
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(2, 3, [[x] for x in stateNamesofInterest], titles=stateNamesofInterest)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.inputGrid = QtWidgets.QGridLayout()
		self.inputLayout.addLayout(self.inputGrid)
		self.inputLayout.addStretch()
		self.inputSliders = list()

		for index, value in enumerate(stateNamesofInterest):
			if index > 2:
				row = 1
				minValue = -180
				maxValue = 180
			else:
				row = 0
				minValue = -positionRange
				maxValue = positionRange
			col = index % 3
			# print(row, col)
			newSlider = ece163.Display.SliderWithValue.SliderWithValue(value, minValue, maxValue, onChangePointer=self.sliderChangeResponse)
			self.inputSliders.append(newSlider)
			self.inputGrid.addWidget(newSlider, row, col)

		self.playButton.setDisabled(True)
		self.showMaximized()
		# self.vehicleInstance.leavePlaneTrail = False

		return

	def updateStatePlots(self, newState):
		stateList = list()
		for key in self.numericStatesDict.keys():
			stateList.append([getattr(newState, key)])
		self.stateGrid.addNewAllData(stateList)
		return

	def getVehicleState(self):
		return self.vehicleState

	def runUpdate(self):
		return

	def sliderChangeResponse(self, newValue, name):
		if name in ['yaw', 'pitch', 'roll']:
			setattr(self.vehicleState, name, math.radians(newValue))
		else:
			if name == 'z':
				newValue = -1*newValue
			setattr(self.vehicleState, name, newValue)
		self.runSimulation()
		return

	def resetSimulationActions(self):
		self.vehicleState = vehicleState.vehicleState()
		self.stateGrid.clearDataPointsAll()
		for slider in self.inputSliders:
			slider.resetSlider()

sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		traceback.print_exception(exctype, value, tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook



qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = testInterface()
ourWindow.show()
qtApp.exec()