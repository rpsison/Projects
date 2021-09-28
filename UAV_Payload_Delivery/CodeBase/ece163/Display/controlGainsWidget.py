"""
widget which handles the creation of linear models and determining the gains
"""

import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import PyQt5.QtWidgets as QtWidgets

from ..Controls import VehiclePerturbationModels
from ..Containers import Controls
from ..Containers import Linearized
from ..Controls import VehicleControlGains
from ..Constants import VehiclePhysicalConstants
import sys
import os
import datetime

import math
import threading
import pickle
import matplotlib.pyplot as plt
import matplotlib
import time

testGainsFileName = 'LastGainsTest.png'
testNumSteps = 1000

lateralNames = ['Wn_roll', 'Zeta_roll', 'Wn_course', 'Zeta_course', 'Wn_sideslip', 'Zeta_sideslip']
longitudinalNames = ['Wn_pitch', 'Zeta_pitch', 'Wn_altitude','Zeta_altitude', 'Wn_SpeedfromThrottle',
                        'Zeta_SpeedfromThrottle', 'Wn_SpeedfromElevator', 'Zeta_SpeedfromElevator']

gainNames = ['kp_roll', 'kd_roll', 'ki_roll', 'kp_sideslip', 'ki_sideslip', 'kp_course', 'ki_course',
				'kp_pitch', 'kd_pitch', 'kp_altitude', 'ki_altitude', 'kp_SpeedfromThrottle', 'ki_SpeedfromThrottle',
                'kp_SpeedfromElevator', 'ki_SpeedfromElevator']
longitudinalGainNames = []

defaultTuningParameterFileName = "VehicleTuningParameters_Data.pickle"
defaultGainsFileName = "VehicleGains_Data.pickle"

gainTypes = ['Roll', 'SideSlip', 'Course', 'Pitch', 'Altitude', 'Speed']
rollNames = ['kp_roll', 'kd_roll', 'ki_roll']
sideslipNames = ['kp_sideslip', 'ki_sideslip']
courseNames = ['kp_course', 'ki_course']
pitchNames = ['kp_pitch', 'kd_pitch']
altitudeNames = ['kp_altitude', 'ki_altitude']
speedNames = ['kp_SpeedfromThrottle', 'ki_SpeedfromThrottle', 'kp_SpeedfromElevator', 'ki_SpeedfromElevator']


class displayGainsTest(QtWidgets.QDialog):
	def __init__(self, imagePath, parent=None):
		super().__init__(parent)
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.setWindowTitle("Test of Control Gains")
		self.gainResponse = QtWidgets.QLabel("sls")
		self.gainResponse.setPixmap(QtGui.QPixmap(imagePath))
		self.gainResponse.setScaledContents(True)
		self.gainResponse.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
		self.setMinimumSize(1280, 960)

		self.usedLayout.addWidget(self.gainResponse)

class controlGainsWidget(QtWidgets.QWidget):
	testFinishedSignal = QtCore.pyqtSignal(bool)
	def __init__(self, guiControls, callBackOnSuccesfulGains=None, parent=None):
		super().__init__(parent)
		self.parentInstance = parent
		# self.perturbationInstance = VehiclePerturbationModels.VehiclePerturbation()
		# self.gainsInstance = VehicleControlGains.VehicleControlGains()
		self.curGains = Controls.controlGains()
		self.curParameters = Controls.controlTuning()
		self.callBackOnSuccesfulGains = callBackOnSuccesfulGains
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.currentLinearModel = Linearized.transferFunctions

		self.testWindow = None

		topBoxEnclosure = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(topBoxEnclosure)

		tuningBox = QtWidgets.QVBoxLayout()
		tuningBox.addWidget(QtWidgets.QLabel("Tuning Parameters"))
		controlBox = QtWidgets.QVBoxLayout()
		# outputBox.addWidget(QLabel("Calculated Gains"))
		gainsBox = QtWidgets.QVBoxLayout()
		gainsBox.addWidget(QtWidgets.QLabel("Gains"))
		topBoxEnclosure.addLayout(tuningBox)
		topBoxEnclosure.addLayout(controlBox)
		topBoxEnclosure.addLayout(gainsBox)
		topBoxEnclosure.addStretch()

		# self.ParameterTabs = QTabWidget()
		# self.usedLayout.addWidget(self.ParameterTabs)

		# self.lateralGainsWidget = QWidget()
		# self.ParameterTabs.addTab(self.lateralGainsWidget, "Lateral Gains")

		# lateralGainsLayout = QGridLayout()
		# self.lateralGainsWidget.setLayout(lateralGainsLayout)
		self.parameterGainValues = dict()

		tuningFormLayout = QtWidgets.QFormLayout()
		tuningBox.addLayout(tuningFormLayout)
		tuningBox.addStretch()

		try:
			with open(os.path.join(sys.path[0], defaultTuningParameterFileName), 'rb') as f:
				savedParameters = pickle.load(f)
		except (FileNotFoundError, EOFError):
			savedParameters = Controls.controlTuning()
			for pName in lateralNames+longitudinalNames:
				setattr(savedParameters, pName, 1)

		for boxName, parNames in zip(['Lateral Autopilot', 'Longitudinal Autopilot'], [lateralNames, longitudinalNames]):
			sectionName = QtWidgets.QLabel(boxName)
			sectionName.setAlignment(QtCore.Qt.AlignLeft)
			tuningFormLayout.addRow(sectionName)
			for parameterName in parNames:
				# newInput = doubleInputWithLabel.doubleInputWithLabel(parameterName)
				newInput = QtWidgets.QLineEdit()
				newValidator = QtGui.QDoubleValidator()
				newInput.setValidator(newValidator)
				newInput.setText("{}".format(getattr(savedParameters, parameterName)))
				tuningFormLayout.addRow(parameterName, newInput)
				# inputBox.addWidget(newInput)
				self.parameterGainValues[parameterName] = newInput

		gainFormLayout = QtWidgets.QFormLayout()
		gainsBox.addLayout(gainFormLayout)
		self.gainValuesDict = dict()

		for boxName, gainNames in zip(gainTypes, [rollNames, sideslipNames, courseNames, pitchNames, altitudeNames, speedNames]):
			# print(boxName, gainNames)
			sectionName = QtWidgets.QLabel(boxName)
			sectionName.setAlignment(QtCore.Qt.AlignLeft)
			gainFormLayout.addRow(sectionName)
			for gainName in gainNames:
				newInput = QtWidgets.QLineEdit()
				newValidator = QtGui.QDoubleValidator()
				newInput.setText(str(0.0))
				newInput.setValidator(newValidator)
				gainFormLayout.addRow(gainName, newInput)
				self.gainValuesDict[gainName] = newInput


		gainsBox.addStretch()

		self.calcGainsButton = QtWidgets.QPushButton("Calculate Gains ->")
		self.calcGainsButton.clicked.connect(self.calculateGainsResponse)
		controlBox.addWidget(self.calcGainsButton)

		self.calcParametersButton = QtWidgets.QPushButton("<- Calculate Parameters")
		self.calcParametersButton.clicked.connect(self.calculateParametersResponse)
		controlBox.addWidget(self.calcParametersButton)

		applyGainsButton = QtWidgets.QPushButton("Apply Gains")
		applyGainsButton.clicked.connect(self.applyGains)
		controlBox.addWidget(applyGainsButton)

		self.testGainsButton = QtWidgets.QPushButton("&Test Gains")
		self.testGainsButton.clicked.connect(self.startTestGains)
		controlBox.addWidget(self.testGainsButton)

		self.saveGainsButton = QtWidgets.QPushButton("Save Parameters and Gains")
		self.saveGainsButton.clicked.connect(self.saveParametersGainsResponse)
		controlBox.addWidget(self.saveGainsButton)
		controlBox.addStretch()
		self.statusText = QtWidgets.QLabel("No Info")
		self.usedLayout.addWidget(self.statusText)
		# compression.addStretch()


		self.testFinishedSignal.connect(self.gainTestOverSignal)
		# self.gainsTextBox = QPlainTextEdit()
		# self.gainsTextBox.setReadOnly(True)
		# outputBox.addWidget(self.gainsTextBox)
		# outputBox.addStretch()

		try:
			with open(os.path.join(sys.path[0], defaultGainsFileName), 'rb') as f:
				self.curGains = pickle.load(f)
			self.updateGainsDisplay(self.curGains)
		except (FileNotFoundError, EOFError):
			pass
		self.usedLayout.addStretch()

	def createLinearizedModels(self, trimState=None, trimInput=None):
		if trimState is None:
			trimState = self.perturbationInstance.trimState
		if trimInput is None:
			trimInput = self.perturbationInstance.trimInputs
		try:
			self.currentLinearModel = VehiclePerturbationModels.CreateTransferFunction(trimState, trimInput)
		except Exception:
			import traceback
			self.parentInstance.raiseExceptionToUser(traceback.format_exc())
			return
		self.statusText.setText("Linearized Models Made at {}".format(datetime.datetime.now()))
		return

	def calculateGainsResponse(self):
		"""
		calculate the gains given the linear model here

		:return:
		"""
		newParameters = Controls.controlTuning()
		for parameter in lateralNames+longitudinalNames:
			if hasattr(newParameters, parameter):
				setattr(newParameters, parameter, float(self.parameterGainValues[parameter].text()))
			else:
				print(parameter)
		# with open(os.path.join(sys.path[0], defaultTuningParameterFileName), 'wb') as f:
		# 	pickle.dump(newParameters, f)
		# self.gainsInstance.setLinearizedModel(self.perturbationInstance.transferFunction)
		# self.gainsInstance.computeGains(newParameters)
		try:
			self.curGains = VehicleControlGains.computeGains(newParameters, self.currentLinearModel)
		except Exception:
			import traceback
			self.parentInstance.raiseExceptionToUser(traceback.format_exc())
			return
		self.updateGainsDisplay(self.curGains)
		self.statusText.setText(("Gains Calculated"))
		if self.callBackOnSuccesfulGains is not None:
			self.callBackOnSuccesfulGains()
		return

	def calculateParametersResponse(self):
		"""
		calculate the gains given the linear model here

		:return:
		"""
		newGains = Controls.controlGains()
		for gain in newGains.__dict__.keys():
			setattr(newGains, gain, float(self.gainValuesDict[gain].text()))

		# with open(os.path.join(sys.path[0], defaultGainsFileName), 'wb') as f:
		# 	pickle.dump(newGains, f)
		# self.gainsInstance.setLinearizedModel(self.perturbationInstance.transferFunction)
		# self.gainsInstance.computeGains(newParameters)
		try:
			self.curParameters = VehicleControlGains.computeTuningParameters(newGains, self.currentLinearModel)
		except Exception:
			import traceback
			self.parentInstance.raiseExceptionToUser(traceback.format_exc())
			return
		self.updateParametersDisplay(self.curParameters)
		self.statusText.setText(("Parameters Calculated"))
		return

	def updateGainsDisplay(self, newGains):
		for name in gainNames:
			self.gainValuesDict[name].setText(str(getattr(newGains, name)))

	def updateParametersDisplay(self, newParameters):
		for name in newParameters.__dict__.keys():
			self.parameterGainValues[name].setText(str(getattr(newParameters, name)))


	def applyGains(self):
		self.curGains = self.buildCurrentGains()
		if self.callBackOnSuccesfulGains is not None:
			self.callBackOnSuccesfulGains()
		return

	def saveParametersGainsResponse(self):
		"""
		we save the gains here
		"""
		with open(os.path.join(sys.path[0], defaultGainsFileName), 'wb') as f:
			pickle.dump(self.buildCurrentGains(), f)
		with open(os.path.join(sys.path[0], defaultTuningParameterFileName), 'wb') as f:
			pickle.dump(self.buildCurrentParameters(), f)
		self.statusText.setText("Parameters and Gains Saved")
		return

	def buildCurrentGains(self):
		newGains = Controls.controlGains()
		for name in gainNames:
			# self.gainValuesDict[name].setText(str(getattr(newGains, name)))
			try:
				setattr(newGains, name, float(self.gainValuesDict[name].text()))
			except ValueError:
				pass
		return newGains

	def buildCurrentParameters(self):
		newGains = Controls.controlTuning()
		for name in lateralNames+longitudinalNames:
			# self.gainValuesDict[name].setText(str(getattr(newGains, name)))
			try:
				setattr(newGains, name, float(self.parameterGainValues[name].text()))
			except ValueError:
				pass
		return newGains

	def startTestGains(self):
		self.statusText.setText("Starting Gains Test")
		# self.testWindow = displayGainsTest('LastGainsTest.png')
		# self.testWindow.open()
		self.curGains = self.buildCurrentGains()
		threading.Thread(target=self.runGainsTest, daemon=True).start()
		# self.runGainsTest()
		self.testGainsButton.setDisabled(True)
		return

	def gainTestOverSignal(self, itFinished):
		self.testGainsButton.setDisabled(False)
		self.testWindow = displayGainsTest(testGainsFileName)
		self.statusText.setText('Test Run was Completed')
		self.testWindow.open()
		return

	def runGainsTest(self):
		simInstance = self.parentInstance.simulateInstance
		simInstance.reset()  # reset the model
		refControls = self.parentInstance.referenceControl.currentReference # grab the controls for the run

		simInstance.underlyingModel.setControlGains(self.curGains)  # and make sure the gains are set
		trimSettings = self.parentInstance.trimCalcWidget.currentTrimControls

		wantedValues = dict()

		wantedValues['chi'] = list()
		wantedValues['Va'] = list()
		wantedValues['pd'] = list()
		wantedValues['cPitch'] = list()
		wantedValues['aPitch'] = list()
		wantedValues['cRoll'] = list()
		wantedValues['aRoll'] = list()
		wantedValues['Throttle'] = list()
		wantedValues['Aileron'] = list()
		wantedValues['Elevator'] = list()
		wantedValues['Rudder'] = list()

		for i in range(testNumSteps+1):
			simInstance.takeStep(refControls)
			curState = simInstance.getVehicleState()

			wantedValues['chi'].append(math.degrees(curState.chi))
			wantedValues['Va'].append(curState.Va)
			wantedValues['pd'].append(-curState.pd)

			wantedValues['cPitch'].append(math.degrees(refControls.commandedPitch))
			wantedValues['aPitch'].append(math.degrees(curState.pitch))

			wantedValues['cRoll'].append(math.degrees(refControls.commandedRoll))
			wantedValues['aRoll'].append(math.degrees(curState.roll))

			ActualControl = simInstance.underlyingModel.getVehicleControlSurfaces()

			wantedValues['Throttle'].append(ActualControl.Throttle)
			wantedValues['Aileron'].append(math.degrees(ActualControl.Aileron))
			wantedValues['Elevator'].append(math.degrees(ActualControl.Elevator))
			wantedValues['Rudder'].append(math.degrees(ActualControl.Rudder))

		# print(wantedValues)
		matplotlib.use('agg')
		plotData = list()
		staticLength = len(wantedValues['chi'])
		plotData.append([[math.degrees(refControls.commandedCourse)]*staticLength, wantedValues['chi'], 'Course'])
		plotData.append([[refControls.commandedAirspeed]*staticLength, wantedValues['Va'], 'Speed'])
		plotData.append([[refControls.commandedAltitude]*staticLength, wantedValues['pd'], 'Height'])
		plotData.append([wantedValues['cPitch'], wantedValues['aPitch'], 'Pitch'])
		plotData.append([wantedValues['cRoll'], wantedValues['aRoll'], 'Roll'])
		plotData.append([[trimSettings.Throttle]*staticLength, wantedValues['Throttle'], 'Throttle'])
		plotData.append([[math.degrees(trimSettings.Aileron)]*staticLength, wantedValues['Aileron'], 'Aileron'])
		plotData.append([[math.degrees(trimSettings.Elevator)]*staticLength, wantedValues['Elevator'], 'Elevator'])
		plotData.append([[math.degrees(trimSettings.Rudder)]*staticLength, wantedValues['Rudder'], 'Rudder'])

		timeSteps = [VehiclePhysicalConstants.dT*x for x in range(staticLength)]

		for index, (refValues, actualValues, plotName) in enumerate(plotData):
			plt.subplot(3, 4, index+1)
			ref = plt.plot(timeSteps, refValues, label='Reference') # we want the colors to match for the legend
			act = plt.plot(timeSteps, actualValues,  label='Actual')
			plt.title(plotName)

		plt.gcf().legend(handles=(ref[0], act[0]), labels=('Reference', 'Actual'), loc='lower center')
		# the oddity with the list is that even with single values it returns a list
		# and if you feed it a named argument, it can't parse it correctly
		# unnamed argument works but then matplotlib throws a warning
		# have to use named as many of them we do not wish to use.

		plt.tight_layout()
		plt.savefig(testGainsFileName, dpi=300)

		plt.close('all')
		time.sleep(.25)
		self.testFinishedSignal.emit(True)
		return