"""
small widget to control calculating Trim. It holds an instance of the vehicleTrim module and provides convenient access to it.
"""

import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets

import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants
from . import doubleInputWithLabel
from ..Controls import VehicleTrim
import sys
import os
import pickle

import math
import threading

defaultTrimParameters = [('Airspeed', VehiclePhysicalConstants.InitialSpeed), ('Climb Angle', 0), ('Turn Radius', math.inf)]
defaultTrimFileName = 'VehicleTrim_Data.pickle'


class vehicleTrimWidget(QtWidgets.QWidget):
	trimSignal = QtCore.pyqtSignal(tuple)
	def __init__(self, guiControls, callBackOnSuccesfulTrim=None, parent=None):
		"""
		widget to calculate trim within a gui

		:param guiControls: this must be the main gui to allow access to stuff like reset
		:param callBackOnSuccesfulTrim: called if trim was succesful with arguments of the trim parameters
		"""
		super().__init__(parent)
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.guiControls = guiControls
		self.callBack = callBackOnSuccesfulTrim

		self.trimInstance = VehicleTrim.VehicleTrim()

		try:
			with open(os.path.join(sys.path[0], defaultTrimFileName), 'rb') as f:
				self.currentTrimState, self.currentTrimControls = pickle.load(f)
		except FileNotFoundError:
			self.currentTrimState = self.trimInstance.getTrimState()
			self.currentTrimControls = self.trimInstance.getTrimControls()
		valueInputsBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(valueInputsBox)

		self.trimInputsDict = dict()
		for name, initValue in defaultTrimParameters:
			newControl = doubleInputWithLabel.doubleInputWithLabel(name, initValue)
			valueInputsBox.addWidget(newControl)
			self.trimInputsDict[name] = newControl

		trimControlsBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(trimControlsBox)
		self.setStraightandFlatButton = QtWidgets.QPushButton('Set Straight and Level')
		trimControlsBox.addWidget(self.setStraightandFlatButton)
		self.setStraightandFlatButton.clicked.connect(self.straightAndLevelResponse)

		self.calcTrimButton = QtWidgets.QPushButton("Calculate Trim")
		self.calcTrimButton.clicked.connect(self.trimButtonResponse)
		trimControlsBox.addWidget(self.calcTrimButton)
		self.saveTrimButton = QtWidgets.QPushButton("Save Trim")
		self.saveTrimButton.clicked.connect(self.saveTrimResponse)
		trimControlsBox.addWidget(self.saveTrimButton)

		self.trimStatus = QtWidgets.QLabel("No Trim Calculated")
		trimControlsBox.addWidget(self.trimStatus)
		trimControlsBox.addStretch()

		self.trimSignal.connect(self.trimCalculated)

		self.curInputGrid = QtWidgets.QGridLayout()
		self.usedLayout.addLayout(self.curInputGrid)

		self.curInputGridDict = dict()
		for i, name in enumerate([ 'Throttle', 'Aileron', 'Elevator', 'Rudder']):
			newLabel = QtWidgets.QLabel("{}: ".format(name))
			self.curInputGrid.addWidget(newLabel, 0, i)
			self.curInputGridDict[name] = newLabel
		self.updateCurInputGrid(self.currentTrimControls)
		self.usedLayout.addStretch()
		return

	def straightAndLevelResponse(self):
		"""
		sets the inputs back to straight and level
		"""
		for name, value in defaultTrimParameters:
			self.trimInputsDict[name].setValue(value)
		return

	def updateCurInputGrid(self, newInputs):
		"""
		simply updates the current inputs to the new ones

		:param newInputs: an instance of vehicleState
		"""
		for key, label in self.curInputGridDict.items():
			newVal = float(getattr(newInputs, key))
			label.setText("{}: {:03.4}".format(key, newVal))
		return

	def trimButtonResponse(self):
		"""
		disables the gui and starts the thread which actually calculates trim to ensure the gui doesn't stall
		"""
		self.trimStatus.setText("Calculating Trim")
		# self.guiControls.ResetSimulation()
		# self.resetSimulationActions()
		self.setDisabled(True)
		Va = self.trimInputsDict['Airspeed'].getValue()
		Kappa = 1/self.trimInputsDict['Turn Radius'].getValue()
		Gamma = math.radians(self.trimInputsDict['Climb Angle'].getValue())
		threading.Thread(target=self.calculateTrim, name='Calc Trim', args=(Va, Kappa, Gamma), daemon=True).start()
		# self.trimCalcComplete()
		return

	def calculateTrim(self, Vastar, Kappastar, Gammastar):
		"""
		calculates trim and emits a signal to the gui

		:param Vastar: passed to computeTrim
		:param Kappastar: passed to computeTrim
		:param Gammastar: passed to computeTrim
		:return:
		"""
		trimSucceeded = self.trimInstance.computeTrim(Vastar, Kappastar, Gammastar)
		self.trimSignal.emit((trimSucceeded, Vastar, Kappastar, Gammastar))
		return

	def trimCalculated(self, parameters):
		"""
		enables the gui and prints a status message.
		if trim was possible updates the inputs and uses the callback to pass the information on

		:param parameters: all the returns from the thread
		"""
		self.setDisabled(False)
		self.trimStatus.setText("Trim calculations Complete")
		trimSucceeded, Vastar, Kappastar, Gammastar = parameters
		if not trimSucceeded:
			self.trimStatus.setText("Trim parameters given are not possible")
			return

		self.currentTrimControls = self.trimInstance.getTrimControls()
		self.currentTrimState = self.trimInstance.getTrimState()
		self.updateCurInputGrid(self.trimInstance.getTrimControls())
		parametersDict = dict()
		parametersDict['Vastar'] = Vastar
		parametersDict['Kappastar'] = Kappastar
		parametersDict['Gammastar'] = Gammastar
		self.callBack(**parametersDict)
		return

	def saveTrimResponse(self):
		"""
		calls the trim with the path so we export the trimstate and inputs
		"""
		trimExportPath = os.path.join(sys.path[0], defaultTrimFileName)
		with open(trimExportPath, 'wb') as f:
			pickle.dump((self.currentTrimState, self.currentTrimControls), f)
		return