"""
small widget to control wind settings presets taken from VehiclePhysicalConstants
"""

import PyQt5.QtWidgets as QtWidgets
import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants

widgetName = "Wind Control"

class WindControl(QtWidgets.QWidget):
	def __init__(self, aeroInstance, parent=None):
		"""
		generates a wind control widget using the lists in phsyical parameters. Needs a simulate instance so it can set them.

		:param simulateInstance: simulate instance used for setting the wind values
		"""

		super().__init__(parent)

		self.aeroInstance = aeroInstance

		usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(usedLayout)

		steadyWindwsColumn = QtWidgets.QVBoxLayout()
		gustWindwsColumn = QtWidgets.QVBoxLayout()

		windSelectionBox = QtWidgets.QHBoxLayout()
		usedLayout.addLayout(windSelectionBox)
		windSelectionBox.addLayout(steadyWindwsColumn)
		windSelectionBox.addLayout(gustWindwsColumn)

		steadyWindwsColumn.addWidget(QtWidgets.QLabel("Steady Windws"))
		gustWindwsColumn.addWidget(QtWidgets.QLabel("Gust Winds"))

		self.steadyWinds = list()
		self.steadyButtonsGroup = QtWidgets.QButtonGroup()
		for index, (name, value) in enumerate(VehiclePhysicalConstants.SteadyWinds):
			newRadio = QtWidgets.QRadioButton(name)
			self.steadyWinds.append(value)
			steadyWindwsColumn.addWidget(newRadio)
			self.steadyButtonsGroup.addButton(newRadio, index)

		self.steadyButtonsGroup.button(0).setChecked(True)
		# print(self.steadyButtonsGroup.checkedId())

		self.gustWinds = list()
		self.gustButtonsGroup = QtWidgets.QButtonGroup()
		for index, (name, value) in enumerate(VehiclePhysicalConstants.GustWinds):
			newRadio = QtWidgets.QRadioButton(name)
			self.gustWinds.append(value)
			gustWindwsColumn.addWidget(newRadio)
			self.gustButtonsGroup.addButton(newRadio, index)

		self.gustButtonsGroup.button(0).setChecked(True)

		applyWindsButton = QtWidgets.QPushButton("Apply Winds")
		applyWindsButton.clicked.connect(self.applyWindsResponse)
		usedLayout.addWidget(applyWindsButton)

		steadyWindwsColumn.addStretch()
		gustWindwsColumn.addStretch()
		return

	def applyWindsResponse(self):
		"""
		Reads in current wind radio buttons and adjusts the underlying model to the new parameters

		"""
		steadyWindsWanted = self.steadyWinds[self.steadyButtonsGroup.checkedId()]
		gustWindsWanted = self.gustWinds[self.gustButtonsGroup.checkedId()]
		# print(steadyWindsWanted, gustWindsWanted)
		self.aeroInstance.getWindModel().setWindModelParameters(*steadyWindsWanted, gustWindsWanted)
		return
