"""
Simple convenience widget allowing for a double input box with ranges
"""

import PyQt5.QtGui as QtGui
import PyQt5.QtWidgets as QtWidgets

import math

class doubleInputWithLabel(QtWidgets.QWidget):
	def __init__(self, name, startValue= 0.0, minValue=-math.inf, maxValue=math.inf, onChangePointer=None, parent=None):
		"""
		this widget holds a single double value and label with a validator locked to the range desired.
		has an optional parameter to trigger a callback when the value is changed (unimplemented until needed)

		:param name: text name to display
		:param minValue: defaults to negative infinity
		:param maxValue: defaults to positive infinity
		:param startValue: starter value wanted
		:param onChangePointer: callback when value has changed
		"""

		super().__init__(parent)
		self.name = name
		self.funcPointer = onChangePointer
		self.curValue = startValue

		self.usedLayout = QtWidgets.QHBoxLayout()
		self.setLayout(self.usedLayout)
		self.usedLayout.addWidget(QtWidgets.QLabel("{}".format(name)))

		self.textEdit = QtWidgets.QLineEdit()
		self.doubleValidate = QtGui.QDoubleValidator()
		self.doubleValidate.setBottom(minValue)
		self.doubleValidate.setTop(maxValue)
		self.textEdit.setValidator(self.doubleValidate)

		self.usedLayout.addWidget(self.textEdit)
		self.usedLayout.addStretch()

		self.setValue(startValue)
		# print(self.getValue())

		return

	def setValue(self, newValue):
		"""
		sets a double value to the contrl

		:param newValue: the double value to set
		"""
		self.textEdit.setText(str(newValue))
		return

	def getValue(self):
		"""
		get the current value.

		:return: float of the current value
		"""
		return float(self.textEdit.text())