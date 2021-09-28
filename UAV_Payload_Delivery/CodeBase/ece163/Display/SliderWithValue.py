"""
Simple convenience widget allowing for a slider with arbitrary range and an attached value display.
"""
import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets

class SliderWithValue(QtWidgets.QWidget):
	internalMaxValue = 1000
	internalMinValue = 0
	def __init__(self, name, minValue=0, maxValue=100, startValue= 0, onChangePointer=None, parent=None):
		"""
		Generates a new slider with value and returns the qwidget class to be added to a layout manager

		:param name: Name of value to be displayed
		:param minValue: minimum value of slider
		:param maxValue: maximum value of slider
		:param startValue: where does the slider start, defaults to zero. Also where slider goes to at reset.
		:param onChangePointer: function pointer that is called when the slider value is changed, optional.
		"""
		super().__init__(parent)

		self.min = minValue
		self.max = maxValue
		self.name = name
		self.funcPointer = onChangePointer
		if startValue < self.min:
			startValue = self.min
		if startValue > self.max:
			startValue = self.max
		self.curValue = startValue
		self.ratio = (self.max-self.min)/(self.internalMaxValue-self.internalMinValue)
		# print(self.ratio)

		self.usedLayout = QtWidgets.QHBoxLayout()
		self.setLayout(self.usedLayout)
		self.usedLayout.addWidget(QtWidgets.QLabel("{}".format(name)))
		self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
		self.slider.setMinimum(self.internalMinValue)
		self.slider.setMaximum(self.internalMaxValue)
		self.slider.valueChanged.connect(self.updateValue)

		self.usedLayout.addWidget(self.slider)

		self.valueText = QtWidgets.QLabel()
		self.usedLayout.addWidget(self.valueText)

		self.usedLayout.addStretch()
		self.startTick = (startValue - self.min)/self.ratio
		# print(startTick)
		self.slider.setValue(self.startTick)
		self.updateValue(self.startTick)

		return

	def updateValue(self, newValue):
		transformedValue = newValue*self.ratio+self.min
		self.curValue = transformedValue
		self.valueText.setText("{:0.2f}".format(transformedValue))
		if self.funcPointer is not None:
			self.funcPointer(transformedValue, self.name)
		return

	def resetSlider(self):
		"""
		Resets the slider to the start position
		"""
		self.slider.setValue(self.startTick)
		self.updateValue(self.startTick)