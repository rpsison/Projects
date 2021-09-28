"""
	This module provides a wrapper for the pyqtgraph plotwidget and allowing for a simplified thread safe interface. After
	initialziation the normal usage only involves invoking :func:`addDataPoint` to add data to a plot.
"""
import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets
import pyqtgraph
import sys


class variablePlotter(pyqtgraph.PlotWidget):
	"""
		Class to handle a single plot with an arbitrary number of lines. After initialization the normal usage only
		involves invoking :func:`addDataPoint` to add data to a plot.
	"""
	newDataSignal = QtCore.pyqtSignal(list, object)
	def __init__(self, plotNames, title=None, xLabel=None, yLabel=None, useLegend=True , parent=None):
		"""
		Creates a new variablePlotter. Only required item is a list containing names for each plot.

		:param plotNames: Python list of lineLabels. Used for the legend and also to tell how many lines to plot.
		:param title: Title of plot.
		:param xLabel: Label x-axis if desired, appears on the bottom
		:param yLabel: Label y-axis if desired, appears on the left
		:param useLegend: display legend of plots given by plotNames
		"""
		super().__init__(parent)
		self.plotNames = plotNames
		self.plotHandles = list()
		self.dataPoints = list()
		self.timePoints = list()
		if useLegend:
			self.getPlotItem().addLegend()
		for x, name in enumerate(self.plotNames):
			self.dataPoints.append(list())
			self.plotHandles.append(self.getPlotItem().plot(name=name, pen=pyqtgraph.intColor(x)))

		if title is not None:
			self.getPlotItem().setTitle(title)
		if xLabel is not None:
			self.getPlotItem().setLabel('bottom', xLabel)
		if yLabel is not None:
			self.getPlotItem().setLabel('left', yLabel)

		self.newDataSignal.connect(self._ProcessNewPlotData)
		return

	def addDataPoint(self, newDataPoint, t=None):
		"""
		Adds one data plot for each line with an optional timestep via thread safe signal. Does not update the plot itself.
				
		:param newDataPoint: python list representing the new point for each line in plot.  
		:param t: x coordinate for new point. If not present last point will have one added to it.
		"""""
		self.newDataSignal.emit(newDataPoint, t)

	def clearDataPoints(self):
		"""
		Clears data for associated lines in plot
		"""
		self.timePoints.clear()
		for dataList, plotHandle in zip(self.dataPoints, self.plotHandles):
			dataList.clear()
			plotHandle.setData(self.timePoints, dataList)

	def _ProcessNewPlotData(self, newDataPoint, t=None):
		if t is None:
			try:
				self.timePoints.append(self.timePoints[-1]+1)
			except IndexError:
				self.timePoints.append(0)
		else:
			self.timePoints.append(t)
		for dataPoint, dataList, plotHandle in zip(newDataPoint, self.dataPoints, self.plotHandles):
			dataList.append(dataPoint)
			plotHandle.setData(self.timePoints, dataList)
			# print(dataPoint)


if QtCore.__name__ == "__main__":
	import math
	class testDialog(QtWidgets.QDialog):
		def __init__(self, parent=None):
			super().__init__(parent)

			self.usedLayout = QtWidgets.QVBoxLayout()
			self.setLayout(self.usedLayout)

			self.timeStep = 0

			plotBox = QtWidgets.QHBoxLayout()
			self.usedLayout.addLayout(plotBox)

			self.firstPlot = variablePlotter(['s_s', 'y'], 'Hi_s', 'y', 't')

			plotBox.addWidget(self.firstPlot)

			self.secondPlot = variablePlotter(['s'])
			plotBox.addWidget(self.secondPlot)

			self.thirdPlot = variablePlotter(['sine', 'cos'])
			plotBox.addWidget(self.thirdPlot)

			self.firstSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
			self.usedLayout.addWidget(self.firstSlider)
			self.firstSlider.valueChanged.connect(self.updatePlots)
			self.firstSlider.setFocus()

			self.secondSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
			self.usedLayout.addWidget(self.secondSlider)
			self.secondSlider.valueChanged.connect(self.updatePlots)

			clearButton = QtWidgets.QPushButton("Clear Trig Plot")
			self.usedLayout.addWidget(clearButton)
			clearButton.clicked.connect(self.clearTrigPlot)



		def updatePlots(self):
			# print('hi')
			self.timeStep += 1
			firstSliderValue = float(self.firstSlider.value())
			secondSliderValue = float(self.secondSlider.value())
			self.firstPlot.addDataPoint([firstSliderValue, secondSliderValue], self.timeStep)
			self.secondPlot.addDataPoint([secondSliderValue])
			self.thirdPlot.addDataPoint([math.sin(self.timeStep/10), math.cos(self.timeStep/10)])
			return

		def clearTrigPlot(self):
			self.thirdPlot.clearDataPoints()
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
	displaySomePlots = testDialog()
	displaySomePlots.show()
	qtApp.exec()