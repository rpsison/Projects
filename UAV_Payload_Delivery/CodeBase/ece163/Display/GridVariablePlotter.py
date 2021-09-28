"""
This is a convenience module designed to make grids of variables using the :mod:`.variablePlotter.variablePlotter`
module. Please refer to that module for specifics of arguments.
"""
from . import variablePlotter
import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets


class GridVariablePlotter(QtWidgets.QWidget):
	def __init__(self, numRows, numCols, plotNames, titles=list(), xLabels=list(), yLabels=list(), useLegends=list(), parent=None):
		"""
		Instantiates a new grid of variables suitable to be added to a gui.

		:param numRows: Number of rows in grid
		:param numCols: Number of columns in grid
		:param plotNames: list of lists with each sublist following the notation of variablePlotter. Note that
			providing fewer than numRows x numCols will result in not the entire grid being filled.
		:param titles: list of optional titles. Can insert None into list to skip title.
		:param xLabels: list of optional xLabels. Can insert None into list to skip.
		:param yLabels: list of optional yLabels. Can insert None into list to skip.
		:param useLegends: list of True/False indicating for each plot if legend is displayed
		"""
		super().__init__(parent)

		self.usedLayout = QtWidgets.QGridLayout()
		self.setLayout(self.usedLayout)

		self.rows = numRows
		self.cols = numCols
		self.plotCount = self.rows*self.cols

		self.variablePlotters =list()

		for row in range(numRows):
			for col in range(numCols):
				curIndex = col+row*numCols
				# print(row, col, curIndex)
				try:
					curNames = plotNames[curIndex]
				except IndexError:
					continue
				try:
					curTitle = titles[curIndex]
				except IndexError:
					curTitle = None
				try:
					curXLabel = xLabels[curIndex]
				except IndexError:
					curXLabel = None
				try:
					curYLabel = yLabels[curIndex]
				except IndexError:
					curYLabel = None
				try:
					curLegend = useLegends[curIndex]
				except IndexError:
					curLegend = False
				except TypeError:
					curLegend = useLegends
				# print(curLegend)
				newVariablePlotter = variablePlotter.variablePlotter(curNames, curTitle, curXLabel, curYLabel, curLegend)
				self.variablePlotters.append(newVariablePlotter)
				self.usedLayout.addWidget(newVariablePlotter, row, col)

	def addNewAllData(self, newData, t=None):
		"""
		adds a new point of data to all elements in grid.

		:param newData: must be given as list of lists to match arguments to variablePlotter
		:param t: list of t variables, if No t is given creates a list of None to pass to variablePlotter
		"""
		if t is None:
			t = [None]*self.plotCount
		for plot, data, curT in zip(self.variablePlotters, newData, t):
			plot.addDataPoint(data, curT)
		return

	def addNewSingleData(self, index, newData, t=None):
		"""
		updates a single plot from the grid.

		:param index: linear index to update. First plot is 0 and continue in row-major order.
		:param newData: list to feed to variablePlotter
		:param t: directly passed to variablePlotter. None if not given.
		"""
		self.variablePlotters[index].addDataPoint(newData, t)
		return

	def clearDataPointsAll(self):
		"""
		Clears all plots in the grid.
		"""
		for plot in self.variablePlotters:
			plot.clearDataPoints()
		return


if QtCore.__name__ == "__main__":
	import sys
	import string
	import random
	class testDialog(QtWidgets.QDialog):
		def __init__(self, rows, cols, parent=None):
			super().__init__(parent)

			self.usedLayout = QtWidgets.QVBoxLayout()
			self.setLayout(self.usedLayout)

			self.plotCount = rows*cols
			self.t = 0

			plotNames = list()
			for index in range(self.plotCount):
				plotNames.append([x for x in string.ascii_letters[0:index+1]]) # we use letters for plot items

			plotTitles = [x for x in string.ascii_uppercase[0:self.plotCount]] # and more for plot titles

			plotTitles[int(self.plotCount/2)] = None  # remove a random one so we don't have one title

			xLabels = [x for x in list(reversed(string.ascii_lowercase))[0:self.plotCount]] # and more for labels
			xLabels[int(self.plotCount/2)] = None

			yLabels = [x for x in string.ascii_lowercase[0:self.plotCount]] # and more for labels
			yLabels[int(self.plotCount/2)] = None

			legends = [True] * self.plotCount # make all legends true
			legends[int(self.plotCount/2)] = False # except one

			self.gridPlot = GridVariablePlotter(rows, cols, plotNames, plotTitles, xLabels, yLabels, legends)
			self.usedLayout.addWidget(self.gridPlot)
			# self.resize(1280, 720)

			# we need to test the data adding
			# going to do so by adding two qtimers which add data

			# first one updates all the plots
			self.multiUpdate = QtCore.QTimer()
			self.multiUpdate.timeout.connect(self.updateAllPlots)
			self.multiUpdate.setInterval(500)
			self.multiUpdate.start()


			# second one updates the first plot at a different rate
			self.singleUpdate = QtCore.QTimer()
			self.singleUpdate.timeout.connect(self.updateFirstPlot)
			self.singleUpdate.setInterval(400)
			self.singleUpdate.start()

			makeNewOne = QtWidgets.QPushButton("Spawn more")
			makeNewOne.clicked.connect(self.addMore)
			self.usedLayout.addWidget(makeNewOne)


			return

		def updateAllPlots(self):
			newValues = list()
			for index in range(self.plotCount):
				newValues.append([random.random() for x in range(index+1)])
			self.t += 3
			self.gridPlot.addNewAllData(newValues, [self.t]*self.plotCount)

		def updateFirstPlot(self):
			self.gridPlot.addNewSingleData(0, [random.random()])

		def addMore(self):
			self.newPlotter = testDialog(random.randrange(1,5), random.randrange(1,5))
			self.newPlotter.show()



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

	TwoByTwoGrid = testDialog(3, 4)
	TwoByTwoGrid.show()

	qtApp.exec()