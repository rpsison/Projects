"""
For consistency we need a class to actually run the simulation. This stub class handles this process.
Useless without subclassing.
"""
import csv
import pickle

class Simulate(object):
	def __init__(self):

		self.dT = .1
		self.time = 0
		self.variableList = list()
		self.inputNames = list()
		self.underlyingModel = None
		self.takenData = list()
		return

	def takeStep(self, **kwargs):
		"""
		takes a single step however is needed. Do not forget to increment time. It will then record the data

		:param kwargs: Each simulate will have different arguments, kwargs is just an indicator it needs to be overwritten.
		:return:
		"""
		self.time += self.dT
		return

	def reset(self):
		"""
		Reset the data taken and the underlying model using its reset function. It can be overwritten if not usable.
		:return:
		"""
		self.time = 0
		self.takenData.clear()
		self.underlyingModel.reset()
		return

	def exportToPickle(self, filename):
		"""
		exports taken data as tuple, first item in tuple is a string list of recorded variables followed by the data

		:param filename: valid file path to write to
		:return: True if successful, false if not
		"""
		try:
			with open(filename, 'wb') as f:
				pickle.dump((self.__buildHeader(), self.takenData), f)
		except OSError as e:
			print(e)
			return False
		return True

	def exportToCSV(self, filename):
		"""
		exports taken data as csv, first line of tuple is the list of variables

		:param filename: valid file path to write to
		:return: True if successful, false if not
		"""
		try:
			with open(filename, 'w', newline='') as csvFile:
				dataFileWriter = csv.writer(csvFile)
				dataFileWriter.writerow(self.__buildHeader())
				dataFileWriter.writerows(self.takenData)
		except OSError as e:
			print(e)
			return False
		return True

	def recordData(self, inputs):
		"""
		used within takeStep. Stores current data to internal list.

		:param inputs: Same set of inputs in same order is passed as list to recordData for their storage
		:return:
		"""
		newDataLine = [self.time] # each line starts with the current time

		# we handle inputs first
		newDataLine.extend(inputs)

		# and then use the variable list to store everything else wanted
		for model, name, variableNames in self.variableList:
			newValues = model()
			for variableName in variableNames:
				newDataLine.append(getattr(newValues, variableName))

		# print(newDataLine)
		self.takenData.append(newDataLine)
		return

	def __buildHeader(self):
		"""
		internal function to build a list of string headers for the output files
		:return:
		"""
		headers = ['time']
		headers.extend(self.inputNames)
		for model, name, variableNames in self.variableList:
			for variableName in variableNames:
				headers.append(".".join([name, variableName]))

		return headers