import datetime
import os
import sys

import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import PyQt5.QtWidgets as QtWidgets

class DataExport(QtWidgets.QWidget):
	def __init__(self, simulateHandle, filePrefix='', parent=None):
		super().__init__()
		self.simulateHandle = simulateHandle
		self.filePrefix = filePrefix
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)

		pickleBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(pickleBox)
		pickleBox.addWidget(QtWidgets.QLabel('Pickle'))
		self.picklePath = QtWidgets.QLineEdit()

		pickleBox.addWidget(self.picklePath)
		pickleBrowseButton = QtWidgets.QPushButton('Browse')
		pickleBrowseButton.clicked.connect(self.choosePicklePath)
		pickleBox.addWidget(pickleBrowseButton)
		pickleRefreshButton = QtWidgets.QPushButton('Refresh')
		pickleBox.addWidget(pickleRefreshButton)
		pickleRefreshButton.clicked.connect(self.updatePicklePath)
		self.updatePicklePath()
		pickleSaveButton = QtWidgets.QPushButton('Save')
		pickleBox.addWidget(pickleSaveButton)
		pickleSaveButton.clicked.connect(self.savePickleFile)
		pickleBox.addStretch()

		csvBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(csvBox)
		csvBox.addWidget(QtWidgets.QLabel('CSV  '))
		self.csvPath = QtWidgets.QLineEdit()
		csvBox.addWidget(self.csvPath)
		csvBrowseButton = QtWidgets.QPushButton('Browse')
		csvBrowseButton.clicked.connect(self.chooseCSVPath)
		csvBox.addWidget(csvBrowseButton)
		csvRefreshButton = QtWidgets.QPushButton('Refresh')
		csvRefreshButton.clicked.connect(self.updateCSVPath)
		self.updateCSVPath()
		csvBox.addWidget(csvRefreshButton)
		csvSaveButton = QtWidgets.QPushButton('Save')
		csvBox.addWidget(csvSaveButton)
		csvSaveButton.clicked.connect(self.saveCSVFile)
		csvBox.addStretch()

		self.usedLayout.addStretch()
		return

	def testButton(self):
		print(len(self.simulateHandle.takenData))

	def generateFileName(self, extension=''):
		fileName = 'run_'
		if self.filePrefix == '':
			fileName += ''
		else:
			fileName += self.filePrefix+'_'
		fileName += datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+extension

		return fileName

	def updatePicklePath(self):
		currentFilePath = self.picklePath.text()
		folderInfo = os.path.split(currentFilePath)
		if not os.path.exists(folderInfo[0]):
			folder = sys.path[0]
		else:
			folder = folderInfo[0]

		filePath = os.path.join(folder, self.generateFileName('.pickle'))
		self.picklePath.setText(filePath)

	def updateCSVPath(self):
		currentFilePath = self.csvPath.text()
		folderInfo = os.path.split(currentFilePath)
		if not os.path.exists(folderInfo[0]):
			folder = sys.path[0]
		else:
			folder = folderInfo[0]

		filePath = os.path.join(folder, self.generateFileName('.csv'))
		self.csvPath.setText(filePath)

	def choosePicklePath(self):
		fileSelect = QtWidgets.QFileDialog(filter='*.pickle')
		fileSelect.setFileMode(QtWidgets.QFileDialog.AnyFile)
		fileSelect.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
		folder, file = os.path.split(self.picklePath.text())
		fileSelect.setDirectory(folder)
		fileSelect.selectFile(self.generateFileName('.pickle'))
		if fileSelect.exec():
			self.picklePath.setText(os.path.normpath(fileSelect.selectedFiles()[0]))
		return

	def chooseCSVPath(self):
		fileSelect = QtWidgets.QFileDialog(filter='*.csv')
		fileSelect.setFileMode(QtWidgets.QFileDialog.AnyFile)
		fileSelect.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
		folder, file = os.path.split(self.csvPath.text())
		fileSelect.setDirectory(folder)
		fileSelect.selectFile(self.generateFileName('.csv'))
		if fileSelect.exec():
			self.csvPath.setText(os.path.normpath(fileSelect.selectedFiles()[0]))
		return

	def saveCSVFile(self):
		filePath = self.csvPath.text()
		self.simulateHandle.exportToCSV(filePath)
		return

	def savePickleFile(self):
		filePath = self.picklePath.text()
		self.simulateHandle.exportToPickle(filePath)
		return
