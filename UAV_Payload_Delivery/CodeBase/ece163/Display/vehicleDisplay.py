"""
This module handles displaying the vehicle in an OpenGL window for easy visualization
"""
import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets
import pyqtgraph.opengl
import pyqtgraph
from ..Modeling import VehicleGeometry
from ..Modeling import PayloadGeometry
from ..Modeling import TargetGeometry
import numpy
from ..Containers import States
import random
import pywavefront
defaultZoom = 10
defaultAzimuth = 45
defaultElevation = 30
defaultZoomTick = 3

metersToPixelRatio = 1

testLine = [[2, 2, 2], [2, 0, 100], [-2, 0, 100]]

class vehicleDisplay(QtWidgets.QWidget):
	updateVehiclePositionSignal = QtCore.pyqtSignal(list)
	updatePayloadPositionSignal = QtCore.pyqtSignal(list)
	def __init__(self, parent=None):
		"""
		sets up the full window with the plane along with a row of camera controls
		"""
		super().__init__(parent)

		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.trackPlane = True
		self.leavePlaneTrail = True

		self.lastPlanePos = pyqtgraph.Vector(0, 0, 0)

		self.lastPayloadPos = pyqtgraph.Vector(0, 0, 0)

		# we will always have the openGLBase Widget
		self.openGLWindow = pyqtgraph.opengl.GLViewWidget()
		self.usedLayout.addWidget(self.openGLWindow)

		# a random grid, will likely be changed afterwards
		self.openGLWindow.setGeometry(0, 0, 1000, 1000)
		self.grid = pyqtgraph.opengl.GLGridItem()
		self.grid.scale(200, 200, 200)
		self.grid.setSize(2000, 2000, 2000)
		self.openGLWindow.addItem(self.grid)

		# and an arbitrary camera starting position
		self.openGLWindow.setCameraPosition(distance=defaultZoom, elevation=defaultElevation, azimuth=defaultAzimuth)

		# a copy of the vehicle, we assume we will always want a vehicle
		self.vehicleDrawInstance = VehicleGeometry.VehicleGeometry()
		self.payloadDrawInstance = PayloadGeometry.VehicleGeometry()
		self.targetDrawInstance = TargetGeometry.VehicleGeometry()
		# we need to grab the vertices for the vehicle each update
		rawPoints = [[y*metersToPixelRatio for y in x] for x in self.vehicleDrawInstance.getNewPoints(0, 0, 0, 0, 0, 0)]
		newVertices = numpy.array(rawPoints)

		rawPayloadPoints = [[y*metersToPixelRatio for y in x] for x in self.payloadDrawInstance.getNewPoints(0, 0, 0, 0, 0, 0)]
		payloadVertices =numpy.array(rawPayloadPoints)

		rawTargetPoints = [[y*metersToPixelRatio for y in x] for x in self.targetDrawInstance.getNewPoints(0, 0, 0, 0, 0, 0)]
		targetVertices =numpy.array(rawTargetPoints)
		# faces and colors only need to be done once
		payloadFaces = numpy.array(self.payloadDrawInstance.faces)
		payloadColors = numpy.array(self.payloadDrawInstance.colors)
		
		targetFaces = numpy.array(self.targetDrawInstance.faces)
		targetColors = numpy.array(self.targetDrawInstance.colors)

		newFaces = numpy.array(self.vehicleDrawInstance.faces)
		newColors = numpy.array(self.vehicleDrawInstance.colors)



		# we convert the vertices to meshdata which allows us not to have to translate points every time
		self.vehicleMeshData = pyqtgraph.opengl.MeshData(vertexes=newVertices, faces=newFaces, faceColors=newColors)
		self.payloadMeshData = pyqtgraph.opengl.MeshData(vertexes=payloadVertices, faces=payloadFaces, faceColors=payloadColors)
		self.targetMeshData = pyqtgraph.opengl.MeshData(vertexes=targetVertices, faces=targetFaces, faceColors=targetColors)
		# and we create the meshItem, we do not smooth to make the triangles be clean colors
		self.openGLVehicle = pyqtgraph.opengl.GLMeshItem(meshdata=self.vehicleMeshData, drawEdges=True, smooth=False, computeNormals=False)
		self.openGLPayload = pyqtgraph.opengl.GLMeshItem(meshdata=self.payloadMeshData, drawEdges=True, smooth=False, computeNormals=False)
		self.openGLTarget = pyqtgraph.opengl.GLMeshItem(meshdata=self.targetMeshData, drawEdges=True, smooth=False, computeNormals=False)
		# always add the vehicle to the display
		self.openGLWindow.addItem(self.openGLVehicle)
		self.openGLWindow.addItem(self.openGLPayload)
		self.openGLWindow.addItem(self.openGLTarget)
		# and add an axis

		self.Axis = pyqtgraph.opengl.GLAxisItem(glOptions='additive')
		self.Axis.setSize(2000, 2000, 2000)
		self.openGLWindow.addItem(self.Axis)
		self.updatePayloadPositionSignal.connect(self.drawNewPayloadPosition)
		self.updateVehiclePositionSignal.connect(self.drawNewVehiclePosition)

		#  we are going to add a line for tracking the plane here, if not on it does nothing
		self.planeTrailLine = pyqtgraph.opengl.GLLinePlotItem()
		# self.planeTrailLine.setData(color=PyQt5.QtGui.QColor("red"), width=2)
		self.planeTrailLine.setData(color=(1, 0, 0, 1), width=1)
		self.openGLWindow.addItem(self.planeTrailLine)
		self.planeTrailLine.mode = 'line_strip'
		self.planeTrailPoints = list()


		self.aribtraryLines = list()
		# #  and another line for supposed paths and the like
		# self.arbitraryLine = pyqtgraph.opengl.GLLinePlotItem()
		# # self.arbitraryLine.setData(color=PyQt5.QtGui.QColor("blue"), width=1)
		# self.arbitraryLine.setData(color=(0, 0, 1, .5), width=1)
		# self.arbitraryLine.mode = 'line_strip'
		# self.openGLWindow.addItem(self.arbitraryLine)
		# self.arbitraryLinePoints = list()

		# self.setAribtraryLine(testLine)
		# self.setAribtraryLine([[0,0,0],[10,10, 0]])
		# self.clearAribtraryLine()
		# we add another hbox for camera controls

		cameraControlBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(cameraControlBox)

		zoomInButton = QtWidgets.QPushButton("Zoom In")
		zoomInButton.clicked.connect(self.ZoomIn)
		zoomOutButton = QtWidgets.QPushButton("Zoom Out")
		zoomOutButton.clicked.connect(self.ZoomOut)
		cameraControlBox.addWidget(zoomInButton)
		cameraControlBox.addWidget(zoomOutButton)

		self.trackButton = QtWidgets.QPushButton("Track")
		self.trackButton.clicked.connect(self.TrackButtonResponse)
		cameraControlBox.addWidget(self.trackButton)

		self.manualButton = QtWidgets.QPushButton("Manual")
		self.manualButton.clicked.connect(self.ManualButtonResponse)
		cameraControlBox.addWidget(self.manualButton)

		self.resetCameraButton = QtWidgets.QPushButton("Reset")
		self.resetCameraButton.clicked.connect(self.resetCameraView)
		cameraControlBox.addWidget(self.resetCameraButton)

		self.__setCameraModeButtons()
		# self.setAribtraryLine(testLine)
		# self.drawLine(testLine)
		# print(self.buildRandomPoints())
		# self.addAribtraryLine(self.buildRandomPoints())
		# self.addAribtraryLine(self.buildRandomPoints())
		# hmm = self.addAribtraryLine(self.buildRandomPoints())
		# self.removeAribtraryLine(hmm)
		# self.removeAllAribtraryLines()
		return

	def sizeHint(self):
		"""
		Tells Qt preferred size for widget
		"""
		return QtCore.QSize(720, 480)

	def updateVehiclePosition(self, newState):
		"""
		Updates the vehicle position

		:param newState: vehicleState instance to extract the needed parameters from
		"""
		self.updateVehiclePositionSignal.emit([newState.pn, newState.pe, newState.pd, newState.yaw, newState.pitch, newState.roll])
		return

	def updatePayloadPosition(self, newState):
		"""
		Updates the vehicle position

		:param newState: vehicleState instance to extract the needed parameters from
		"""
		self.updatePayloadPositionSignal.emit([newState.pn, newState.pe, newState.pd, newState.yaw, newState.pitch, newState.roll])
		return

	def drawNewVehiclePosition(self, newPosition):
		"""
		Handles update of the plane in the window, NEVER CALLED directly.

		:param newPosition: new position as a list
		"""
		# print(newPosition)
		# we simply create a new set of vertices

		rawPoints = self.vehicleDrawInstance.getNewPoints(*newPosition)
		rawPayloadPoints = self.payloadDrawInstance.getNewPoints(*newPosition)
		# print(rawPoints)
		newVertices = numpy.array([[y * metersToPixelRatio for y in x] for x in rawPoints])
		newPayloadVertices = numpy.array([[y * metersToPixelRatio for y in x] for x in rawPayloadPoints])
		# print(newVertices)
		self.vehicleMeshData.setVertexes(newVertices)  # update our mesh with them
		self.payloadMeshData.setVertexes(newPayloadVertices)

		self.openGLVehicle.setMeshData(meshdata=self.vehicleMeshData, smooth=False, computeNormals=False)  # and setMeshData automatically invokes a redraw
		self.openGLPayload.setMeshData(meshdata=self.payloadMeshData, smooth=False, computeNormals=False)

		self.lastPlanePos = pyqtgraph.Vector(newPosition[1], newPosition[0], -newPosition[2])
		
		
		
		return

	def drawNewPayloadPosition(self, newPosition):
		"""
		Handles update of the plane in the window, NEVER CALLED directly.

		:param newPosition: new position as a list
		"""
		# print(newPosition)
		# we simply create a new set of vertices

		
		rawPayloadPoints = self.payloadDrawInstance.getNewPoints(*newPosition)
		# print(rawPoints)
		
		newPayloadVertices = numpy.array([[y * metersToPixelRatio for y in x] for x in rawPayloadPoints])
		# print(newVertices)
		
		self.payloadMeshData.setVertexes(newPayloadVertices)

		
		self.openGLPayload.setMeshData(meshdata=self.payloadMeshData, smooth=False, computeNormals=False)

		
		self.lastPayloadPos = pyqtgraph.Vector(newPosition[1], newPosition[0], -newPosition[2])
		
		if self.trackPlane:
			self.openGLWindow.setCameraPosition(pos=self.lastPayloadPos)
		if self.leavePlaneTrail:
			self.planeTrailPoints.append([newPosition[1], newPosition[0], -newPosition[2]])
			hmm = numpy.array(self.planeTrailPoints)
			tempColor = numpy.array([[1., 0., 0., 1]])
			colors = numpy.tile(tempColor, (hmm.shape[0], 1))
			self.planeTrailLine.setData(pos=numpy.array(self.planeTrailPoints), color=colors)
		return
	def ZoomIn(self):
		"""
		Zooms in by default tick
		"""
		self.openGLWindow.opts['distance'] -= defaultZoomTick
		self.openGLWindow.update()
		return

	def ZoomOut(self):
		"""
		Zooms out by default tick
		"""
		self.openGLWindow.opts['distance'] += defaultZoomTick
		self.openGLWindow.update()
		return

	def __setCameraModeButtons(self):
		self.trackButton.setDisabled(self.trackPlane)
		self.manualButton.setDisabled(not self.trackPlane)

	def ManualButtonResponse(self):
		"""
		Sets camera to manual mode
		"""
		self.trackPlane = False
		self.__setCameraModeButtons()
		self.openGLWindow.setCameraPosition(pos=pyqtgraph.Vector(0, 0, 0))
		return

	def TrackButtonResponse(self):
		"""
		Sets Camera to Track plane
		"""
		self.trackPlane = True
		self.__setCameraModeButtons()
		self.openGLWindow.setCameraPosition(pos=self.lastPlanePos)
		self.openGLWindow.update()
		return

	def resetCameraView(self):
		"""
		Resets the camera to the default azimuth, elevation and zoom. This does not change the tracking mode
		"""
		self.openGLWindow.setCameraPosition(distance=defaultZoom, azimuth=defaultAzimuth, elevation=defaultElevation)

	def reset(self, resetState=None):
		"""
		resets the elements that need to be reset
		"""
		self.resetCameraView()
		self.planeTrailPoints.clear()
		if resetState is not None:
			self.updateVehiclePosition(resetState)
		else:
			self.updateVehiclePosition(States.vehicleState())


	def addAribtraryLine(self, points, color=(1.0, 1.0, 1.0, 1.0)):
		"""
		adds an aribtrary line to the plot. This is done oddly due to some bugs that were not resolvable.
		read the comments carefully.

		:param points:
		:param color:
		:return:
		"""
		pointArray = list()  # we need to manipulate the points
		pointArray.append(points[0])  # add the first point
		for item in points[1:]:  # iterate through the rest of the list
			pointArray.append(item)  # each points get added twice
			pointArray.append(item)  # so that we have the appropriate number of line segments
		# print(pointArray)
		colors = numpy.tile(color, (len(pointArray), 1))  # there are also issues with static colors so instead we paint each line
		numpyPoints = numpy.array(pointArray)
		newLine = pyqtgraph.opengl.GLLinePlotItem()
		self.aribtraryLines.append(newLine)
		self.openGLWindow.addItem(newLine)
		newLine.setData(pos=numpyPoints, color=colors, width=1, mode='lines')
		return newLine
		return

	def removeAribtraryLine(self, line):
		if line in self.aribtraryLines:
			self.aribtraryLines.remove(line)
			self.openGLWindow.removeItem(line)
			return True
		return False

	def removeAllAribtraryLines(self):
		for line in self.aribtraryLines:
			self.openGLWindow.removeItem(line)
		self.aribtraryLines.clear()
		return

	def buildRandomPoints(self):
		"""
		just builds a set of points within a range for random testing.
		:return:
		"""
		points = list()
		for i in range(random.randrange(5, 30)):
			points.append([random.uniform(-50, 50), random.uniform(-50, 50), random.uniform(-50, 50)])

		return points

	def getRandomColor(self):
		"""
		just creates a random color for use in creating aribtrary lines

		:return:
		"""
		return (random.random(), random.random(), random.random())
