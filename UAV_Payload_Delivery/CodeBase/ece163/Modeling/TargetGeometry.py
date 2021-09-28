"""
Holds the vehicle graphics, only operation on it is to return a set of points and meshes with the appropriate rotation/translation
currently just returns the modified points, does not update the base ones. Module uses its baseUnit variable to scale the model to an
arbitrary size for good rendering in the display window.
"""

from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
import pywavefront
import math
baseUnit = 1.6

class VehicleGeometry():
	def __init__(self):
		"""
		defines the vehicle in NED coordinates around the local body frame origin. Rotations and translations will be
		around the [0,0,0] point of this local frame. Has to be in NED or the rotation matrices will not work. Vehicle is
		scaled to match the wing span of the actual vehicle, this all the points are in meters.

		"vertices" is an [n x 3] matrix of xyz points for each vertex of the vehicle;
		"faces" is an [m x3] index matrix of which vertices connect to which face (only triangles allowed for faces);
		"colors" is an [m x 4] matrix where each row is a CMYK definition of that face color
		"""

		white = [0.,0.,1.,0]
		

		self.scene = pywavefront.Wavefront("thomas.obj",create_materials=False, collect_faces=True)
		
		self.vertices = self.scene.vertices

		self.vertices = mm.offset(self.vertices, 1.666,-1.666,-46.6666)
		self.vertices = mm.multiply(self.vertices, Rotations.euler2DCM(0, math.pi/2, 0))
		self.vertices = mm.multiply(self.vertices, Rotations.euler2DCM(0, 0, -math.pi/2))

		self.vertices = mm.scalarMultiply(baseUnit, self.vertices) 
		
		self.vertices = mm.offset(self.vertices, 1200,1000,0)
		self.faces = self.scene.mesh_list[0].faces

		self.colors = [white for face in self.faces]

		return

	def getNewPoints(self, x, y, z, yaw, pitch, roll):
		"""
		Function to get new ENU points of the vehicle in inertial space from Euler angles, NED displacements, and base
		drawing contained within the __init__ function. That is, points to be remapped are contained within self.vertices

		:param x: North Displacement (Pn) in [m]
		:param y: East Displacement (Pe) in [m]
		:param z: Down Displacement (Pd) in [m]
		:param yaw: rotation about inertial down [rad]
		:param pitch: rotation about intermediate y-axis [rad]
		:param roll: rotation about body x-axis [rad]
		:return: Points in inertial EAST-NORTH-UP frame (for plotting)
		"""
		newPoints = self.vertices
		#student code goes here
		rot = Rotations.euler2DCM(yaw, pitch, roll)

		newPoints = mm.multiply(newPoints, rot)
		newPoints = mm.offset(newPoints, x, y, z)
		newPoints = Rotations.ned2enu(newPoints)
		return newPoints
