"""
Holds the vehicle graphics, only operation on it is to return a set of points and meshes with the appropriate rotation/translation
currently just returns the modified points, does not update the base ones. Module uses its baseUnit variable to scale the model to an
arbitrary size for good rendering in the display window.
"""

from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

baseUnit = 1.0

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


		red = [1., 0., 0., 1]
		green = [0., 1., 0., 1]
		blue = [0., 0., 1., 1]
		yellow = [1., 1., 0., 1]
		white = [1.,1.,1.,1]
		black = [0., 0., 0., 0.]

		# squat squarish vehicle as a more complicated practice

		# self.vertices = [[baseUnit,0,0], # [0] nose
		# 				 [baseUnit/2,-baseUnit/2,-baseUnit/4],   # [1] front top left
		# 				 [-baseUnit/2,-baseUnit/2,-baseUnit/4],  # [2] rear top left
		# 				 [-baseUnit/2,baseUnit/2,-baseUnit/4],   # [3] rear top right
		# 				 [baseUnit/2,baseUnit/2,-baseUnit/4],     # [4] front top right
		# 				 [baseUnit/2,baseUnit/2,baseUnit/4],    # [5] front bottom right
		# 				 [baseUnit/2,-baseUnit/2,baseUnit/4],   # [6] front bottom left
		# 				 [-baseUnit/2,-baseUnit/2,baseUnit/4],   # [7] rear bottom left
		# 				 [-baseUnit/2,baseUnit/2,baseUnit/4],    # [8] rear bottom right
		# 				 [-baseUnit/2,0,-0.75*baseUnit],			 # [9] fin top point
		# 				 [-baseUnit/2,0,0],                      # [10] fin rear point
		# 				 [0,0,0]]                      # [11] fin front point
		#
		# self.faces = [[0,1,4],         # [0] nose top
		# 			  [1,3,4],[1,2,3], # [1],[2] body top
		# 			  [0,1,6],         # [3] nose left
		# 			  [1,7,6],[1,7,2], # [4],[5] body left
		# 			  [2,3,8],[2,8,7], # [6],[7] body rear
		# 			  [3,4,8],[8,4,5], # [8],[9] body right
		# 			  [5,0,4],         # [10] nose right
		# 			  [0,5,6],         # [11] nose bottom
		# 			  [6,8,5],[6,8,7], # [12],[13] body bottom
		# 			  [9,10,11]]       # [14] fin
		#
		# self.colors=[yellow,yellow,yellow, # top
		# 			 blue,blue,blue, # left
		# 			 red,red, # back
		#              green,green,green, # right
		# 			 white,white,white, # bottom
		# 			 blue] # tail

		# actual MAV model from Beard Chapter 2

		# define MAV body parameters
		scalingUnit = VPC.b / 6.0	# scaling determined by the wingspan of the aircraft in VehiclePhysicalConstants

		fuse_h = scalingUnit
		fuse_w = scalingUnit
		fuse_l1 = 2 * scalingUnit
		fuse_l2 = scalingUnit
		fuse_l3 = 4 * scalingUnit
		wing_l = scalingUnit
		wing_w = 6 * scalingUnit	# will match the wingspan (VPC.b) in meters exactly
		tail_h = scalingUnit
		tail_l = scalingUnit
		tail_w = 2 * scalingUnit

		self.vertices = [[fuse_l1, 0, 0],  # point 1 [0]
						 [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point 2 [1]
						 [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point 3 [2]
						 [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point 4 [3]
						 [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point 5 [4]
						 [-fuse_l3, 0, 0],  # point 6 [5]
						 [0, wing_w / 2.0, 0],  # point 7 [6]
						 [-wing_l, wing_w / 2.0, 0],  # point 8 [7]
						 [-wing_l, -wing_w / 2.0, 0],  # point 9 [8]
						 [0, -wing_w / 2.0, 0],  # point 10 [9]
						 [-fuse_l3 + tail_l, tail_w / 2.0, 0],  # point 11 [10]
						 [-fuse_l3, tail_w / 2.0, 0],  # point 12 [11]
						 [-fuse_l3, -tail_w / 2.0, 0],  # point 13 [12]
						 [-fuse_l3 + tail_l, -tail_w / 2.0, 0],  # point 14 [13]
						 [-fuse_l3 + tail_l, 0, 0],  # point 15 [14]
						 [-fuse_l3, 0, -tail_h]]  # point 16 [15]

		self.faces = [[0, 1, 2],
					  [0, 1, 4],
					  [0, 3, 4],
					  [0, 3, 2],
					  [5, 2, 3],
					  [5, 1, 2],
					  [5, 1, 4],
					  [5, 3, 4],
					  [6, 7, 9],
					  [7, 8, 9],
					  [10, 11, 12],
					  [10, 12, 13],
					  [5, 14, 15]]

		self.colors = [yellow, yellow, yellow, yellow,
					   blue, blue, blue,
					   red,
					   green, green, green, green,
					   blue]

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
