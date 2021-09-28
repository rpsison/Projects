"""
File contains the class primitive for the collection of sensors available on the UAV.
"""

import math

testingAbs_tol = 1e-6

class vehicleSensors:
	def __init__(self,  gyro_x=0.0,  gyro_y=0.0,  gyro_z=0.0,  accel_x=0.0,  accel_y=0.0,  accel_z=0.0,  mag_x=0.0,  mag_y=0.0,  mag_z=0.0,  baro=0.0,  pitot=0.0,  gps_n=0.0,  gps_e=0.0,  gps_alt=0.0,  gps_sog=0.0,  gps_cog=0.0, ):
		"""
		Defines the typical sensor suite on the UAV. This includes a 3-axis accelerometer, a 3-axis gyro, and a 3-axis
		magnetometer. A simple barometer (absolute pressure sensor) and a pitot tube (differential pressure sensor) are
		also included. Lastly, GPS is included, providing position and both speed over ground (SOG) and course over
		ground (COG).
		"""
		# gyros
		self.gyro_x = gyro_x
		self.gyro_y = gyro_y
		self.gyro_z = gyro_z
		# accelerometers
		self.accel_x = accel_x
		self.accel_y = accel_y
		self.accel_z = accel_z
		# magnetometers
		self.mag_x = mag_x
		self.mag_y = mag_y
		self.mag_z = mag_z
		# pressure sensors
		self.baro = baro
		self.pitot = pitot
		# gps
		self.gps_n = gps_n
		self.gps_e = gps_e
		self.gps_alt = gps_alt
		self.gps_sog = gps_sog
		self.gps_cog = gps_cog
		return

	def __repr__(self):
		return "{0.__name__}(gyro_x={1.gyro_x}, gyro_y={1.gyro_y}, gyro_z={1.gyro_z}, accel_x={1.accel_x}, accel_y={1.accel_y}, " \
			   "accel_z={1.accel_z}, mag_x={1.mag_x}, mag_y={1.mag_y}, mag_z={1.mag_z}, baro={1.baro}, pitot={1.pitot}, " \
			   "gps_n={1.gps_n}, gps_e={1.gps_e}, gps_alt={1.gps_alt}, gps_sog={1.gps_sog}, gps_cog={1.gps_cog})".format(type(self), self)

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all(
					[math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in
					 ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z', 'baro',
					  'pitot', 'gps_n', 'gps_e', 'gps_alt', 'gps_sog', 'gps_cog']]):
				return False
			else:
				return True
		else:
			return NotImplemented

