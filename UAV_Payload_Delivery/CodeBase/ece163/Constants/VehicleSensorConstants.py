"""
Contains the sensor parameters for developing the sensor noise models used to corrupt the actual sensor readings for realistic
noise (both broadband and bias). Corrupted sensors will later be used in estimation.
"""

import math
from ..Constants import VehiclePhysicalConstants as VPC

Pground = 101325.0 				# sea level pressure at ground level

accel_bias = 0.1 * VPC.g0		# 100 millig bias
accel_sigma = 0.025 * VPC.g0	# 25 millig white noise

gyro_bias = math.radians(5.0)			# 5 deg/sec bias limit [rad/s]
gyro_sigma = math.radians(0.15)			# 0.15 deg/sec white noise [rad/s]
gyro_tau = 400.0						# Gauss Markov bias drift model [s]
gyro_eta = math.radians(0.073)	# Gauss Markov driving noise [rad/s]

mag_bias = 500.0				# nT bias on turn-on
mag_sigma = 25.0				# nT noise

baro_bias = 0.1 * 1000.0		# offset
baro_sigma = 0.01 * 1000.0		# in N/m^2 (Pascal)

pitot_bias = 0.02 * 1000.0		# offset in N/m^2
pitot_sigma = 0.002 * 1000.0	# in N/m^2 (Pascal)

magfield = [[22750.0],[5286.8],[41426.3]]										# magnetic field components NED, nT
magfieldTotal = math.hypot(magfield[0][0], magfield[1][0], magfield[2][0])		# total magnetic field
magfieldDeclination = math.radians(13.0 + 4.0/60.0 + 57.0/3600.0)				# magnetic field declination [rad]
magfieldInclination = math.radians(-(60.0 + 35.0/60.0 + 8.0/3600.0))			# magnetic field inclination [rad]

GPS_rate = 1.0											# GPS update rate [Hz]
GPS_tau = 1100.0										# Gauss Markov time constant [s]
GPS_etaHorizontal = 0.21								# Gauss Markov sigma [m]
GPS_etaVertical = 0.4									# Gauss Markov sigma [m]
GPS_sigmaHorizontal = 0.4								# horizontal measurement noise [m]
GPS_sigmaVertical = 0.7									# vertical measurement noise [m]
GPS_sigmaSOG = 0.05										# speed magnitude noise [m/s]
GPS_sigmaCOG = GPS_sigmaSOG / VPC.InitialSpeed			# angular noise, needs to be scaled by ratio of initial to actual speed

