"""All the vehicle constants that define the various constants that are used in the simulation and control
   model. These are physical constants, mathematical constants, and the rest."""

import math
from ece163.Utilities import MatrixMath
from ece163.Containers import Inputs

LINEARMAX = 250
ROTATEMAX = 180

dT = 1/100	# Time step for simulation
RstarMax = 2500	# maximum radius before considered a straight line

dropZoneError = .25 #maximum distance from drop point to drop payload

Radius = .1
payCD = 0.47  # source https://en.wikipedia.org/wiki/Drag_coefficient
mass = 1  # [kg]

fPa = .0537029184
fPb =   -.740772051
fPc =  -.000115534628
fPd = .432374767
fPe = -23.7620031

rho = 1.2682  # [kg / m^3]
g0 = 9.81  # gravity, [m/s^2]
b = 0.1  # wing-span [m]
c = 0.1  # wing chord [m]
S = 0.01  # wing area [m^2]
e = 1  # Oswald's Efficiency Factor []

AR = b ** 2 / S

# parameters for Aerosonde UAV
InitialSpeed = 25.0	# [m/s]
InitialNorthPosition = 0.0	# displacement to north [m]
InitialEastPosition = 0.0	# displacement to east [m]
InitialDownPosition = -100.0	# [m], negative is above ground
InitialYawAngle = math.radians(0.0)	# initial heading angle [rad]

M = 50.  # barrier function coefficient for angle of attack
alpha0 = math.radians(27.)  # angle at which stall occurs [deg]

Jxx = Jyy = Jzz = 2/5 * mass *Radius**2  # [kg m^2]

Jxz =0  # [kg m^2]

Jbody = [[Jxx, 0., -Jxz], [0., Jyy, 0.], [-Jxz, 0., Jzz]]
Jdet = (Jxx * Jzz - Jxz ** 2)
JinvBody = MatrixMath.scalarMultiply(1. / Jdet, [[Jzz, 0., Jxz], [0., Jdet / Jyy, 0.], [Jxz, 0., Jxx]])

Gamma1 = (Jxx - Jyy + Jzz)* JinvBody[0][2]
Gamma2 = (Jzz * (Jzz - Jyy) + Jxz ** 2) / Jdet
Gamma7 = (Jxx * (Jxx - Jyy) + Jxz ** 2) / Jdet

# Aerodynamic Partial Derivatives for Forces

# Lift
CL0 = 0 # zero angle of attack lift coefficient
CLalpha = math.pi * AR / (1 + math.sqrt(1 + (AR / 2.) ** 2))
CLalpha = 5.61  # given in book
CLq = 7.95  # needs to be normalized by c/2*Va
CLdeltaE = 0.13  # lift due to elevator deflection

# Drag
CDp = 0.0  # minimum drag
CDalpha = 0.03  # drag slope
CD0 = 0.043  # intercept of linarized drag slope
CDq = 0  # drag wrt pitch rate
CDdeltaE = 0.0135  # drag due to elevator deflection

# Pitching Moment
CM0 = 0.0135  # intercept of pitching moment
CMalpha = -2.74  # pitching moment slope
CMq = -38.21  # pitching moment wrt q
CMdeltaE = -0.99  # pitching moment from elevator

# Sideforce
CY0 = 0.
CYbeta = -0.98
CYp = 0.
CYr = 0.
CYdeltaA = 0.075
CYdeltaR = 0.19

# Rolling Moment
Cl0 = 0.
Clbeta = -0.13
Clp = -0.51
Clr = 0.25
CldeltaA = 0.17
CldeltaR = 0.0024

# Yawing Moment
Cn0 = 0.
Cnbeta = 0.073
Cnp = 0.069
Cnr = -0.095
CndeltaA = -0.011
CndeltaR = -0.069

# Propeller Thrust
Sprop = 0.2027 # propellor area [m^2]
kmotor = 80.  # motor constant
kTp = 0.  # motor torque constant
kOmega = 0.  # motor speed constant
Cprop = 1.0  # propeller thrust coefficient

# Alternate Propellor Model
D_prop = 20. * (0.0254)  # prop diameter in m
#
# # Motor parameters
KV = 145.  # from datasheet RPM/V
KQ = (1. / KV) * 60. / (2. * math.pi)  # KQ in N-m/A, V-s/rad
R_motor = 0.042  # ohms
i0 = 1.5  # no-load (zero-torque) current (A)

# Inputs
ncells = 12.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit (from lecture slide)
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357

# roll rate and yaw rate derived parameters
Cp0 = JinvBody[0][0] * Cl0 + JinvBody[0][2] * Cn0
Cpbeta = JinvBody[0][0] * Clbeta + JinvBody[0][2] * Cnbeta
Cpp = JinvBody[0][0] * Clp + JinvBody[0][2] * Cnp
Cpr = JinvBody[0][0] * Clr + JinvBody[0][2] * Cnr
CpdeltaA = JinvBody[0][0] * CldeltaA + JinvBody[0][2] * CndeltaA
CpdeltaR = JinvBody[0][0] * CldeltaR + JinvBody[0][2] * CndeltaR
Cr0 = JinvBody[0][2] * Cl0 + JinvBody[2][2] * Cn0
Crbeta = JinvBody[0][2] * Clbeta + JinvBody[2][2] * Cnbeta
Crp = JinvBody[0][2] * Clp + JinvBody[2][2] * Cnp
Crr = JinvBody[0][2] * Clr + JinvBody[2][2] * Cnr
CrdeltaA = JinvBody[0][2] * CldeltaA + JinvBody[2][2] * CndeltaA
CrdeltaR = JinvBody[0][2] * CldeltaR + JinvBody[2][2] * CndeltaR

# rudder and aileron trim matrix
CprdeltaARinv = MatrixMath.scalarMultiply(1.0 / (CpdeltaA * CrdeltaR - CpdeltaR * CrdeltaA), [[CrdeltaR, -CpdeltaR],
																									[-CrdeltaA, CpdeltaA]])

# Dyden Wind Gust Model Coefficients
DrydenLowAltitudeLight = Inputs.drydenParameters(200.0, 200.0, 50.0, 1.06, 1.06, 0.7)
DrydenLowAltitudeModerate = Inputs.drydenParameters(200.0, 200.0, 50.0, 2.12, 2.12, 1.4)
DrydenHighAltitudeLight = Inputs.drydenParameters(533.0, 533.0, 533.0, 1.5, 1.5, 1.5)
DrydenHighAltitudeModerate = Inputs.drydenParameters(533.0, 533.0, 533.0, 3.0, 3.0, 3.0)
DrydenNoGusts = Inputs.drydenParameters(200.0, 200.0, 50.0, 0.0, 0.0, 0.0)
DrydenNoWind = Inputs.drydenParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

SteadyWinds = [('No Wind',(0.0, 0.0, 0.0)),('Light Wind',(3.0, -5.0, 0.0)),
			   ('Moderate Wind',(-12.0, 3.5, 0.0)), ('Strong Wind',(-16.0, -16.0, 0.0))]
GustWinds = [('No Gusts', DrydenNoGusts),('Light Low Altitude', DrydenLowAltitudeLight), ('Moderate Low Altitude', DrydenLowAltitudeModerate),
			 ('Light High Altitude', DrydenHighAltitudeLight), ('Moderate High Altitude', DrydenHighAltitudeModerate)]

#Min Max Control Inputs for checking limits
minControls = Inputs.controlInputs(0.0, -math.radians(25.0), -math.radians(25.0), -math.radians(25.0))
maxControls = Inputs.controlInputs(1.0, math.radians(25.0), math.radians(25.0), math.radians(25.0))

#Aircraft maneuver limits
bankAngleLimit = 60.0	# [deg] This is aggressive, 30-45 degrees is more usual
courseAngleLimit = 45.0	# [deg] how much course change before saturating integrator
pitchAngleLimit = 30.0	# [deg] This is aggressive, 15-20 degrees is more usual
altitudeHoldZone = 30.0	# [m] the saturation zone when to switch modes on altitude hold
