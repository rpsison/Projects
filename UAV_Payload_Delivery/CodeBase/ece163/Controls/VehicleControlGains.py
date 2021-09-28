import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeTuningParameters(controlGains , linearizedModel):
    # Things we need to calculate and then return:

    # tuning knobs for lateral control (ignoring Ki_phi)
    Wn_roll =  math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2) # Wn_roll = 0.0,
    Zeta_Roll = (linearizedModel.a_phi1 + linearizedModel.a_phi2*controlGains.kd_roll)/(2*Wn_roll)  # Zeta_roll = 0.0,

    Wn_course = math.sqrt( (VPC.g0/(linearizedModel.Va_trim)  * controlGains.ki_course) )   # Wn_course = 0.0,
    Zeta_course = (VPC.g0/(2 * Wn_course * linearizedModel.Va_trim)) * controlGains.kp_course   # Zeta_course = 0.0,

    Wn_sideslip = math.sqrt( linearizedModel.a_beta2 * controlGains.ki_sideslip ) # Wn_sideslip = 0.0,
    Zeta_sideslip = (linearizedModel.a_beta1 + linearizedModel.a_beta2 * controlGains.kp_sideslip)/(2 * Wn_sideslip)    # Zeta_sideslip = 0.0,

    # tuning knobs for longitudinal control
    Wn_pitch = math.sqrt(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)  # Wn_pitch = 0.0,
    Zeta_pitch = (linearizedModel.a_theta1 + controlGains.kd_pitch * linearizedModel.a_theta3 )/(2 * Wn_pitch)   # Zeta_pitch = 0.0,

# For the longitudinal values we need the K_thetaDC
    K_thetaDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)

    Wn_altitude = math.sqrt(K_thetaDC * linearizedModel.Va_trim * controlGains.ki_altitude)    # Wn_altitude = 0.0	,
    Zeta_altitude = (K_thetaDC * linearizedModel.Va_trim * controlGains.kp_altitude)/(2 * Wn_altitude)   # Zeta_altitude = 0.0,

    Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)   # Wn_SpeedfromThrottle = 0.0,
    Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + linearizedModel.a_V2 * controlGains.kp_SpeedfromThrottle)/(2 * Wn_SpeedfromThrottle)    # Zeta_SpeedfromThrottle = 0.0,

    Wn_SpeedfromElevator = math.sqrt(-1 * K_thetaDC * VPC.g0 * controlGains.ki_SpeedfromElevator)     # Wn_SpeedfromElevator = 0.0,
    Zeta_SpeedfromElevator = (linearizedModel.a_V1 - K_thetaDC * VPC.g0 * controlGains.kp_SpeedfromElevator)/(2 * Wn_SpeedfromElevator)    # Zeta_SpeedfromElevator = 0.0)

    tuneResults = Controls.controlTuning()

    tuneResults.Wn_roll = Wn_roll
    tuneResults.Zeta_roll = Zeta_Roll

    tuneResults.Wn_course = Wn_course
    tuneResults.Zeta_course = Zeta_course

    tuneResults.Wn_sideslip = Wn_sideslip
    tuneResults.Zeta_sideslip = Zeta_sideslip

    tuneResults.Wn_pitch = Wn_pitch
    tuneResults.Zeta_sideslip = Zeta_sideslip

    tuneResults.Wn_pitch = Wn_pitch
    tuneResults.Zeta_pitch = Zeta_pitch

    tuneResults.Wn_altitude = Wn_altitude
    tuneResults.Zeta_altitude = Zeta_altitude

    tuneResults.Wn_SpeedfromThrottle = Wn_SpeedfromThrottle
    tuneResults.Zeta_SpeedfromThrottle = Zeta_SpeedfromThrottle

    tuneResults.Wn_SpeedfromElevator = Wn_SpeedfromElevator
    tuneResults.Zeta_SpeedfromElevator = Zeta_SpeedfromElevator


    return tuneResults



def computeGains(tuningParameters, linearizedModel):

    kp_roll = (tuningParameters.Wn_roll ** 2)/(linearizedModel.a_phi2)
    kd_roll = (2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll - linearizedModel.a_phi1)/(linearizedModel.a_phi2)
    ki_roll = 0.001

    kp_course = (2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim)/VPC.g0
    ki_course = ((tuningParameters.Wn_course ** 2) * linearizedModel.Va_trim)/VPC.g0

    kp_sideslip = ((2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip) - linearizedModel.a_beta1)/linearizedModel.a_beta2
    ki_sideslip = (tuningParameters.Wn_sideslip ** 2)/linearizedModel.a_beta2

    kp_pitch = ((tuningParameters.Wn_pitch ** 2) - linearizedModel.a_theta2)/linearizedModel.a_theta3
    kd_pitch = ((2 * tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch) - linearizedModel.a_theta1)/linearizedModel.a_theta3

    K_thetaDC = (kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + kp_pitch * linearizedModel.a_theta3)

    ki_altitude = (tuningParameters.Wn_altitude ** 2)/(K_thetaDC * linearizedModel.Va_trim)
    kp_altitude = (2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude)/(K_thetaDC * linearizedModel.Va_trim)

    ki_SpeedfromThrottle = (tuningParameters.Wn_SpeedfromThrottle ** 2)/linearizedModel.a_V2
    kp_SpeedfromThrottle = (2 * tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1)/linearizedModel.a_V2

    ki_SpeedfromElevator = - 1 * ((tuningParameters.Wn_SpeedfromElevator ** 2)/(K_thetaDC * VPC.g0))
    kp_SpeedfromElevator = (linearizedModel.a_V1 - 2 * tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator)/(K_thetaDC * VPC.g0)

    outputGains = Controls.controlGains()

    outputGains.kp_roll = kp_roll
    outputGains.kd_roll = kd_roll
    outputGains.ki_roll = ki_roll

    outputGains.kp_course = kp_course
    outputGains.ki_course = ki_course

    outputGains.ki_sideslip = ki_sideslip
    outputGains.kp_sideslip = kp_sideslip

    outputGains.kd_pitch = kd_pitch
    outputGains.kp_pitch = kp_pitch

    outputGains.ki_altitude = ki_altitude
    outputGains.kp_altitude = kp_altitude

    outputGains.ki_SpeedfromThrottle = ki_SpeedfromThrottle
    outputGains.kp_SpeedfromThrottle = kp_SpeedfromThrottle

    outputGains.ki_SpeedfromElevator = ki_SpeedfromElevator
    outputGains.kp_SpeedfromElevator = kp_SpeedfromElevator


    return outputGains





