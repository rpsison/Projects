"""
VehiclePerturbationsMoedl: Contains the utility funcitons for perturbations
Author Matthew Bennett mabennet@ucsc.edu
"""

import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath


def CreateTransferFunction(trimState, trimInputs):

    tF = Linearized.transferFunctions()

    #Copied from the trimstate
    tF.Va_trim = trimState.Va
    tF.alpha_trim = trimState.alpha
    tF.beta_trim = trimState.beta
    tF.gamma_trim = trimState.pitch - trimState.alpha
    tF.theta_trim = trimState.pitch
    tF.phi_trim = trimState.roll

    #Beard 5.23 and 5.24
    tF.a_phi1 = -1 * (VPC.rho * (trimState.Va ** 2) * VPC.S * VPC.b  * VPC.Cpp)/2 * (VPC.b/(2 * trimState.Va))
    tF.a_phi2 = (VPC.rho * (trimState.Va ** 2) * VPC.S * VPC.b * VPC.CpdeltaA)/2

    #Beard 5.28
    tF.a_beta1 = -1 * (VPC.rho * trimState.Va * VPC.S)/(2* VPC.mass) * VPC.CYbeta
    tF.a_beta2 = (VPC.rho * trimState.Va * VPC.S)/(2 * VPC.mass) * VPC.CYdeltaR

    #Beard 5.29
    tF.a_theta1 = -1 * (VPC.rho * (trimState.Va ** 2) * VPC.c * VPC.S)/(2 * VPC.Jyy) * VPC.CMq * (VPC.c)/(2 * trimState.Va)
    tF.a_theta2 = -1 * (VPC.rho * (trimState.Va ** 2) * VPC.c * VPC.S)/(2 * VPC.Jyy) * VPC.CMalpha
    tF.a_theta3 = (VPC.rho * (trimState.Va ** 2) * VPC.c * VPC.S)/(2 * VPC.Jyy) * VPC.CMdeltaE



    #Supplemental UAV page 26
    # First we need the values of the system using the helper functions
    dTdDeltaT = dThrust_dThrottle(trimState.Va, trimInputs.Throttle)
    dTdVa = dThrust_dVa(trimState.Va, trimInputs.Throttle)

    a_V1 = ((VPC.rho * trimState.Va * VPC.S)/VPC.mass) * (VPC.CD0 + VPC.CDalpha * trimState.alpha + VPC.CDdeltaE * trimInputs.Elevator)
    tF.a_V1 = a_V1 - (((1)/(VPC.mass)) * dTdVa)

    tF.a_V2 = (1/VPC.mass) * dTdDeltaT

    tF.a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)
    return tF

def dThrust_dThrottle( Va, Throttle, epsilon=0.01):
    tempAeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    dummyMoment = 0;

    dTdDeltaT, dummyMoment = tempAeroModel.CalculatePropForces(Va, (Throttle + epsilon))
    tempForce, dummyMoment = tempAeroModel.CalculatePropForces(Va, Throttle)

    dTdDeltaT = dTdDeltaT - tempForce
    dTdDeltaT = dTdDeltaT/epsilon
    return dTdDeltaT


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    tempAeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    dummyMoment = 0

    dTdVa, dummyMoment = tempAeroModel.CalculatePropForces((Va + epsilon), Throttle)
    tempForce, dummyMoment = tempAeroModel.CalculatePropForces(Va, Throttle)

    dTdVa = dTdVa - tempForce
    dTdVa = dTdVa / epsilon
    return dTdVa


