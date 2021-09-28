import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

import numpy as np

class VehicleAerodynamicsModel:
    # "Public" Functions------------------------------------------------------------------------------------------------
    # init and reset functions function for the class
    def __init__(self, initialSpeed=25.0, initialHeight=100):
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        self.WM = WindModel.WindModel()
        return
    def reset(self):
        self.initialSpeed = VPC.InitialSpeed
        self.initialHeight = VPC.InitialDownPosition
        self.WM = WindModel.WindModel()
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        return

    # Get functions ---------------------------------------------------
    def getVehicleDynamicsModel(self):
        return self.VDM

    def getVehicleState(self):
        state = self.VDM.getVehicleState()
        return state

    def getWindModel(self):
        return self.WM

    def setWindModel(self, windModel):
        self.WM = windModel
        return

    # Set functions ---------------------------------------------------
    def setVehicleState(self, state):
        self.VDM.setVehicleState(state)
        return

    def Update(self, controls):
        state = self.VDM.getVehicleState()
        self.WM.Update()
        retForces = self.updateForces(state, controls)
        self.VDM.Update(retForces)

    # "Private" Functions------------------------------------------------------------------------------------------------
    def gravityForces(self, state):
        DCM = state.R
        DCM = MatrixMath.scalarMultiply(VPC.mass, DCM)
        graVector = [[0], [0], [VPC.g0]]
        gForce = Inputs.forcesMoments()
        tempMat = MatrixMath.multiply(DCM, graVector)
        gForce.Fx = tempMat[0][0]
        gForce.Fy = tempMat[1][0]
        gForce.Fz = tempMat[2][0]
        return gForce

    def CalculateCoeff_alpha(self, alpha):
        CLlam = VPC.CL0 + VPC.CLalpha * alpha
        CDlam = VPC.CDp + ((VPC.CL0 + VPC.CLalpha * alpha)**2)/(math.pi * VPC.e * VPC.AR)
        CLturb = 2*math.sin(alpha)*math.cos(alpha)
        CDturb = 2 * math.sin(alpha) * math.sin(alpha)
        # calculate sigma
        num = (1 + np.exp(-VPC.M * (alpha - VPC.alpha0)) + np.exp(VPC.M * (alpha + VPC.alpha0)))
        den = (1 + np.exp(-VPC.M * (alpha - VPC.alpha0))) * (1 + np.exp(VPC.M * (alpha + VPC.alpha0)))
        sigma = num/den

        C_L = (1-sigma) * CLlam + (sigma * CLturb)
        C_D = (1-sigma) * CDlam + (sigma * CDturb)
        C_M = VPC.CM0 + VPC.CMalpha * alpha

        return C_L, C_D, C_M
    def aeroForces(self, state):
        #call coeffAlpha
        C_L, C_D, C_M = self.CalculateCoeff_alpha(state.alpha)
        # calculate fLift and fDrag
        alpha = state.alpha
        beta = state.beta
        retVal = Inputs.forcesMoments()
        fLift = ((VPC.rho * (state.Va ** 2) * VPC.S)/2) * (C_L) + VPC.rho * state.Va * VPC.S * VPC.CLq * ((VPC.c)/4) * state.q
        fDrag = ((VPC.rho * (state.Va ** 2) * VPC.S)/2) * (C_D) + VPC.rho * state.Va * VPC.S * VPC.CDq * ((VPC.c)/4) * state.q

        # with the lift and drag values calculate fx and fz
        fxzMatrix = [[math.cos(alpha), -1 * math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]]
        fdlMat = [[-1*fDrag], [-1*fLift]]
        fxz = MatrixMath.multiply(fxzMatrix, fdlMat)
        retVal.Fx = fxz[0][0]
        retVal.Fz = fxz[1][0]

        # calculate the fy l and n
        fy = ((VPC.rho * (state.Va ** 2) * VPC.S)/2) * (VPC.CY0 + (VPC.CYbeta * beta)) + ((VPC.rho * state.Va * VPC.CYp * VPC.b * state.p)/4) + ((VPC.rho * state.Va * VPC.CYr * VPC.b * state.r)/4)

        m = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.c) / 2) * (VPC.CM0 + VPC.CMalpha * alpha) + (( VPC.S * VPC.rho * VPC.CMq * VPC.c * VPC.c * state.q * state.Va) / 4)

        l = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.b)/2) * (VPC.Cl0 + (VPC.Clbeta * beta)) + ((VPC.rho * state.Va * VPC.Clp * VPC.b * VPC.b * state.p * VPC.S) / 4) + ((VPC.rho * state.Va * VPC.Clr * VPC.b * VPC.b * state.r * VPC.S) / 4)
        n = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.b)/2) * (VPC.Cn0 + (VPC.Cnbeta * beta)) + ((VPC.rho * state.Va * VPC.Cnp * VPC.b * VPC.b * state.p * VPC.S) / 4) + ((VPC.rho * state.Va * VPC.Cnr * VPC.b * VPC.b * state.r * VPC.S) / 4)



        retVal.Fy = fy
        retVal.Mx = l
        retVal.My = m
        retVal.Mz = n
        return retVal

    def CalculatePropForces(self, Va, Throttle):
        # calculate the CT and CQ values
        # need to find J first
        # need K_E and K_T

        Vin = Throttle * VPC.V_max
        K_T = 60/(2 * math.pi * VPC.KV)
        K_E = 60/(2 * math.pi * VPC.KV)
        a = (VPC.rho * (VPC.D_prop ** 5) * VPC.C_Q0)/(4 * (math.pi ** 2))
        b = ((VPC.rho * (VPC.D_prop ** 4) * Va * VPC.C_Q1)/(2 * math.pi)) + (K_T * K_E)/VPC.R_motor
        c = (VPC.rho * (VPC.D_prop ** 3) * (Va ** 2) * VPC.C_Q2) - (K_T * Vin)/(VPC.R_motor) + K_T * VPC.i0

        try:
            Omega = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        except:
            #print("Crashed Propeller Model doing square root (imaginary), Va = {}, dT = {}, b = {}, c = {}".format(Va, DeltaT, b, c))
            Omega = 100.0

        #Omega = ((-1*b + (math.sqrt(((b**2) - 4*a*c))))/(2*a))
        J = (2*math.pi*Va)/(Omega*VPC.D_prop)

        # now we can find the values for CT and CQ
        CT = VPC.C_T0 + VPC.C_T1*(J) + VPC.C_T2*(J ** 2)
        CQ = VPC.C_Q0 + VPC.C_Q1*(J) + VPC.C_Q2*(J ** 2)

        FX = (VPC.rho*(Omega ** 2)*(VPC.D_prop ** 4)*CT)/(4 * (math.pi**2))
        MX = (VPC.rho*(Omega ** 2)*(VPC.D_prop ** 5)*CQ)/(4 * (math.pi**2)) * -1
        return FX, MX

    def controlForces(self, state, controls):
        alpha = state.alpha
        beta = state.beta
        deltaE = controls.Elevator
        #print(deltaE)
        conRet = Inputs.forcesMoments()
        fLift = ((VPC.rho * (state.Va ** 2) * VPC.S) / 2) * (VPC.CLdeltaE * deltaE)
        fDrag = ((VPC.rho * (state.Va ** 2) * VPC.S) / 2) * (VPC.CDdeltaE * deltaE)

        # with the lift and drag values calculate fx and fz
        fxzMatrix = [[math.cos(alpha), -1 * math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]]
        fdlMat = [[-1 * fDrag], [-1 * fLift]]
        fxz = MatrixMath.multiply(fxzMatrix, fdlMat)
        conRet.Fx = fxz[0][0]
        conRet.Fz = fxz[1][0]
        # now calculate the prop forces and add accordingly
        throttle = controls.Throttle
        propFX, propMX = self.CalculatePropForces(state.Va, throttle)
        conRet.Fx = conRet.Fx + propFX

        deltaA = controls.Aileron
        deltaR = controls.Rudder
        #calculate fy
        fy = ((VPC.rho * (state.Va ** 2) * VPC.S) / 2) * (VPC.CYdeltaA * deltaA + VPC.CYdeltaR * deltaR)
        conRet.Fy = fy
        #caluclate l m and n
        m = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.c) / 2 ) * (VPC.CMdeltaE * deltaE)

        l = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.b) / 2) * (VPC.CldeltaA * deltaA + VPC.CldeltaR * deltaR)
        n = ((VPC.rho * (state.Va ** 2) * VPC.S * VPC.b) / 2) * (VPC.CndeltaA * deltaA + VPC.CndeltaR * deltaR)

        conRet.Mx = l + propMX
        conRet.My = m
        conRet.Mz = n


        return conRet


    def updateForces(self, state, control, wind=None):

        #Replacing this code with the Calculate airspeed function call for this system

        #state.Va =  math.hypot(state.u, state.v, state.w)
        #state.alpha = math.atan2(state.w, state.u)  # angle of attack
        #if math.isclose(state.Va, 0.0):  # Sideslip Angle, no airspeed
        #    state.beta = 0.0
        #else:
        #    state.beta = math.asin(state.v / state.Va)  # Sideslip Angle, normal definition

        state.Va, state.alpha, state.beta = self.CalculateAirspeed(state, wind)

        aeroRet = Inputs.forcesMoments()
        aeroRet = self.aeroForces(state)

        controlRet = Inputs.forcesMoments()
        controlRet = self.controlForces(state, control)

        gravRet = self.gravityForces(state)

        sumForces = Inputs.forcesMoments()

        sumForces.Fx = aeroRet.Fx + controlRet.Fx + gravRet.Fx
        sumForces.Fy = aeroRet.Fy + controlRet.Fy + gravRet.Fy
        sumForces.Fz = aeroRet.Fz + controlRet.Fz + gravRet.Fz
        sumForces.Mx = aeroRet.Mx + controlRet.Mx + gravRet.Mx
        sumForces.My = aeroRet.My + controlRet.My + gravRet.My
        sumForces.Mz = aeroRet.Mz + controlRet.Mz + gravRet.Mz

        return sumForces

    def CalculateAirspeed(self, state, wind):

        # Get the windspeed
        u = state.u
        v = state.v
        w = state.w
        if wind == None:
            Wn = 0
            We = 0
            Wd = 0
        else:
            Wn = wind.Wn
            We = wind.We
            Wd = wind.Wd


        UVW_i = [[u], [v], [w]]
        windNed = [[Wn], [We], [Wd]]



        # Take the Wu v w wind values and bring them to the inertial Wind Model Handout
        Ws = math.sqrt(Wn ** 2 + We ** 2 + Wd ** 2)

        chiw = math.atan2(We, Wn)

        if Ws == 0:
            gamw = 0
        else:
            gamw = -1 * math.asin(Wd/Ws)

        # make the matrix Yw and Xw
        R_ywxw = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        R_ywxw[0][0] = math.cos(chiw) * math.cos(gamw)
        R_ywxw[0][1] = math.cos(gamw) * math.sin(chiw)
        R_ywxw[0][2] = -1 * math.sin(gamw)

        R_ywxw[1][0] = -1 * math.sin(chiw)
        R_ywxw[1][1] =  math.cos(chiw)
        R_ywxw[1][2] = 0

        R_ywxw[2][0] = math.cos(chiw) * math.sin(gamw)
        R_ywxw[2][1] =  math.sin(chiw) * math.sin(gamw)
        R_ywxw[2][2] = math.cos(gamw)

        R_ywxwT = MatrixMath.transpose(R_ywxw)

        if wind == None:
            Wind_uvw = [[0], [0], [0]]
        else:
            Wind_uvw = [[wind.Wu], [wind.Wv], [wind.Ww]]


        Wind_uvw = MatrixMath.multiply(R_ywxwT, Wind_uvw)

        sumWind = MatrixMath.add(windNed, Wind_uvw)


        DCM = state.R

        Wb = MatrixMath.multiply(DCM, sumWind)
        UVW_r = MatrixMath.subtract(UVW_i, Wb)

        u_r = UVW_r[0][0]
        v_r = UVW_r[1][0]
        w_r = UVW_r[2][0]

        Va = math.sqrt( u_r ** 2 + v_r ** 2 + w_r ** 2 )




        alpha = math.atan2(w_r, u_r)

        denom = math.sqrt(u_r ** 2 + v_r ** 2 + w_r ** 2)
        if denom == 0:
            beta = math.asin(0)
        else:
            beta = math.asin(v_r/(denom))


        return Va, alpha, beta


