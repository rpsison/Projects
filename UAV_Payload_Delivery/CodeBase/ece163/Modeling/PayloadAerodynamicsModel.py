import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import PayloadDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import PayloadPhysicalConstants as PPC

import numpy as np

class PayloadAerodynamicsModel:
    # "Public" Functions------------------------------------------------------------------------------------------------
    # init and reset functions function for the class
    def __init__(self, initialSpeed=25.0, initialHeight=100):
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.VDM = PayloadDynamicsModel.PayloadDynamicsModel()
        self.WM = WindModel.WindModel()
        return
    def reset(self):
        self.initialSpeed = PPC.InitialSpeed
        self.initialHeight = PPC.InitialDownPosition
        self.WM = WindModel.WindModel()
        self.VDM = PayloadDynamicsModel.PayloadDynamicsModel()
        return

    # Get functions ---------------------------------------------------
    def getPayloadDynamicsModel(self):
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

    def Update(self):
        state = self.VDM.getVehicleState()
        self.WM.Update()
        retForces = self.updateForces(state)
        self.VDM.Update(retForces)

    # "Private" Functions------------------------------------------------------------------------------------------------
    def gravityForces(self, state):
        DCM = state.R
        DCM = MatrixMath.scalarMultiply(PPC.mass, DCM)
        graVector = [[0], [0], [PPC.g0]]
        gForce = Inputs.forcesMoments()
        tempMat = MatrixMath.multiply(DCM, graVector)
        gForce.Fx = tempMat[0][0]
        gForce.Fy = tempMat[1][0]
        gForce.Fz = tempMat[2][0]
        return gForce

    

    def aeroForces(self, state):
        #call coeffAlpha
        C_D = PPC.payCD
        # calculate fLift and fDrag
        alpha = state.alpha
        beta = state.beta
        retVal = Inputs.forcesMoments()
        fLift = 0
        circleArea = np.pi*PPC.Radius**2
        fDrag = ((PPC.rho * (state.Va ** 2) * circleArea)/2) * (C_D)
        bodyV = np.array([state.u,state.v,state.w])
        dragDirectionVec = -bodyV/(math.sqrt(state.u**2 + state.v**2 + state.w**2))
        #print(dragDirectionVec)
        dragBody = fDrag*dragDirectionVec
        
        retVal= Inputs.forcesMoments(Fx=dragBody[0],Fy=dragBody[1],Fz=dragBody[2])

        return retVal

    def updateForces(self, state, wind=None):

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

        
        

        gravRet = self.gravityForces(state)
        sumForces = Inputs.forcesMoments()

        sumForces.Fx = aeroRet.Fx + gravRet.Fx
        sumForces.Fy = aeroRet.Fy + gravRet.Fy
        sumForces.Fz = aeroRet.Fz + gravRet.Fz
        sumForces.Mx = aeroRet.Mx + gravRet.Mx
        sumForces.My = aeroRet.My + gravRet.My
        sumForces.Mz = aeroRet.Mz + gravRet.Mz

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


