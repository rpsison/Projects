"""
Wind Model class, Handles all simulation of wind using the dryden wind model
Matthew Bennett mabennet@ucsc.edu
"""


import math
import random
from ..Containers import States
from ..Containers import  Inputs
from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC

class WindModel:
    def __init__(self, dT=0.01, Va=25.0, drydenParamters=Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw=0.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0)):
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParamters

        self.Phiu = None
        self.Gammau = None
        self.Hu = None

        self.Phiv = None
        self.Gammav = None
        self.Hv = [[0,0]]

        self.Phiw = None
        self.Gammaw = None
        self.Hw = None

        self.xu = [[0]]
        self.xv =  [[0], [0]]
        self.xw = [[0], [0]]

        self.CreateDrydenTransferFcns(dT, Va, drydenParamters)

        self.windState = States.windState()
        return

    def reset(self):
        self.dT = VPC.dT
        self.Va = 25.0
        return
# Update function-------------------------------------------------------------------------------------------------------

    def Update(self, uu=None, uv=None, uw=None):
        # Get the wind values from the parameters or from random generation
        if uu is None:
            uu = random.gauss(0, 1)
        if uv is None:
            uv = random.gauss(0, 1)
        if uw is None:
            uw = random.gauss(0, 1)


       # Update xu------------------------------------------------------------------------------------------------------
        xminu = self.xu
        xplusu = mm.add(mm.multiply(self.Phiu, xminu), mm.scalarMultiply(uu, self.Gammau))  # step 2
        Wu = mm.multiply(self.Hu, xplusu) # step 3
        self.xu = xplusu # step 4

       # Update xv------------------------------------------------------------------------------------------------------
        xminv = self.xv
        xplusv = mm.add(mm.multiply(self.Phiv, xminv), mm.scalarMultiply(uv, self.Gammav)) # step 2
        Wv = mm.multiply(self.Hv, xplusv) # step 3
        self.xv = xplusv # step 4

       # Update xw------------------------------------------------------------------------------------------------------
        xminw = self.xw
        xplusw = mm.add(mm.multiply(self.Phiw, xminw), mm.scalarMultiply(uw, self.Gammaw))  # step 2
        Ww = mm.multiply(self.Hw, xplusw) # step 3
        self.xw = xplusw # step 4

        # Put all of the needed values into the system.
        self.windState.Wu = Wu[0][0]
        self.windState.Wv = Wv[0][0]
        self.windState.Ww = Ww[0][0]


        #step 5 should already be handled as far as I can tell, its just when the sim calls Update() again





# getters and setters --------------------------------------------------------------------------------------------------
    def getDrydenTransferFns(self):
        return self.Phiu, self.Gammau, self.Hu, self.Phiv, self.Gammav, self.Hv, self.Phiw, self.Gammaw, self.Hw

    def getWind(self):
        return self.windState

    def setWind(self, windState):
        self.windState = windState
        return
    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw=0.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0)):
        self.windState.Wn = Wn
        self.windState.We = We
        self.windState.Wd = Wd

        self.CreateDrydenTransferFcns(self.dT, self.Va, drydenParamters)

        return

# --------------------------------------------------------------------------------------------------
    def CreateDrydenTransferFcns(self, dT, Va, drydenParamters):
        # take the parameters and separate into the needed parts--------------------------------------------------------
        Lu = drydenParamters.Lu
        Lv = drydenParamters.Lv
        Lw = drydenParamters.Lw

        sigmau = drydenParamters.sigmau
        sigmav = drydenParamters.sigmav
        sigmaw = drydenParamters.sigmaw

        # setup other needed variables----------------------------------------------------------------------------------
        e = math.e
        # calculate the u values----------------------------------------------------------------------------------------

        #need to handle the case of Lu LV or Lw is 0
        # going to use the method outlined in Piazza @313 PS Thank You Christian Alexis Ocon
        if drydenParamters.Lu == 0.0:
            self.Phiu = [[1]]
            self.Gammau = [[0]]
            self.Hu = [[1]]

        else:
            self.Phiu = [[e ** ((-Va / Lv) * dT)]]

            self.Gammau = [[Lu/Va * (1- e ** (-1 * (Va/Lu) * dT ))]]

            self.Hu = [[sigmau * math.sqrt( (2*Va)/(math.pi * Lu) )]]



        # calculate the v values----------------------------------------------------------------------------------------
        if drydenParamters.Lv == 0.0:
            self.Phiv = [[1, 0], [0, 1]]
            self.Gammav = [[0], [0]]
            self.Hv = [[1, 1]]

        else:
            phiMatv = [[1 - (Va / Lv) * dT, -((Va / Lv) ** 2) * dT], [dT, 1 + (Va / Lv) * dT]]

            gammaMatv = [[dT], [((Lv / Va) ** 2) * ((e ** ((Va / Lv) * dT)) - 1) - (Lv) / (Va) * dT]]

            Hmatv = [[1, (Va) / (math.sqrt(3) * Lv)]]

            phiMatv = mm.scalarMultiply(e ** ((-Va / Lv) * dT), phiMatv)

            scale = (e ** ((-Va / Lv) * dT))

            gammaMatv[0][0] = gammaMatv[0][0] * scale
            gammaMatv[1][0] = gammaMatv[1][0] * scale

            Hmatv[0][0] = (sigmav * math.sqrt((3 * Va) / (math.pi * Lv))) * Hmatv[0][0]
            Hmatv[0][1] = (sigmav * math.sqrt((3 * Va) / (math.pi * Lv))) * Hmatv[0][1]

            self.Phiv = phiMatv
            self.Gammav = gammaMatv
            self.Hv = Hmatv


        # calculate the w values----------------------------------------------------------------------------------------
        if drydenParamters.Lw == 0.0:
            self.Phiw = [[1, 0], [0, 1]]
            self.Gammaw = [[0], [0]]
            self.Hw = [[1, 1]]
        else:
            e = math.e

            phiMatw = [[1 - (Va / Lw) * dT, -((Va / Lw) ** 2) * dT], [dT, 1 + (Va / Lw) * dT]]

            gammaMatw = [[dT], [((Lw / Va) ** 2) * ((e ** ((Va / Lw) * dT)) - 1) - (Lw) / (Va) * dT]]

            Hmatw = [[1, (Va) / (math.sqrt(3) * Lw)]]

            phiMatw = mm.scalarMultiply(e ** ((-Va / Lw) * dT), phiMatw)

            scale = (e ** ((-Va / Lw) * dT))

            gammaMatw[0][0] = gammaMatw[0][0] * scale
            gammaMatw[1][0] = gammaMatw[1][0] * scale

            Hmatw[0][0] = (sigmaw * math.sqrt((3 * Va) / (math.pi * Lw))) * Hmatw[0][0]
            Hmatw[0][1] = (sigmaw * math.sqrt((3 * Va) / (math.pi * Lw))) * Hmatw[0][1]

            self.Phiw = phiMatw
            self.Gammaw = gammaMatw
            self.Hw = Hmatw

        return