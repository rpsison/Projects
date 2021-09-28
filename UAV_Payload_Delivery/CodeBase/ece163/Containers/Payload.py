"""
Package Delivery Project: Payload.py
Contains the functions needed to:
    Create a payload object
    Set the payloads Aerodynamics model
    Update the payloads location
"""

from ece163.Containers import States
from ece163.Modeling import PayloadAerodynamicsModel as PAM
from ece163.Constants import PayloadConstants
from ece163.Containers import Inputs
from ece163.Constants import PayloadPhysicalConstants as PPC    
import numpy as np
# In preparation for the later stages of the project I'm going to make a PAM alreeady and use it's state variable
# Techinically we could just use a State variable imported from states, but this will work and will be better for
# the later system
class Payload:

    """
    Init and Reset:
        MVP: Just needs a PAM object that we will only use the state for

    """

    def __init__(self, aeroModel=PAM.PayloadAerodynamicsModel(), MVPstate=States.vehicleState()):
        self.aeroModel = aeroModel
        return

    """
    
    """
    def reset(self):
        self.aeroModel = PAM.PayloadAerodynamicsModel()
        return

    # ------------------------------------------------------------------------------------------------------------------

    """
    Getters and Setters:
    Used to get or set values as needed for the system    
    """

    def getAerodynamicsModel(self):
        return self.aeroModel

    def getState(self):
        return self.aeroModel.getVehicleState()

    def setAerodynamicsModel(self, aeroModel):
        self.aeroModel = aeroModel
        return

    def setState(self, MVPState):
        self.aeroModel.setVehicleState(MVPState)
        return

    # ------------------------------------------------------------------------------------------------------------------
    """
    Update
    The update function will take the current state the PDC system is in in order to determined what the payload should be doing.
        
        Waiting/FlyingTo: Payload state continues to be identical to the UAVs state
        Dropping: Payload will update based on its own falling dynamics
        Delivered: Payload will remain at whatever position it was at when it finished falling (IE altitude = 0)
    
    """
    def Update(self, stateIn=None, dotIn=None):
        
        

        if stateIn != None:
            newState = States.vehicleState()
            newDot = States.vehicleState()

            for i in [a for a in dir(newState) if not a.startswith('__')]:
                setattr(newState, i, getattr(stateIn, i))
            for i in [a for a in dir(newDot) if not a.startswith('__')]:
                setattr(newDot, i, getattr(dotIn, i))


            self.aeroModel.setVehicleState(newState)
            self.aeroModel.VDM.setVehicleDerivative(newDot)
        else:
            self.aeroModel.Update()
        return self.aeroModel







