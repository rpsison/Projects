import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Constants.VehiclePhysicalConstants as VPC

import ece163.Containers.Payload as PL
import ece163.Containers.Target as Tar

from ece163.Containers import States
from ece163.Containers import Controls

from ece163.Constants import PayloadConstants as PC

from ece163.Constants import PayloadPhysicalConstants as PPC
import enum
import math


class DeliveryStates(enum.Enum):
    WAITING = enum.auto()
    FLYINGTO = enum.auto()
    DROPPING = enum.auto()
    DELIVERED = enum.auto()


class PayloadDeliveryControl():
    def __init__(self, targetState):

        self.VCLC = VCLC.VehicleClosedLoopControl(dT=VPC.dT)
        self.chi = 0

        self.target = Tar.Target(targetState)
        self.Payload = PL.Payload()
        self.mode = DeliveryStates.WAITING
        self.dropDist = 0
        self.distToTarget = 0

        return

    def reset(self):

        self.VCLC = VCLC.VehicleClosedLoopControl(dT=VPC.dT)
        self.chi = 0
        self.target = Tar.Target()
        self.Payload = PL.Payload()
        self.mode = DeliveryStates.WAITING
        self.dropDist = 0
        self.distToTarget = 0

        return

    def UpdateDelivery(self, start_signal, Va, altitude, Drop_pn, Drop_pe):

        referenceCommands = Controls.referenceCommands(self.chi, altitude, Va)

<<<<<<< HEAD
        # the Payload update function or some other function would need to produce the translation from VAM most and then we add that to the upper and lower threshold
        upper_thresh_pn = Drop_pn + PPC.dropZoneError
        lower_thresh_pn = Drop_pn - PPC.dropZoneError
        upper_thresh_pe = Drop_pe + PPC.dropZoneError
        lower_thresh_pe = Drop_pe - PPC.dropZoneError
=======
        # calculates the range that the UAV should be inside when it drops the package
        upper_thresh_pn = Drop_pn + 1
        lower_thresh_pn = Drop_pn - 1
        upper_thresh_pe = Drop_pe + 1
        lower_thresh_pe = Drop_pe - 1
>>>>>>> 10ec1e67369ab4ef39095fa73a759716b1f971f9

        # not needed for MVP
        # Target.update(targetState)

        # get the current UAV state, will use to set the payload state equal to this so it follows the UAV path
        UAV_state = self.VCLC.VAM.getVehicleState()


        # transitions for state machine
        if (self.mode == DeliveryStates.WAITING):
            # stay in waiting until user is ready to drop payload
            if (start_signal == 1):
                self.mode = DeliveryStates.FLYINGTO
                

        elif (self.mode == DeliveryStates.FLYINGTO):
            # if UAV enters the drop zone,go to dropping state
            if (upper_thresh_pn > UAV_state.pn and UAV_state.pn > lower_thresh_pn and upper_thresh_pe > UAV_state.pe and UAV_state.pe > lower_thresh_pe):
                self.mode = DeliveryStates.DROPPING

        elif (self.mode == DeliveryStates.DROPPING):
            #wait until the payload has reached the ground, then go to delivered
            if (self.Payload.aeroModel.VDM.state.pd >= 0):
                self.mode = DeliveryStates.DELIVERED
<<<<<<< HEAD
                actualDistanceToTarget = math.sqrt((self.target.pn-self.Payload.aeroModel.getVehicleState().pn)**2 + (self.target.pe-self.Payload.aeroModel.getVehicleState().pe)**2)
                print(actualDistanceToTarget)

=======

        # not needed for MVP, add if we want to make multiple deliveries
>>>>>>> 10ec1e67369ab4ef39095fa73a759716b1f971f9
        #elif (self.mode == DeliveryStates.DELIVERED):
            #self.mode = DeliveryStates.WAITING

        # actions for state machine
        if (self.mode == DeliveryStates.WAITING):
            # in WAITING the payload follows the UAV path, the UAV flies straight
            referenceCommands.courseCommand = 0
            self.VCLC.Update(referenceCommands)
            UAV_state = self.VCLC.VAM.getVehicleState()
            UAV_dot = self.VCLC.VAM.VDM.getVehicleDerivative()
            Payload_aeroModel = self.Payload.Update(UAV_state, UAV_dot)
            

        elif (self.mode == DeliveryStates.FLYINGTO):
            # in FLYINGTO the payload follows the UAV path, the UAV turns towards the drop point and flies that way
            self.VCLC.Update(referenceCommands)
            UAV_state = self.VCLC.VAM.getVehicleState()
            UAV_dot = self.VCLC.VAM.VDM.getVehicleDerivative()
            Payload_aeroModel = self.Payload.Update(UAV_state, UAV_dot)
            

        elif (self.mode == DeliveryStates.DROPPING):
            # in DROPPING the payload drops to the ground, the UAV continues flying
            self.VCLC.Update(referenceCommands)
            Payload_aeroModel = self.Payload.Update()

            

        elif (self.mode == DeliveryStates.DELIVERED):
            # stops the motion of the payload once delivered, the UAV flies straight ahead
            referenceCommands.courseCommand = 0
            self.VCLC.Update(referenceCommands)
            self.Payload.aeroModel.VDM.state.u = 0
            self.Payload.aeroModel.VDM.state.v = 0
            self.Payload.aeroModel.VDM.state.w = 0
            Payload_aeroModel = self.Payload.getAerodynamicsModel()
            
        

        return Payload_aeroModel

    def Update(self, start_signal):

        
        if self.mode == DeliveryStates.FLYINGTO:
            self.CalculateChi()
            self.CalculateDropDist()
        
        elif self.mode == DeliveryStates.DELIVERED:
            self.chi = 0

        Drop_pn, Drop_pe = self.CalculateDropPoint()
        
        # Update delivery returns an aeroModel, for the MVP we don't need the aero model
        PL_aeroModel = self.UpdateDelivery(start_signal, self.target.deliveryAirSpeed, self.target.deliveryAltitude,
                                           Drop_pn, Drop_pe)

        return

    def CalculateChi(self):
        """
        Returns the course angle chi required to create a straight line from the uav to the target point
        """
        state = self.VCLC.VAM.getVehicleState()

        self.chi = math.atan2(self.target.pe - state.pe, self.target.pn - state.pn)

        return self.chi

    def CalculateDropDist(self):
        """
        Returns radius from drop Target to drop package and updates internal variable
        """
        self.dropDist = self.target.deliveryAirSpeed * math.sqrt(self.target.deliveryAltitude * 2 / (VPC.g0))
        return self.dropDist

    def CalculateDropPoint(self):
        """
        Returns a 2-tuple of (dropPe, dropPn) computed from self attributes.
        """
        dPn = self.target.pn + self.dropDist * math.cos(self.chi + math.pi)
        dPe = self.target.pe + self.dropDist * math.sin(self.chi + math.pi)
        return dPn, dPe

