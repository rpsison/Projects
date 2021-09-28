"""
Package Delivery Project: PayloadDeliveryControl.py

Control System for the Payload Delivery
This system is operated by the client through the update function this

"""

import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Constants.VehiclePhysicalConstants as VPC

import ece163.Containers.Payload as PL
import ece163.Containers.Target as Tar

from ece163.Containers import Controls

from ece163.Constants import PayloadPhysicalConstants as PPC

import enum
import math


class DeliveryStates(enum.Enum):
    WAITING = enum.auto()
    CLIMBING = enum.auto()
    FLYINGTO = enum.auto()
    DROPPING = enum.auto()
    DELIVERED = enum.auto()


class PayloadDeliveryControl():
    def __init__(self, targetState):

        self.VCLC = VCLC.VehicleClosedLoopControl(dT=VPC.dT)
        self.chi = 0

        self.target = Tar.Target(state=targetState)
        self.Payload = PL.Payload()
        self.mode = DeliveryStates.WAITING
        self.dropDist = 0
        self.distToTarget = 0
        self.referenceCommands = Controls.referenceCommands()

        return

    def reset(self):

        self.VCLC = VCLC.VehicleClosedLoopControl(dT=VPC.dT)
        self.chi = 0
        self.target = Tar.Target()
        self.Payload = PL.Payload()
        self.mode = DeliveryStates.WAITING
        self.dropDist = 0
        self.distToTarget = 0
        self.referenceCommands = Controls.referenceCommands()
        return

    def getVehicleState(self):
        return self.VCLC.getVehicleState()

    def getVehicleAerodynamicsModel(self):
        return self.VCLC.getVehicleAerodynamicsModel()

    def setControlGains(self, controls):
        self.VCLC.setControlGains(controls)

    def setTrimInputs(self, trims):
        self.VCLC.setTrimInputs(trims)

    def getVehicleControlSurfaces(self):
        return self.VCLC.getVehicleControlSurfaces()

    def UpdateDelivery(self, start_signal, Va, altitude, Drop_pn, Drop_pe):

        self.referenceCommands = Controls.referenceCommands(self.chi, altitude, Va)

        # calculates the range that the UAV should be inside when it drops the package
        upper_thresh_pn = Drop_pn + PPC.dropZoneError
        lower_thresh_pn = Drop_pn - PPC.dropZoneError
        upper_thresh_pe = Drop_pe + PPC.dropZoneError
        lower_thresh_pe = Drop_pe - PPC.dropZoneError


        # not needed for MVP
        # Target.update(targetState)

        # get the current UAV state, will use to set the payload state equal to this so it follows the UAV path
        UAV_state = self.VCLC.VAM.getVehicleState()


        # transitions for state machine
        if (self.mode == DeliveryStates.WAITING):
            # stay in waiting until user is ready to drop payload
            if (start_signal == 1):
                self.mode = DeliveryStates.CLIMBING
        elif (self.mode == DeliveryStates.CLIMBING):
            # stay in waiting until user is ready to drop payload
            
            if (abs(-UAV_state.pd-altitude) <= 1):
                self.mode = DeliveryStates.FLYINGTO        

        elif (self.mode == DeliveryStates.FLYINGTO):
            # if UAV enters the drop zone,go to dropping state
            if (upper_thresh_pn > UAV_state.pn and UAV_state.pn > lower_thresh_pn and upper_thresh_pe > UAV_state.pe and UAV_state.pe > lower_thresh_pe):
                self.mode = DeliveryStates.DROPPING

        elif (self.mode == DeliveryStates.DROPPING):
            #wait until the payload has reached the ground, then go to delivered
            if (self.Payload.aeroModel.VDM.state.pd >= 0):
                self.mode = DeliveryStates.DELIVERED
                
                actualDistanceToTarget = math.sqrt((self.target.pn-self.Payload.aeroModel.getVehicleState().pn)**2 + (self.target.pe-self.Payload.aeroModel.getVehicleState().pe)**2)
                print("Distance from target: %.3f meters"%(actualDistanceToTarget))


        # not needed for MVP, add if we want to make multiple deliveries
        #elif (self.mode == DeliveryStates.DELIVERED):
            #self.mode = DeliveryStates.WAITING

        # actions for state machine
        if (self.mode == DeliveryStates.WAITING):
            # in WAITING the payload follows the UAV path, the UAV flies straight
            self.referenceCommands.courseCommand = 0
            self.VCLC.Update(self.referenceCommands)
            UAV_state = self.VCLC.VAM.getVehicleState()
            UAV_dot = self.VCLC.VAM.VDM.getVehicleDerivative()
            Payload_aeroModel = self.Payload.Update(UAV_state, UAV_dot)

        elif(self.mode == DeliveryStates.CLIMBING):
            self.referenceCommands.chi = 0
            self.referenceCommands.Va = 20
            self.VCLC.Update(self.referenceCommands)
            UAV_state = self.VCLC.VAM.getVehicleState()
            UAV_dot = self.VCLC.VAM.VDM.getVehicleDerivative()
            Payload_aeroModel = self.Payload.Update(UAV_state, UAV_dot)
            

        elif (self.mode == DeliveryStates.FLYINGTO):
            # in FLYINGTO the payload follows the UAV path, the UAV turns towards the drop point and flies that way
            self.VCLC.Update(self.referenceCommands)
            UAV_state = self.VCLC.VAM.getVehicleState()

            # UAV_state.p=UAV_state.q=UAV_state.r = 0
            # UAV_state.mx=UAV_state.q=UAV_state.r = 0

            UAV_dot = self.VCLC.VAM.VDM.getVehicleDerivative()

            # UAV_dot.yaw=UAV_dot.pitch=UAV_dot.roll = 0
            # UAV_dot.p=UAV_dot.q=UAV_dot.r = 0
            Payload_aeroModel = self.Payload.Update(UAV_state, UAV_dot)
            

        elif (self.mode == DeliveryStates.DROPPING):
            # in DROPPING the payload drops to the ground, the UAV continues flying
            self.VCLC.Update(self.referenceCommands)
            Payload_aeroModel = self.Payload.Update()
            

        elif (self.mode == DeliveryStates.DELIVERED):
            # stops the motion of the payload once delivered, the UAV flies straight ahead
            self.referenceCommands.courseCommand = 0
            self.VCLC.Update(self.referenceCommands)
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
        airspeed =self.target.deliveryAirSpeed
        altitude = self.target.deliveryAltitude
        self.dropDist = self.target.deliveryAirSpeed * math.sqrt(self.target.deliveryAltitude * 2 / (VPC.g0)) - (airspeed**2*PPC.fPa+airspeed*PPC.fPb + altitude**2*PPC.fPc + altitude*PPC.fPd + PPC.fPe)
        
        return self.dropDist

    def CalculateDropPoint(self):
        """
        Returns a 2-tuple of (dropPe, dropPn) computed from self attributes.
        """
        dPn = self.target.pn + self.dropDist * math.cos(self.chi + math.pi)
        dPe = self.target.pe + self.dropDist * math.sin(self.chi + math.pi)
        return dPn, dPe

