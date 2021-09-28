import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule

"""
Matthew Bennett mabennet@ucsc.edu
VehicleClosedLoopControl.Py
Code to implement the Closed Loop control of the planes flight.
"""

class VehicleClosedLoopControl:
    def __init__(self, dT=0.01, rudderControlSource='SIDESLIP'):
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.controlGains = Controls.controlGains()
        self.mode = Controls.AltitudeStates.HOLDING
        self.dT = dT

        # The two controlInputs
        self.trimControls = Inputs.controlInputs()
        self.outputControls = Inputs.controlInputs()

        # Controllers
        # Pi controllers
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()

        #PD controller
        self.elevatorFromPitch = PDControl()

        #PID controller
        self.aileronFromRoll = PIDControl()
        return

    def setControlGains(self, controlGains=Controls.controlGains()):

        #Pi controllers
        #(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0
        self.rollFromCourse.setPIGains(dT=self.dT,
                                       kp=controlGains.kp_course,
                                       ki=controlGains.ki_course,
                                       trim=0.0,
                                       lowLimit=-math.radians(VPC.bankAngleLimit),
                                       highLimit=math.radians(VPC.bankAngleLimit))

        self.rudderFromSideslip.setPIGains(dT=self.dT,
                                           kp=controlGains.kp_sideslip,
                                           ki=controlGains.ki_sideslip,
                                           trim=self.trimControls.Rudder,
                                           lowLimit=VPC.minControls.Rudder,
                                           highLimit=VPC.maxControls.Rudder)

        self.throttleFromAirspeed.setPIGains(dT=self.dT,
                                             kp=controlGains.kp_SpeedfromThrottle,
                                             ki=controlGains.ki_SpeedfromThrottle,
                                             trim=self.trimControls.Throttle,
                                             lowLimit=VPC.minControls.Throttle,
                                             highLimit=VPC.maxControls.Throttle)

        self.pitchFromAltitude.setPIGains(dT=self.dT,
                                          kp=controlGains.kp_altitude,
                                          ki=controlGains.ki_altitude,
                                          trim=0.0,
                                          lowLimit=-math.radians(VPC.pitchAngleLimit),
                                          highLimit=math.radians(VPC.pitchAngleLimit))

        self.pitchFromAirspeed.setPIGains(dT=self.dT,
                                          kp=controlGains.kp_SpeedfromElevator,
                                          ki=controlGains.ki_SpeedfromElevator,
                                          trim=0.0,
                                          lowLimit=-math.radians(VPC.pitchAngleLimit),
                                          highLimit=math.radians(VPC.pitchAngleLimit))

        self.elevatorFromPitch.setPDGains(kp=controlGains.kp_pitch,
                                            kd=controlGains.kd_pitch,
                                            trim=self.trimControls.Elevator,
                                            lowLimit=VPC.minControls.Elevator,
                                            highLimit=VPC.maxControls.Elevator)

        self.aileronFromRoll.setPIDGains(dT=self.dT,
                                         kp=controlGains.kp_roll,
                                         kd=controlGains.kd_roll,
                                         ki=controlGains.ki_roll,
                                         trim=self.trimControls.Aileron,
                                         lowLimit=VPC.minControls.Aileron,
                                         highLimit=VPC.maxControls.Aileron)

        return

    def reset(self):
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()

        # PID controller
        self.aileronFromRoll.resetIntegrator()

        #reset the aerodynamics model
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        return

    #wrapper update
    def Update(self, referenceCommands=Controls.referenceCommands):
        controlOutputs = self.UpdateControlCommands(referenceCommands, self.VAM.getVehicleState())
        self.trimControls = controlOutputs
        self.VAM.Update(controlOutputs)
        return

    #workhorse update
    def UpdateControlCommands(self, referenceCommands, state):
        commandedAltitude = referenceCommands.commandedAltitude
        upperThresh = commandedAltitude + VPC.altitudeHoldZone
        lowerThresh = commandedAltitude - VPC.altitudeHoldZone

        #state machine

        # determine which state we need to be in based on current state and the altitude
        if self.mode == Controls.AltitudeStates.DESCENDING:
            if lowerThresh < -state.pd < upperThresh:
                self.mode = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()


        if self.mode == Controls.AltitudeStates.HOLDING:
            if -state.pd < lowerThresh:
                self.mode = Controls.AltitudeStates.CLIMBING
                self.pitchFromAirspeed.resetIntegrator()

            if -state.pd > upperThresh:
                self.mode = Controls.AltitudeStates.DESCENDING
                self.pitchFromAirspeed.resetIntegrator()


        if self.mode == Controls.AltitudeStates.CLIMBING:
            if lowerThresh < -state.pd < upperThresh:
                self.mode = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()



        if self.mode == Controls.AltitudeStates.DESCENDING:
            pitchCommand = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
            throttleCommand = VPC.minControls.Throttle

        # with the state chosen now do the calculation
        if self.mode == Controls.AltitudeStates.HOLDING:
            pitchCommand = self.pitchFromAltitude.Update(command=referenceCommands.commandedAltitude, current=-state.pd)
            throttleCommand = self.throttleFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)


        if self.mode == Controls.AltitudeStates.CLIMBING:
            pitchCommand = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
            throttleCommand = VPC.maxControls.Throttle



        # Post state machine: Find the rollCommand
        chiCheck = referenceCommands.commandedCourse - state.chi
        if chiCheck <= -math.pi:
            state.chi = state.chi - 2*math.pi

        elif chiCheck >= math.pi:
            state.chi = state.chi + 2*math.pi

        rollCommand = self.rollFromCourse.Update(command=referenceCommands.commandedCourse, current=state.chi)

        referenceCommands.commandedRoll = rollCommand
        referenceCommands.commandedPitch = pitchCommand

        # Determine the control inputs and load into the return object
        outputControls = Inputs.controlInputs

        #Aileron

        aileronCommand = self.aileronFromRoll.Update(command=rollCommand, current=state.roll, derivative=state.p)
        outputControls.Aileron = aileronCommand

        #Elevator
        elevatorCommand = self.elevatorFromPitch.Update(command=pitchCommand, current=state.pitch, derivative=state.q)
        outputControls.Elevator = elevatorCommand

        #Rudder
        rudderCommand = self.rudderFromSideslip.Update(command=0, current=state.beta)
        outputControls.Rudder = rudderCommand

        #Throttle
        if throttleCommand < VPC.minControls.Throttle:
            throttleCommand = VPC.minControls.Throttle

        if throttleCommand > VPC.maxControls.Throttle:
            throttleCommand = VPC.maxControls.Throttle

        outputControls.Throttle = throttleCommand

        self.outputControls = outputControls
        return outputControls

    # Getters and Setters-------------------------------------------------------------------------------------------------
    def getControlGains(self):
        return self.controlGains

    def getTrimInputs(self):
        return self.trimControls

    def getVehicleAerodynamicsModel(self):
        return self.VAM

    def getVehicleControlSurfaces(self):
        return self.outputControls

    def getVehicleState(self):
        return self.VAM.getVehicleState()


    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.trimControls = trimInputs

    def setVehicleState(self, state):
        self.VAM.setVehicleState(state)



        

"""
 PD PI and PID Loops Classes
"""


class PDControl:
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0,):
        self.kp = kp
        self.kd = kd

        self.trim = trim

        self.lowLimit = lowLimit
        self.highLimit = highLimit



        return


    def Update(self, command=0.0, current=0.0, derivative=0.0):

        error = command - current

        Kp = self.kp
        Kd = self.kd
        dot = derivative

        u = 0
        u = Kp * error - Kd * dot + self.trim


        if u < self.lowLimit:
            u = self.lowLimit

        if u > self.highLimit:
            u = self.highLimit
        return u

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return


# Needed Module level Variables:
# PIoldAccumulator = 0
# PIprevError = 0
class PIControl:
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.ki = ki

        self.kp = kp

        self.trim = trim

        self.lowLimit = lowLimit
        self.highLimit = highLimit

        self.oldError = 0
        self.oldAccumulator = 0
        return

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.ki = ki

        self.kp = kp

        self.trim = trim

        self.lowLimit = lowLimit
        self.highLimit = highLimit


    def Update(self, command=0.0, current=0.0):
        newaccumulator = 0
        Iterm = 0
        u = 0

        newError = command - current

        newAccumulator = self.oldAccumulator + ((self.oldError + newError) * 0.5 * self.dT)
        integralControl = newAccumulator * self.ki

        proportionalControl = self.kp * newError


        u = proportionalControl + integralControl + self.trim

        if u < self.lowLimit:
            u = self.lowLimit

        if u > self.highLimit:
            u = self.highLimit
        else:
            self.oldAccumulator = newAccumulator

        self.oldError = newError
        return u

    def resetIntegrator(self):
        self.oldAccumulator = 0
        return

class PIDControl:
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.ki = ki
        self.kd = kd
        self.kp = kp

        self.trim = trim

        self.lowLimit = lowLimit
        self.highLimit = highLimit

        self.oldError = 0
        self.oldAccumulator = 0
        return

    def Update(self, command=0.0, current=0.0,  derivative=0.0):
        newaccumulator = 0
        Iterm = 0
        u = 0

        newError = command - current

        newAccumulator = self.oldAccumulator + ((self.oldError + newError) * 0.5 * self.dT)
        integralControl = newAccumulator * self.ki

        proportionalControl = self.kp * newError

        derivativeControl = self.kd * derivative

        u = proportionalControl + integralControl - derivativeControl  + self.trim

        if u < self.lowLimit:
            u = self.lowLimit

        if u > self.highLimit:
            u = self.highLimit
        else:
            self.oldAccumulator = newAccumulator

        self.oldError = newError
        return u


    def resetIntegrator(self):
        self.oldAccumulator = 0
        return

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.ki = ki
        self.kd = kd
        self.kp = kp

        self.trim = trim

        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return





