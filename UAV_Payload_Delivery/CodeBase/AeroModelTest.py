"""
Test harness for Lab5.
Currently its just the lab4 test harness that I need to adapt to fit lab 5 but it is 5 am and I am going to sleep
"""
from ece163.Containers.Payload import Payload
import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG
import ece163.Containers.Controls as Controls
import ece163.Modeling.VehicleAerodynamicsModel as VAM
from ece163.Modeling import WindModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Controls import VehicleTrim
from ece163.Controls import VehiclePerturbationModels as Perturb
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Controls import PayloadDeliveryControl as PDC
from ece163.Sensors import SensorsModel
from ece163.Containers import Target
from ece163.Containers import Inputs
from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math
import numpy as np

doWeWantPlots = True

t_steps = np.arange(0,50, VPC.dT)

n_steps = len(t_steps)



targetAirspeeds = np.linspace(25,35, 10)
targetAltitudes = np.linspace(25, 500, 10)
resultingErrors = np.zeros(targetAltitudes.shape)


PayloadPnVec = np.zeros(t_steps.shape)
PayloadPeVec = np.zeros(t_steps.shape)
PayloadPdVec = np.zeros(t_steps.shape)

UAVPnVec = np.zeros(t_steps.shape)
UAVPeVec = np.zeros(t_steps.shape)
UAVPdVec = np.zeros(t_steps.shape)

commandedChi = np.zeros(t_steps.shape)
commandedAltitude = np.zeros(t_steps.shape)
commandedVa = np.zeros(t_steps.shape)

UAVChi = np.zeros(t_steps.shape)
UAVAltitude = np.zeros(t_steps.shape)
UAVVa = np.zeros(t_steps.shape)

PayloadVa = np.zeros(t_steps.shape)

aileronDeflection = np.zeros(t_steps.shape)

changeOfPackageAirspeed = np.zeros(t_steps.shape)

def evaluateError(targetVa,targetAlt):

    pdc = PDC.PayloadDeliveryControl(States.vehicleState(pn=500,pe=500))

    conGains = Controls.controlGains(kp_roll =3.0,
                                        kd_roll = 0.04,
                                        ki_roll = 0.001,
                                        kp_sideslip = 2.0,
                                        ki_sideslip = 2.0,
                                        kp_course = 5.0,
                                        ki_course = 2.0,
                                        kp_pitch = -10.0,
                                        kd_pitch = -0.8,
                                        kp_altitude = 0.08,
                                        ki_altitude = 0.03,
                                        kp_SpeedfromThrottle = 2.0,
                                        ki_SpeedfromThrottle = 1.0,
                                        kp_SpeedfromElevator = -0.5,
                                        ki_SpeedfromElevator = -0.1)

    initState = States.vehicleState(pn=0.0,
                                        pe=0.0,
                                        pd=-100.0,
                                        u=20.0,
                                        v=0.0,
                                        w=0.0,
                                        yaw=math.pi/4,
                                        pitch=0.0,
                                        roll=0.0,
                                        p=0.0,
                                        q=0.0,
                                        r=0.0)
    pdc.VCLC.setVehicleState(initState)
    pdc.VCLC.setControlGains(conGains)
    pdc.target.setDeliveryAirSpeed(targetVa)
    pdc.target.setDeliveryAltitude(targetAlt)
    pdc.Update(1)
    for i in range(n_steps):
        
        payloadState = pdc.Payload.getState()
        UAVState = pdc.VCLC.VAM.getVehicleState()

        PayloadPnVec[i] = payloadState.pn
        PayloadPeVec[i] = payloadState.pe
        PayloadPdVec[i] = payloadState.pd

        UAVPnVec[i] = UAVState.pn
        UAVPeVec[i] = UAVState.pe
        UAVPdVec[i] = UAVState.pd

        commandedChi[i] = pdc.chi
        commandedAltitude[i] = pdc.target.deliveryAltitude
        commandedVa[i] = pdc.target.deliveryAirSpeed

        UAVChi[i] = UAVState.chi
        UAVAltitude[i] = -UAVState.pd
        UAVVa[i] = UAVState.Va
        PayloadVa[i] = payloadState.Va

        aileronDeflection[i] = pdc.VCLC.getVehicleControlSurfaces().Aileron
        pdc.Update(0)

    actualDistanceToTarget = math.sqrt((pdc.target.pn-pdc.Payload.aeroModel.getVehicleState().pn)**2 + (pdc.target.pe-pdc.Payload.aeroModel.getVehicleState().pe)**2)
    return actualDistanceToTarget



pnPlot = plt.figure()

plt.plot(t_steps,PayloadPnVec)
plt.plot(t_steps,UAVPnVec)
plt.legend(["Payload Pn","UAV Pn"])

pePlot = plt.figure()

plt.plot(t_steps,PayloadPeVec)
plt.plot(t_steps,UAVPeVec)
plt.legend(["Payload Pe","UAV Pe"])

pdPlot = plt.figure()

plt.plot(t_steps,PayloadPdVec)
plt.plot(t_steps,UAVPdVec)
plt.legend(["Payload Pd","UAV Pd"])

chiPlot = plt.figure()

plt.plot(t_steps,commandedChi)
plt.plot(t_steps,UAVChi)
plt.legend(["Commanded Chi","UAV Chi"])

altPlot = plt.figure()

plt.plot(t_steps,commandedAltitude)
plt.plot(t_steps,UAVAltitude)
plt.plot(t_steps,-PayloadPdVec)
plt.legend(["Commanded Altitude","UAV Altitude","Payload Altitude"])

VaPlot = plt.figure()

plt.plot(t_steps,commandedVa)
plt.plot(t_steps,UAVVa)
plt.plot(t_steps,PayloadVa)
plt.legend(["Commanded Airspeed","UAV Airspeed","Payload Airspeed"])

AileronPlot = plt.figure()
plt.plot(t_steps,aileronDeflection)
plt.legend(["Aileron Deflection"])

OrthoNvE = plt.figure()
plt.plot(PayloadPnVec,PayloadPeVec)
plt.plot(UAVPnVec,UAVPeVec)
plt.legend(["Payload Position","UAV Position"])


plot3d = plt.figure()

ax = plt.axes(projection='3d')

# Data for a three-dimensional line

ax.plot3D(PayloadPnVec, PayloadPeVec, -PayloadPdVec, 'gray')
ax.plot3D(UAVPnVec, UAVPeVec, -UAVPdVec, 'orange')
ax.set_xlabel('N')
ax.set_ylabel('E')
ax.set_zlabel('Alt')
plt.show()




AltVSsAirspeedVSError = plt.figure()

#tVa, tAlt = np.meshgrid(targetAirspeeds, targetAltitudes)

#tError = evaluateError(tVa,tAlt)
# Data for a three-dimensional line

ax = plt.axes(projection='3d')
#ax.contour3D(tVa, tAlt, tError, 50, cmap='binary')
errors = np.zeros((10,10))

# for i in range(10):
#     for j in range(10):
#         print(i*10 + j)
#         error = evaluateError(targetAirspeeds[i],targetAltitudes[j])
#         errors[i][j] = error
#         ax.scatter(targetAirspeeds[i],targetAltitudes[j],error)
# ax.set_xlabel('Airspeed')
# ax.set_ylabel('Altitude')
# ax.set_zlabel('Error')
# np.save("ErrorsArray",errors)
# np.save("targetAirspeeds",targetAirspeeds)
# np.save("targetAltitudes",targetAltitudes)
# plt.show()

print(evaluateError(30,100))