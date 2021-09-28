"""
MVP Test.py
The test parameters
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

targetVa = 25
targetAlt = 100

t_steps = np.arange(0,100, VPC.dT)

n_steps = len(t_steps)

flag = 0

while flag != 1:
    northPosition = input("Enter a target Pn value: ")
    eastPosition = input("Enter a target Pe value: ")
    targetVa = int(input("Enter a target airspeed: "))
    targetAlt = int(input("Enter a target altitude: "))
    
    targetPn = int(northPosition)
    targetPe = int(eastPosition)

    if (abs(targetPe) < 100) or (abs(targetPn) < 100):
        print("Invalid target! Target is TOO CLOSE. \nPlease input a Pn that is > 100 or < -100,  and > 100 or < -100")
    elif (targetPe > 5000) or (targetPn > 5000):
        print("Invalid target! Target is TOO FAR. \nPlease input a Pn < 5,000 or > -5,000 and < 5,000 or > -5,000")
    elif (targetVa > 35) or (targetVa < 25):
        print("Invalid target airspeed! \nPlease input a Va > 25 and < 35")
    elif (targetAlt > 200) or (targetAlt < 25):
        print("Invalid target altitude! \nPlease input an altitude > 25  and < 200")    
    else:
        flag = 1


pdc = PDC.PayloadDeliveryControl(States.vehicleState(pn=targetPn,pe=targetPe))


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
    pdc.Update(0)


actualDistanceToTarget = math.sqrt((pdc.target.pn-pdc.Payload.aeroModel.getVehicleState().pn)**2 + (pdc.target.pe-pdc.Payload.aeroModel.getVehicleState().pe)**2)
print("Distance from target: %.3f meters"%(actualDistanceToTarget))

pnPlot = plt.figure()

plt.plot(t_steps,PayloadPnVec)
plt.plot(t_steps,UAVPnVec)
plt.xlabel('Time is seconds')
plt.ylabel('Pn in meters')
plt.title('Pn value of the UAV and the Payload')
plt.legend(["Payload Pn","UAV Pn"])


pePlot = plt.figure()

plt.plot(t_steps,PayloadPeVec)
plt.plot(t_steps,UAVPeVec)
plt.xlabel('Time is seconds')
plt.ylabel('Pe in meters')
plt.title('Pe value of the UAV and the Payload')
plt.legend(["Payload Pe","UAV Pe"])


chiPlot = plt.figure()

plt.plot(t_steps,commandedChi)
plt.plot(t_steps,UAVChi)
plt.xlabel('Time is seconds')
plt.ylabel('Course angle in radians')
plt.title('Course Angle of the UAV versus in realation to the commanded angle')
plt.legend(["Commanded Chi","UAV Chi"])


plot3d = plt.figure()

ax = plt.axes(projection='3d')

# Data for a three-dimensional line

ax.plot3D(PayloadPnVec, PayloadPeVec, -PayloadPdVec, 'blue')
ax.plot3D(UAVPnVec, UAVPeVec, -UAVPdVec, 'orange')
ax.legend(["Payload Path","UAV Path","Target"])
ax.scatter(targetPn,targetPe, 0, c=['red'])
ax.set_xlabel('N')
ax.set_ylabel('E')
ax.set_zlabel('Alt')

plot3d.suptitle('Position of the UAV and Payload', fontsize=20)

plt.show()




