"""
Test harness for Lab5.
Currently its just the lab4 test harness that I need to adapt to fit lab 5 but it is 5 am and I am going to sleep
"""
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

from ece163.Sensors import SensorsModel

from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math
from ece163.Constants import VehiclePhysicalConstants as VPC
import numpy as np



# Test 1 Descending case -------------------------------------------------------------------------------------------------

# Create a VehicleClosedLoopControl object to serve as the basis for our test.

testVcontrol = VCLC.VehicleClosedLoopControl()
#set starting state to holding
testVcontrol.mode = Controls.AltitudeStates.HOLDING

# Create and set a controlGains object with some decent values
testConGains = Controls.controlGains(kp_roll =3.0,
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
testVcontrol.setControlGains(testConGains)


# It will need a state which we can instatiate with a standard starting states
# Start at N = 0, E = 0, D = -100
#Start with now speed in anyy direction not pitch yaw or roll and no change to them

testInitState = States.vehicleState(pn=0.0,
                                    pe=0.0,
                                    pd=-100.0,
                                    u=20.0,
                                    v=0.0,
                                    w=0.0,
                                    yaw=0.0,
                                    pitch=0.0,
                                    roll=0.0,
                                    p=0.0,
                                    q=0.0,
                                    r=0.0)

testVcontrol.setVehicleState(testInitState)


testTrim = VehicleTrim.VehicleTrim()

#compute a trim to use
testTrim.computeTrim(Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0)


# For lab5 testing we will also need a sensors model that will take the states and dots of the system

testSensors = SensorsModel



# finally lets make a set of reference commands to pass in.

# Test 1

# In this case we'll use a system that moves it from HOLDING to DESCENDING
#courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed

# Test 1 reference commands will descend from Starting Altitude = 100 to Finishing Altitude =  50
testRefValues = Controls.referenceCommands(courseCommand=VPC.InitialYawAngle,
                                           altitudeCommand=50,
                                           airspeedCommand=VPC.InitialSpeed)

timeSim = 2000 # Run for 10000 update calls
samplePeriod = 20 # Take a sample every 100 values.

altitudes = np.zeros(101)
pitch = np.zeros(101)


holdState = testVcontrol.getVehicleState()
j = 0
altitudes[j] = -holdState.pd
pitch[j] = holdState.pitch

for i in range(timeSim):
    testVcontrol.Update(testRefValues)
    if (i%samplePeriod) == 0:
        j = j+1
        holdState = testVcontrol.getVehicleState()
        altitudes[j] = -holdState.pd
        pitch[j] = holdState.pitch


figTest1altitude = plt.figure()
plt.plot(altitudes)
plt.xlabel("Time Sample Period = 20")
plt.ylabel("Altitude")

plt.title("Test 1 Descending Case Altitude")

figTest1pitch = plt.figure()
plt.plot(pitch)
plt.xlabel("Time Sample Period = 20")
plt.ylabel("Pitch")

plt.title("Test 1 Descending Case Pitch")




# Test 2  Ascending test-------------------------------------------------------------------------------------------------

# Create a VCLC object to serve as the basis for our test.
test2Vcontrol = VCLC.VehicleClosedLoopControl()
#set starting state to holding
test2Vcontrol.mode = Controls.AltitudeStates.HOLDING

# Create and set a controlGains object with some decent values
test2ConGains = Controls.controlGains(kp_roll =3.0,
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
test2Vcontrol.setControlGains(test2ConGains)


# It will need a state which we can instatiate with a standard starting states
# Start at N = 0, E = 0, D = -100
#Start with now speed in anyy direction not pitch yaw or roll and no change to them

test2InitState = States.vehicleState(pn=0.0,
                                    pe=0.0,
                                    pd=-100.0,
                                    u=20.0,
                                    v=0.0,
                                    w=20.0,
                                    yaw=0.0,
                                    pitch=0.0,
                                    roll=0.0,
                                    p=0.0,
                                    q=0.0,
                                    r=0.0)

test2Vcontrol.setVehicleState(test2InitState)


test2Trim = VehicleTrim.VehicleTrim()

#compute a trim to use
test2Trim.computeTrim(Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0)

# finally lets make a set of reference commands to pass in.

# Test 2

# In this case we'll use a system that moves it from HOLDING to DESCENDING
#courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed

# Test 2 reference commands will descend from Starting Altitude = 100 to Finishing Altitude =  150
test2RefValues = Controls.referenceCommands(courseCommand=VPC.InitialYawAngle,
                                           altitudeCommand=150,
                                           airspeedCommand=VPC.InitialSpeed)

timeSim = 2000 # Run for 10000 update calls
samplePeriod = 20 # Take a sample every 100 values.

altitudes2 = np.zeros(101)
pitch2 = np.zeros(101)

holdState = test2Vcontrol.getVehicleState()
j = 0
altitudes2[j] = -holdState.pd
pitch2[j] = holdState.pitch
for i in range(timeSim):
    test2Vcontrol.Update(test2RefValues)
    if (i%samplePeriod) == 0:
        j = j+1
        holdState = test2Vcontrol.getVehicleState()
        altitudes2[j] = -holdState.pd
        pitch2[j] = holdState.pitch

figTest1 = plt.figure()
plt.plot(altitudes2)
plt.xlabel("Time Sample Period = 20")
plt.ylabel("Altitude")
plt.ylim(50, 175)

plt.title("Test 2 Ascending Case Altitude")

figTest1pitch = plt.figure()
plt.plot(pitch2)
plt.xlabel("Time Sample Period = 20")
plt.ylabel("Pitch")

plt.title("Test 2 Ascending Case Pitch")

# Test 3 Positive Course Change -------------------------------------------------------------------------------------------------

# Create a VCLC object to serve as the basis for our test.
test3Vcontrol = VCLC.VehicleClosedLoopControl()
#set starting state to holding
test3Vcontrol.mode = Controls.AltitudeStates.HOLDING

# Create and set a controlGains object with some decent values
test3ConGains = Controls.controlGains(kp_roll =3.0,
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
test3Vcontrol.setControlGains(test3ConGains)


# It will need a state which we can instatiate with a standard starting states
# Start at N = 0, E = 0, D = -100
#Start with now speed in anyy direction not pitch yaw or roll and no change to them

test3InitState = States.vehicleState(pn=0.0,
                                    pe=0.0,
                                    pd=-100.0,
                                    u=20.0,
                                    v=0.0,
                                    w=20.0,
                                    yaw=0.0,
                                    pitch=0.0,
                                    roll=0.0,
                                    p=0.0,
                                    q=0.0,
                                    r=0.0)

test3Vcontrol.setVehicleState(test3InitState)


test3Trim = VehicleTrim.VehicleTrim()

#compute a trim to use
test3Trim.computeTrim(Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0)

# finally lets make a set of reference commands to pass in.

# Test 3

# In this case we'll use a system that will do a 90 degree turn
#courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed

# Test 1 reference commands will descend from 0 to a course of 90
test3RefValues = Controls.referenceCommands(courseCommand=math.radians(90.0),
                                           altitudeCommand=100,
                                           airspeedCommand=VPC.InitialSpeed)

timeSim = 2000 # Run for 10000 update calls
samplePeriod = 20 # Take a sample every 20 values.

yaw = np.zeros(101)
roll = np.zeros(101)

holdState = test3Vcontrol.getVehicleState()
j = 0
yaw[j] = holdState.yaw
roll[j] = holdState.roll
for i in range(timeSim):
    test3Vcontrol.Update(test3RefValues)
    if (i%samplePeriod) == 0:
        j = j+1
        holdState = test3Vcontrol.getVehicleState()
        yaw[j] = math.degrees(holdState.yaw)
        roll[j] = math.degrees(holdState.roll)


figTest3yaw = plt.figure()
plt.plot(yaw)
plt.xlabel("Time Sample Period = 20" )
plt.ylabel("Yaw angle degrees")
plt.ylim(0, 120)

plt.title("Test 3 Positive Turn Case Yaw Value")

figTest3Roll = plt.figure()
plt.plot(roll)
plt.xlabel("Time Sample Period = 20" )
plt.ylabel("Roll angle degrees")

plt.title("Test 3 Positive Turn Case Roll Value")




# Test 4 Negative Course Change -------------------------------------------------------------------------------------------------

# Create a VCLC object to serve as the basis for our test.
test4Vcontrol = VCLC.VehicleClosedLoopControl()
#set starting state to holding
test4Vcontrol.mode = Controls.AltitudeStates.HOLDING

# Create and set a controlGains object with some decent values
test4ConGains = Controls.controlGains(kp_roll =3.0,
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
test4Vcontrol.setControlGains(test4ConGains)


# It will need a state which we can instatiate with a standard starting states
# Start at N = 0, E = 0, D = -100
#Start with no speed in u v and w direction no pitch yaw or roll and no change to them at start

test4InitState = States.vehicleState(pn=0.0,
                                    pe=0.0,
                                    pd=-100.0,
                                    u=0.0,
                                    v=0.0,
                                    w=0.0,
                                    yaw=0.0,
                                    pitch=0.0,
                                    roll=0.0,
                                    p=0.0,
                                    q=0.0,
                                    r=0.0)

test4Vcontrol.setVehicleState(test4InitState)


test4Trim = VehicleTrim.VehicleTrim()

#compute a trim to use
test4Trim.computeTrim(Vastar=VPC.InitialSpeed, Kappastar=0.0, Gammastar=0.0)

# finally lets make a set of reference commands to pass in.

# Test 4

# In this case we'll use a system that will do a -90 degree turn
#courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed

# Test 1 reference commands will descend from Starting Altitude = 100 to Finishing Altitude =  50
test4RefValues = Controls.referenceCommands(courseCommand=-math.radians(90.0),
                                           altitudeCommand=100,
                                           airspeedCommand=VPC.InitialSpeed)

timeSim = 2000 # Run for 10000 update calls
samplePeriod = 20 # Take a sample every 20 values.

yaw4 = np.zeros(101)
roll4 = np.zeros(101)

holdState = test4Vcontrol.getVehicleState()
j = 0
yaw4[j] = holdState.yaw
roll4[j] = holdState.roll
for i in range(timeSim):
    test4Vcontrol.Update(test4RefValues)
    if (i%samplePeriod) == 0:
        j = j+1
        holdState = test4Vcontrol.getVehicleState()
        yaw4[j] = math.degrees(holdState.yaw)
        roll4[j] = math.degrees(holdState.roll)


figTest4yaw = plt.figure()
plt.plot(yaw4)
plt.xlabel("Time Sample Period = 20" )
plt.ylabel("Yaw angle degrees")
plt.title("Test 4 Negative Turn Case Yaw Value")

figTest4Roll = plt.figure()
plt.plot(roll4)
plt.xlabel("Time Sample Period = 20" )
plt.ylabel("Roll angle degrees")

plt.title("Test 4 Negative Turn Case Roll Value")

plt.show()








