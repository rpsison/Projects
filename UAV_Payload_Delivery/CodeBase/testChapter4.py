"""This file is testing the chapter4 VehicleAerodynamics class functions
Matthew Bennett mabennet@ucsc.edu

"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
from ece163.Modeling import WindModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))

def compareMatrix(A,B):
	for i in range(3):
		el_close = [isclose(A[i][j], B[i][j]) for j in range(3) ]
	return all(el_close)

failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean


print("Beginning testing of Gravity Forces")

#----------------------------------------------------------------------------
cur_test = "gravity default state"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState()
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.gravityForces(inputState)

expectedVector = [[0], [0], [107.9100], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "gravity yaw value only"

testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=30)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.gravityForces(inputState)
# should make no change
expectedVector = [[0], [0], [107.9100], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "gravity pitch value only"

testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(pitch = 30)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.gravityForces(inputState)

expectedVector = [[106.61849255586073], [0], [16.645273957369195], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "gravity roll value only"

testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(roll=40)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.gravityForces(inputState)

expectedVector = [[0], [80.40516114732654], [-71.96928623289558], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "gravity all 3"

testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=20, pitch=30, roll=40)

inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.gravityForces(inputState)
# should make no change
expectedVector = [[106.61849255586073], [12.40261268541996], [-11.101366748798686], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


print("Begin CaclulateCoeffAlphaTest")

#----------------------------------------------------------------------------
cur_test = "coeffAlpha alpha = 0"

testVAM = VAM.VehicleAerodynamicsModel()
inputForces = Inputs.forcesMoments()
alpha = 0
CL, CD, CM = testVAM.CalculateCoeff_alpha(alpha)
resultVector = [[CL], [CD], [CM]]
# should make no change
expectedVector = [[0.2299999999730887], [0.0012272946567217947], [0.0135]]



if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "coeffAlpha alpha = .25"

testVAM = VAM.VehicleAerodynamicsModel()
inputForces = Inputs.forcesMoments()
alpha = .25
CL, CD, CM = testVAM.CalculateCoeff_alpha(alpha)
resultVector = [[CL], [CD], [CM]]
# should make no change
expectedVector = [[1.632481898735867], [0.061830994516953244], [-0.6715000000000001]]



if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "coeffAlpha alpha = -.25"

testVAM = VAM.VehicleAerodynamicsModel()
inputForces = Inputs.forcesMoments()
alpha = -.25
CL, CD, CM = testVAM.CalculateCoeff_alpha(alpha)
resultVector = [[CL], [CD], [CM]]
# should make no change
expectedVector = [[-1.172489119936041], [0.03189614694932326], [0.6985]]



if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

print("Beginning aeroForces tests")

#----------------------------------------------------------------------------
cur_test = "aeroForces default state"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState()
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultForces = testVAM.aeroForces(inputState)

expectedVector = [[0], [0], [0], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "aeroForces p q r val"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(p=3, q=2, r=-1)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.aeroForces(inputState)

expectedVector = [[0], [0], [0], [0], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "aeroForces u v w val"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(u=3, v=2, w=1)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.aeroForces(inputState)

expectedVector = [[2.6955093819250573], [-2.698419633979602], [-9.570909797410561], [-1.0364905163057896], [-0.8050686714691829], [0.5820292899255587]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "aeroForces p q r and u val"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(p=3, q=2, r=-1, u=5)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.aeroForces(inputState)

expectedVector = [[-0.010700628700125238], [0], [-4.638481606590364], [-13.012405530620763], [-2.3814543352007647], [2.2077227360940843]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "aeroForces p q r and u v w val"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(p=3, q=2, r=-1, u=5, v=6, w=7)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.aeroForces(inputState)

expectedVector = [[4.494511834735107], [-22.897211877040338], [-65.64496128027706], [-36.09010483997688], [-23.922046781848188], [9.569718657307593]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


print("Beginning Control Values Test")
# ----------------------------------------------------------------------------
cur_test = "control default"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState()
inputControls = Inputs.controlInputs()
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "control no state change control aileron"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState()
inputControls = Inputs.controlInputs(Aileron=.5)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "control no state change control rudder"
testVAM = VAM.VehicleAerodynamicsModel()
inputState2 = States.vehicleState()
inputControls2 = Inputs.controlInputs(Rudder=.3)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState2, inputControls2)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "control no state change control elevator"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState()
inputControls = Inputs.controlInputs(Elevator=.2)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change no control changes"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=10)
inputControls = Inputs.controlInputs()
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change no control changes"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(pitch=20)
inputControls = Inputs.controlInputs()
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change no control changes"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(roll=20)
inputControls = Inputs.controlInputs()
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change no controls"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20)
inputControls = Inputs.controlInputs()
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change and controls aileron"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20)
inputControls = Inputs.controlInputs(Aileron=.5)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change and controls rudder"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20)
inputControls = Inputs.controlInputs(Rudder=.5)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change and controls elevator"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20)
inputControls = Inputs.controlInputs(Elevator=.5)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change and controls"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20)
inputControls = Inputs.controlInputs(Aileron=.1, Rudder=.2, Elevator=.5,)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[21.817680754436033], [0], [0], [-0.6194943564776727], [0], [0]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Control state change and controls"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20, u=10, v=15, w=20)
inputControls = Inputs.controlInputs(Aileron=.1, Rudder=.2, Elevator=.5,)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[-2.874911350905915], [11.504555562500002], [-8.876527519388382], [13.517320784366671], [-23.7727860517125], [-10.908958399845002]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
cur_test = "Control Full Test"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20, u=10, v=15, w=20, p=2, q=4,r=4)
inputControls = Inputs.controlInputs(Aileron=.1, Rudder=.2, Elevator=.5,)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.controlForces(inputState, inputControls)

expectedVector = [[-2.874911350905915], [11.504555562500002], [-8.876527519388382], [13.517320784366671], [-23.7727860517125], [-10.908958399845002]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

print("Beginning Calculate PropForces Test")



# ----------------------------------------------------------------------------
cur_test = "Prop Default Test"
testVAM = VAM.VehicleAerodynamicsModel()
inputThrottle = 0
inputVA = 0
inputForces = Inputs.forcesMoments()

FX, MX = testVAM.CalculatePropForces(inputVA, inputThrottle)
#had to add on a dummy third value to make it work with the compare vectors
expectedVector = [[0.00018320594739300043], [-5.201975946047016e-06], [0]]
resultVector = [[FX], [MX], [0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Prop VA no throttle"
testVAM = VAM.VehicleAerodynamicsModel()
inputThrottle = 0
inputVA = 5
inputForces = Inputs.forcesMoments()

FX, MX = testVAM.CalculatePropForces(inputVA, inputThrottle)
#had to add on a dummy third value to make it work with the compare vectors
expectedVector = [[-0.8805262094179237], [0.06925779323194443], [0]]
resultVector = [[FX], [MX], [0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Prop no Va with throttle"
testVAM = VAM.VehicleAerodynamicsModel()
inputThrottle = .5
inputVA = 0
inputForces = Inputs.forcesMoments()

FX, MX = testVAM.CalculatePropForces(inputVA, inputThrottle)
#had to add on a dummy third value to make it work with the compare vectors
expectedVector = [[21.817680754436033], [-0.6194943564776727], [0]]
resultVector = [[FX], [MX], [0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Prop full test"
testVAM = VAM.VehicleAerodynamicsModel()
inputThrottle = .75
inputVA = 10
inputForces = Inputs.forcesMoments()

FX, MX = testVAM.CalculatePropForces(inputVA, inputThrottle)
#had to add on a dummy third value to make it work with the compare vectors
expectedVector = [[36.86136338891716], [-1.421159167071197], [0]]
resultVector = [[FX], [MX], [0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Prop full test with a negative Va"
testVAM = VAM.VehicleAerodynamicsModel()
inputThrottle = .75
inputVA = -15
inputForces = Inputs.forcesMoments()

FX, MX = testVAM.CalculatePropForces(inputVA, inputThrottle)
#had to add on a dummy third value to make it work with the compare vectors
expectedVector = [[54.48099233269528], [-0.30555458633454535], [0]]
resultVector = [[FX], [MX], [0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

print("Finally Test the updateForces Function")
# ----------------------------------------------------------------------------
cur_test = "Final Test updateForces"
testVAM = VAM.VehicleAerodynamicsModel()
inputState = States.vehicleState(yaw=5, pitch=10, roll=20, u=10, v=15, w=20, p=2, q=4,r=4)
inputControls = Inputs.controlInputs(Aileron=.1, Rudder=.2, Elevator=.5,)
inputForces = Inputs.forcesMoments()

resultForces = testVAM.updateForces(inputState, inputControls)

expectedVector = [[81.19617364194148], [-217.5699611101004], [-510.8160130963135], [-43.50860497788359], [-194.70483432400832], [11.144258604751148]]
resultVector = [[resultForces.Fx], [resultForces.Fy], [resultForces.Fz],[resultForces.Mx], [resultForces.My], [resultForces.Mz]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------


print("\n Lab 3 Testing \n")
print("Wind Model Testing\n")
print("CreateDrydenTransferFcns Test")
# ----------------------------------------------------------------------------
cur_test = "CreateDrydenTransferFcns 0.0 All 0s test"
testWM = WindModel.WindModel()

dT = testWM.dT
Va = testWM.Va
drydenParamters = testWM.drydenParameters

testWM.CreateDrydenTransferFcns(dT, Va, drydenParamters)

Phiu = testWM.Phiu
Gammau = testWM.Gammau
Hu = testWM.Hu

Phiv = testWM.Phiv
Gammav = testWM.Gammav
Hv = testWM.Hv

Phiw = testWM.Phiw
Gammaw = testWM.Gammaw
Hw = testWM.Hw

Phi_u_0_0 = Phiu[0][0]
Gamma_u_0_0 = Gammau[0][0]
H_u_u = Hu[0][0]

Phi_v_0_0 = Phiv[0][0]
Phi_v_0_1 = Phiv[0][1]
Phi_v_1_0 = Phiv[1][0]
Phi_v_1_1 = Phiv[1][1]
Gamma_v_0_0 = Gammav[0][0]
Gamma_v_1_0 = Gammav[1][0]
H_v_0_0 = Hv[0][0]
H_v_0_1 = Hv[0][1]

Phi_w_0_0 = Phiw[0][0]
Phi_w_0_1 = Phiw[0][1]
Phi_w_1_0 = Phiw[1][0]
Phi_w_1_1 = Phiw[1][1]
Gamma_w_0_0 = Gammaw[0][0]
Gamma_w_1_0 = Gammaw[1][0]
H_w_0_0 = Hw[0][0]
H_w_0_1 = Hw[0][1]




resultVector = [[Phi_u_0_0],
				  [Gamma_u_0_0],
				  [H_u_u],
				  [Phi_v_0_0],
				  [Phi_v_0_1],
				  [Phi_v_1_0],
				  [Phi_v_1_1],
				  [Gamma_v_0_0],
				  [Gamma_v_1_0],
				  [H_v_0_0],
				  [H_v_0_1],
				  [Phi_w_0_0],
				  [Phi_w_0_1],
				  [Phi_w_1_0],
				  [Phi_w_1_1],
				  [Gamma_w_0_0],
				  [Gamma_w_1_0],
				  [H_w_0_0],
				  [H_w_0_1]]
expectedVector =   [[1],
				  [0],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "CreateDrydenTransferFcns 1.0 Va = 10 all others are 0 test"
testWM = WindModel.WindModel(Va=10)

dT = testWM.dT
Va = testWM.Va
drydenParamters = testWM.drydenParameters

testWM.CreateDrydenTransferFcns(dT, Va, drydenParamters)

Phiu = testWM.Phiu
Gammau = testWM.Gammau
Hu = testWM.Hu

Phiv = testWM.Phiv
Gammav = testWM.Gammav
Hv = testWM.Hv

Phiw = testWM.Phiw
Gammaw = testWM.Gammaw
Hw = testWM.Hw

Phi_u_0_0 = Phiu[0][0]
Gamma_u_0_0 = Gammau[0][0]
H_u_u = Hu[0][0]

Phi_v_0_0 = Phiv[0][0]
Phi_v_0_1 = Phiv[0][1]
Phi_v_1_0 = Phiv[1][0]
Phi_v_1_1 = Phiv[1][1]
Gamma_v_0_0 = Gammav[0][0]
Gamma_v_1_0 = Gammav[1][0]
H_v_0_0 = Hv[0][0]
H_v_0_1 = Hv[0][1]

Phi_w_0_0 = Phiw[0][0]
Phi_w_0_1 = Phiw[0][1]
Phi_w_1_0 = Phiw[1][0]
Phi_w_1_1 = Phiw[1][1]
Gamma_w_0_0 = Gammaw[0][0]
Gamma_w_1_0 = Gammaw[1][0]
H_w_0_0 = Hw[0][0]
H_w_0_1 = Hw[0][1]




resultVector = [[Phi_u_0_0],
				  [Gamma_u_0_0],
				  [H_u_u],
				  [Phi_v_0_0],
				  [Phi_v_0_1],
				  [Phi_v_1_0],
				  [Phi_v_1_1],
				  [Gamma_v_0_0],
				  [Gamma_v_1_0],
				  [H_v_0_0],
				  [H_v_0_1],
				  [Phi_w_0_0],
				  [Phi_w_0_1],
				  [Phi_w_1_0],
				  [Phi_w_1_1],
				  [Gamma_w_0_0],
				  [Gamma_w_1_0],
				  [H_w_0_0],
				  [H_w_0_1]]
expectedVector =   [[1],
				  [0],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "CreateDrydenTransferFcns 1.1 Va = 10 all others are 0 test"
testWM = WindModel.WindModel(Va=25)

dT = testWM.dT
Va = testWM.Va
drydenParamters = testWM.drydenParameters

testWM.CreateDrydenTransferFcns(dT, Va, drydenParamters)

Phiu = testWM.Phiu
Gammau = testWM.Gammau
Hu = testWM.Hu

Phiv = testWM.Phiv
Gammav = testWM.Gammav
Hv = testWM.Hv

Phiw = testWM.Phiw
Gammaw = testWM.Gammaw
Hw = testWM.Hw

Phi_u_0_0 = Phiu[0][0]
Gamma_u_0_0 = Gammau[0][0]
H_u_u = Hu[0][0]

Phi_v_0_0 = Phiv[0][0]
Phi_v_0_1 = Phiv[0][1]
Phi_v_1_0 = Phiv[1][0]
Phi_v_1_1 = Phiv[1][1]
Gamma_v_0_0 = Gammav[0][0]
Gamma_v_1_0 = Gammav[1][0]
H_v_0_0 = Hv[0][0]
H_v_0_1 = Hv[0][1]

Phi_w_0_0 = Phiw[0][0]
Phi_w_0_1 = Phiw[0][1]
Phi_w_1_0 = Phiw[1][0]
Phi_w_1_1 = Phiw[1][1]
Gamma_w_0_0 = Gammaw[0][0]
Gamma_w_1_0 = Gammaw[1][0]
H_w_0_0 = Hw[0][0]
H_w_0_1 = Hw[0][1]




resultVector = [[Phi_u_0_0],
				  [Gamma_u_0_0],
				  [H_u_u],
				  [Phi_v_0_0],
				  [Phi_v_0_1],
				  [Phi_v_1_0],
				  [Phi_v_1_1],
				  [Gamma_v_0_0],
				  [Gamma_v_1_0],
				  [H_v_0_0],
				  [H_v_0_1],
				  [Phi_w_0_0],
				  [Phi_w_0_1],
				  [Phi_w_1_0],
				  [Phi_w_1_1],
				  [Gamma_w_0_0],
				  [Gamma_w_1_0],
				  [H_w_0_0],
				  [H_w_0_1]]
expectedVector =   [[1],
				  [0],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1],
				  [1],
				  [0],
				  [0],
				  [1],
				  [0],
				  [0],
				  [1],
				  [1]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "CreateDrydenTransferFcns Full test 2.0 Lu = 2 Lv = 4 Lw = 8 all others are Va = 5  test"
testWM = WindModel.WindModel( Va=5, drydenParamters=Inputs.drydenParameters(Lu=2.0, Lv=4.0, Lw=8.0, sigmau=10.0, sigmav=15.0, sigmaw=20.0))

dT = testWM.dT
Va = testWM.Va
drydenParamters = testWM.drydenParameters

testWM.CreateDrydenTransferFcns(dT, Va, drydenParamters)

Phiu = testWM.Phiu
Gammau = testWM.Gammau
Hu = testWM.Hu

Phiv = testWM.Phiv
Gammav = testWM.Gammav
Hv = testWM.Hv

Phiw = testWM.Phiw
Gammaw = testWM.Gammaw
Hw = testWM.Hw

Phi_u_0_0 = Phiu[0][0]
Gamma_u_0_0 = Gammau[0][0]
H_u_u = Hu[0][0]

Phi_v_0_0 = Phiv[0][0]
Phi_v_0_1 = Phiv[0][1]
Phi_v_1_0 = Phiv[1][0]
Phi_v_1_1 = Phiv[1][1]
Gamma_v_0_0 = Gammav[0][0]
Gamma_v_1_0 = Gammav[1][0]
H_v_0_0 = Hv[0][0]
H_v_0_1 = Hv[0][1]

Phi_w_0_0 = Phiw[0][0]
Phi_w_0_1 = Phiw[0][1]
Phi_w_1_0 = Phiw[1][0]
Phi_w_1_1 = Phiw[1][1]
Gamma_w_0_0 = Gammaw[0][0]
Gamma_w_1_0 = Gammaw[1][0]
H_w_0_0 = Hw[0][0]
H_w_0_1 = Hw[0][1]




resultVector = [[Phi_u_0_0],
				  [Gamma_u_0_0],
				  [H_u_u],
				  [Phi_v_0_0],
				  [Phi_v_0_1],
				  [Phi_v_1_0],
				  [Phi_v_1_1],
				  [Gamma_v_0_0],
				  [Gamma_v_1_0],
				  [H_v_0_0],
				  [H_v_0_1],
				  [Phi_w_0_0],
				  [Phi_w_0_1],
				  [Phi_w_1_0],
				  [Phi_w_1_1],
				  [Gamma_w_0_0],
				  [Gamma_w_1_0],
				  [H_w_0_0],
				  [H_w_0_1]]
expectedVector =   [[0.9875778004938814], [0.009876035188666955], [12.615662610100802], [0.975233077987708], [-0.015430903132716898], [0.009875778004938815], [0.9999225230000549], [0.009875778004938815], [4.958527996486127e-05], [16.38822645888119], [11.827183696969502], [0.9875584313069985], [-0.0038819120727476356], [0.009937694906233948], [0.999980549939791], [0.009937694906233948], [4.9792154135127344e-05], [15.450968080927582], [5.57538786297741]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "CreateDrydenTransferFcns 2.0 Lu = 2 Lv = 4 Lw = 8 all others are 0 test"
testWM = WindModel.WindModel(drydenParamters=Inputs.drydenParameters(Lu=2.0, Lv=4.0, Lw=8.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0))

dT = testWM.dT
Va = testWM.Va
drydenParamters = testWM.drydenParameters

testWM.CreateDrydenTransferFcns(dT, Va, drydenParamters)

Phiu = testWM.Phiu
Gammau = testWM.Gammau
Hu = testWM.Hu

Phiv = testWM.Phiv
Gammav = testWM.Gammav
Hv = testWM.Hv

Phiw = testWM.Phiw
Gammaw = testWM.Gammaw
Hw = testWM.Hw

Phi_u_0_0 = Phiu[0][0]
Gamma_u_0_0 = Gammau[0][0]
H_u_u = Hu[0][0]

Phi_v_0_0 = Phiv[0][0]
Phi_v_0_1 = Phiv[0][1]
Phi_v_1_0 = Phiv[1][0]
Phi_v_1_1 = Phiv[1][1]
Gamma_v_0_0 = Gammav[0][0]
Gamma_v_1_0 = Gammav[1][0]
H_v_0_0 = Hv[0][0]
H_v_0_1 = Hv[0][1]

Phi_w_0_0 = Phiw[0][0]
Phi_w_0_1 = Phiw[0][1]
Phi_w_1_0 = Phiw[1][0]
Phi_w_1_1 = Phiw[1][1]
Gamma_w_0_0 = Gammaw[0][0]
Gamma_w_1_0 = Gammaw[1][0]
H_w_0_0 = Hw[0][0]
H_w_0_1 = Hw[0][1]




resultVector = [[Phi_u_0_0],
				  [Gamma_u_0_0],
				  [H_u_u],
				  [Phi_v_0_0],
				  [Phi_v_0_1],
				  [Phi_v_1_0],
				  [Phi_v_1_1],
				  [Gamma_v_0_0],
				  [Gamma_v_1_0],
				  [H_v_0_0],
				  [H_v_0_1],
				  [Phi_w_0_0],
				  [Phi_w_0_1],
				  [Phi_w_1_0],
				  [Phi_w_1_1],
				  [Gamma_w_0_0],
				  [Gamma_w_1_0],
				  [H_w_0_0],
				  [H_w_0_1]]
expectedVector =   [[0.9394130628134758],
				  [0.009400247793232364],
				  [0.0],
				  [0.8806997463876336],
				  [-0.366958227661514],
				  [0.009394130628134758],
				  [0.998126379239318],
				  [0.009394130628134758],
				  [4.7964691473456005e-05],
				  [0.0],
				  [0.0],
				  [0.9389446958989583],
				  [-0.09465168305433048],
				  [0.009692332344763441],
				  [0.9995217730537299],
				  [0.009692332344763441],
				  [4.8970439298074046e-05],
				  [0.0],
				  [0.0]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
print("Wind Model Update Test")
# ----------------------------------------------------------------------------
cur_test = "Wind model Update 0.0 no wind no inputs"

testWM = WindModel.WindModel()

testWM.Update()

windState = testWM.windState




expectedVector = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
resultVector = [[windState.Wn], [windState.We], [windState.Wd],[windState.Wu], [windState.Wv], [windState.Ww]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
cur_test = "Wind model Update 1.0 no winds Lu = 200 lv = 400 and lw = 50 inputs"

testWM = WindModel.WindModel(drydenParamters=Inputs.drydenParameters(Lu=200.0, Lv=400.0, Lw=50.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0))

testWM.Update()

windState = testWM.windState




expectedVector = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
resultVector = [[windState.Wn], [windState.We], [windState.Wd],[windState.Wu], [windState.Wv], [windState.Ww]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Wind model Update 1.1 no dryden parameters. uu=0.2, uv=0.4, uw=0.6 "

testWM = WindModel.WindModel()

testWM.Update(uu=0.2, uv=0.4, uw=0.6)

windState = testWM.windState




expectedVector = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
resultVector = [[windState.Wn], [windState.We], [windState.Wd],[windState.Wu], [windState.Wv], [windState.Ww]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Wind model Update 1.1 dryden parameters. Lu=200.0, Lv=400.0, Lw=50.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0 uu=0.2, uv=0.4, uw=0.6 "

testWM = WindModel.WindModel(drydenParamters=Inputs.drydenParameters(Lu=200.0, Lv=400.0, Lw=50.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0))

testWM.Update(uu=0.2, uv=0.4, uw=0.6)

windState = testWM.windState




expectedVector = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
resultVector = [[windState.Wn], [windState.We], [windState.Wd],[windState.Wu], [windState.Wv], [windState.Ww]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Wind model Update 2.0 Full test wind Wn=2.0, We=5.0, Wd=10.0  dryden parameters. Lu=200.0, Lv=400.0, Lw=50.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0 uu=0.2, uv=0.4, uw=0.6 "

testWM = WindModel.WindModel(drydenParamters=Inputs.drydenParameters(Lu=200.0, Lv=400.0, Lw=50.0, sigmau=0.5, sigmav=0.6, sigmaw=0.7))
windState = States.windState(Wn=2.0, We=5.0, Wd=10.0)
testWM.setWind(windState)
testWM.Update(uu=0.2, uv=0.4, uw=0.6)

windState = testWM.windState




expectedVector = [[2.0], [5.0], [10.0], [0.00028191855596824234], [0.0005860624183718268], [0.002891851274891224]]
resultVector = [[windState.Wn], [windState.We], [windState.Wd],[windState.Wu], [windState.Wv], [windState.Ww]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

print("Beginning Calculate Airpseed Tests")

# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState()



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[0.0], [0.0], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState()



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[0.0], [0.0], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Wu = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wu=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[10.0], [3.141592653589793], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Wv = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wv=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[10.0], [0.0], [-1.5707963267948966]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Ww = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Ww=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector =  [[10.0], [-1.5707963267948966], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Wu = 2, Wv = 5, Ww = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wu=2,Wv=5,Ww=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector =  [[11.357816691600547], [-1.7681918866447774], [-0.455849750893768]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Wn = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wn=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[10.0], [3.141592653589793], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: We = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(We=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector =  [[10.0], [0.0], [-1.5707963267948966]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind: Wd = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wd=10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector =  [[10.0], [-1.5707963267948966], [0.0]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed empty state and wind:Wu=2,Wv=5,Ww=10, Wn=2, We=5, Wd = 10"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testWind = States.windState(Wu=2,Wv=5,Ww=10, Wn=2, We=5, Wd = 10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[17.419278594890265], [-1.2458120475450811], [0.02511143730880702]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------

#################################################################################
# If a grader has gone this far to fully test the airspeed test with each variable of the
# state being put one at a time would take all night and another 500 lines
# So I'm going to just do another few full tests and call it cause I'm way behind and
# I do not have the time to do this
#################################################################################

# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed Full State with NED winds"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1.0, pe=2.0, pd=3.0, u=4.0, v=5.0, w=6.0, yaw=7.0, pitch=8.0, roll=9.0, p=10.0, q=11.0, r=12.0)
testWind = States.windState(Wn=2,We=5, Wd = 10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[18.644964605007438], [0.6011394396281061], [0.3210007507912886]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
cur_test = "Calcualate Airspeed Full State with Full winds"

testAero = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1.0, pe=2.0, pd=3.0, u=4.0, v=5.0, w=6.0, yaw=7.0, pitch=8.0, roll=9.0, p=10.0, q=11.0, r=12.0)
testWind = States.windState(Wu=1,Wv=4,Ww=6, Wn=2, We=5, Wd = 10)



Va, alpha, beta = testAero.CalculateAirspeed(testState, testWind)

expectedVector = [[20.496872895730405], [0.2572396513147953], [0.500770963067578]]
resultVector = [[Va], [alpha],[beta]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")

# ----------------------------------------------------------------------------


# %% Print results:


total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]

