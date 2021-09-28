"""Personal test harness for the Chpater 3 functions
Matthew Bennett mabennet@ucsc.edu
"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG
import ece163.Modeling.VehicleDynamicsModel as VDM
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


print("Beginning testing of Derivative")

print("Pn Pe Pd tests")
#----------------------------------------------------------------------------
cur_test = "Pn test 1 yaw 0, pitch 0, roll 0 u = 1, v = 0 w = 0 () "
testVDM = VDM.VehicleDynamicsModel()
inputState = States.vehicleState(u=1, v=0, w=0)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultState = testVDM.derivative(inputState, inputForces)

expectedVector = [[1], [0], [0]]
resultVector = [[resultState.pn], [resultState.pe], [resultState.pd]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "Pe test 1 yaw 0, pitch 0, roll 0 u = 0, v = 1 w = 0 () "
testVDM = VDM.VehicleDynamicsModel()
inputState = States.vehicleState(u=0, v=1, w=0)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultState = testVDM.derivative(inputState, inputForces)

expectedVector = [[0], [1], [0]]
resultVector = [[resultState.pn], [resultState.pe], [resultState.pd]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "Pd test 1 yaw 0, pitch 0, roll 0 u = 0, v = 0 w = 1 () "
testVDM = VDM.VehicleDynamicsModel()
inputState = States.vehicleState(u=0, v=0, w=1)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultState = testVDM.derivative(inputState, inputForces)

expectedVector = [[0], [0], [1]]
resultVector = [[resultState.pn], [resultState.pe], [resultState.pd]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "Pned test 1 yaw 0, pitch 0, roll 0 u = 1, v = 1 w = 1 () "
testVDM = VDM.VehicleDynamicsModel()
inputState = States.vehicleState(u=1, v=1, w=1)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultState = testVDM.derivative(inputState, inputForces)

expectedVector = [[1], [1], [1]]
resultVector = [[resultState.pn], [resultState.pe], [resultState.pd]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
cur_test = "Pn test 2 yaw 30, pitch 45 roll 60 u = 1, v = 0 w = 0 () "
testVDM = VDM.VehicleDynamicsModel()
inputState = States.vehicleState(u=1, v=0, w=0, yaw=30, pitch=45, roll=60)
inputForces = Inputs.forcesMoments()
expectedState = States.vehicleState()
resultState = testVDM.derivative(inputState, inputForces)

expectedVector = [[(6**(1/2))/4], [(3*(2**(1/2))-2)/8], [(6**(1/2) + 2*(3**(1/2)))/8]]
resultVector = [[resultState.pn], [resultState.pe], [resultState.pd]]


if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#----------------------------------------------------------------------------
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]

