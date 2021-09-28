"""This file is testing the chapter5 VehiclePerturbation utility model functions
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
from ece163.Controls import VehicleTrim
from ece163.Controls import VehiclePerturbationModels as Perturb

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

print("Beginning testing of the Perturbation Model")

print("Ensure we can call Compute Trim")
#----------------------------------------------------------------------------
cur_test = "ComputeTrim test"
testTrim = VehicleTrim.VehicleTrim()

testTrim.computeTrim()


print("If you're reading this it means we called the ComputeTrim, and it didn't crash. Yay!")


#----------------------------------------------------------------------------
print("Ensure we can use the trim values to call CreateTransferFunction")

trimState = testTrim.getTrimState()
trimInputs = testTrim.getTrimControls()

testPerturb = Perturb.CreateTransferFunction(trimState, trimInputs)

print("If you're reading this it means we called the CreateTransferFunction, and it didn't crash. Yay!")


