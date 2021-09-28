"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

 It is meant to be run from the root directory of the repo with:

python testChapter2.py

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

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


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")



"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

cur_test = "Euler2dcm pitch test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(0, 90*math.pi/180, 0)
orig_vec = [[1],[0],[0]]
expected_vec =  [[0], [0.0], [1.0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "Euler2dcm roll test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(0, 0, 90*math.pi/180)
orig_vec = [[0],[0],[1]]
expected_vec =  [[0.0], [1.0], [0]]

actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec)):
	print(f"{expected_vec} != {actual_vec}")



cur_test = "Euler2dcm common test 1 "
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(30*math.pi/180,90*math.pi/180, 60*math.pi/180)
orig_vec = [[1],[2],[3]]
expected_vec = [[-3.0], [2.2320508075688776], [-0.1339745962155612]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "Euler2dcm common test 2 "
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(30*math.pi/180,45*math.pi/180, 90*math.pi/180)
orig_vec = [[2],[2],[2]]
expected_vec =  [[0.5176380902050415], [3.346065214951232], [-0.7320508075688773]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "Euler2dcm All 0 test "
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(30*math.pi/180,45*math.pi/180, 60*math.pi/180)
orig_vec = [[0],[0],[0]]
expected_vec = [[0], [0], [0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#-----------------------------------------------------------------------------------
#dcm2euler tests

print("Beginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler Trivial ALl 0s"
#we know that rotaitng [1,0,0] by 90 degrees about Z should produce [0,-1,0], so

inputDCM = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]
yaw, pitch, roll = Rotations.dcm2Euler(inputDCM)
resultEulers = [[yaw], [pitch], [roll]]
expectEulers = [[0], [0], [0]]
if not evaluateTest(cur_test, compareVectors(expectEulers, resultEulers) ):
	print(f"{expectEulers} != {resultEulers}")

cur_test = "dcm2Euler yaw "
#we know that rotaitng [1,0,0] by 90 degrees about Z should produce [0,-1,0], so

inputDCM = [[math.cos(45*math.pi/180), math.sin(45*math.pi/180), 0],[-1 * math.sin(45*math.pi/180), math.cos(45*math.pi/180), 0],[0, 0, 1]]
yaw, pitch, roll = Rotations.dcm2Euler(inputDCM)
resultEulers = [[yaw], [pitch], [roll]]
expectEulers = [[0.7853981633974483], [-0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(expectEulers, resultEulers) ):
	print(f"{expectEulers} != {resultEulers}")

cur_test = "dcm2Euler pitch "
#we know that rotaitng [1,0,0] by 90 degrees about Z should produce [0,-1,0], so

psi = 0
theta = 45*math.pi/180
phi = 0

inputDCM = [[math.cos(theta) * math.cos(psi), math.cos(theta)*math.sin(psi), -1* math.sin(theta)],
			[math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi), math.sin(phi)*math.cos(theta)],
			[math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.sin(psi)-math.sin(phi)*math.cos(psi), math.cos(phi)*math.cos(theta)]]
yaw, pitch, roll = Rotations.dcm2Euler(inputDCM)
resultEulers = [[yaw], [pitch], [roll]]
expectEulers = [[0.0], [0.7853981633974484], [0.0]]
if not evaluateTest(cur_test, compareVectors(expectEulers, resultEulers) ):
	print(f"{expectEulers} != {resultEulers}")

cur_test = "dcm2Euler roll "
#we know that rotaitng [1,0,0] by 90 degrees about Z should produce [0,-1,0], so

psi = 0
theta = 0
phi =  45*math.pi/180

inputDCM = [[math.cos(theta) * math.cos(psi), math.cos(theta)*math.sin(psi), -1* math.sin(theta)],
			[math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi), math.sin(phi)*math.cos(theta)],
			[math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.sin(psi)-math.sin(phi)*math.cos(psi), math.cos(phi)*math.cos(theta)]]
yaw, pitch, roll = Rotations.dcm2Euler(inputDCM)
resultEulers = [[yaw], [pitch], [roll]]
expectEulers = [[0.0], [0], [0.7853981633974484]]
if not evaluateTest(cur_test, compareVectors(expectEulers, resultEulers) ):
	print(f"{expectEulers} != {resultEulers}")

cur_test = "dcm2Euler common"
#we know that rotaitng [1,0,0] by 90 degrees about Z should produce [0,-1,0], so

psi =  30*math.pi/180
theta =  45*math.pi/180
phi =  60*math.pi/180

inputDCM = [[math.cos(theta) * math.cos(psi), math.cos(theta)*math.sin(psi), -1* math.sin(theta)],
			[math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.sin(phi)*math.sin(theta)*math.sin(psi) + math.cos(phi)*math.cos(psi), math.sin(phi)*math.cos(theta)],
			[math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.sin(psi)-math.sin(phi)*math.cos(psi), math.cos(phi)*math.cos(theta)]]
yaw, pitch, roll = Rotations.dcm2Euler(inputDCM)
resultEulers = [[yaw], [pitch], [roll]]
expectEulers = [[0.5235987755982988], [0.7853981633974484], [1.0471975511965976]]
if not evaluateTest(cur_test, compareVectors(expectEulers, resultEulers) ):
	print(f"{expectEulers} != {resultEulers}")


#-----------------------------------------------------------------------------------
#ned2enu tests

print("Beginning testing of ned2enu")

cur_test = "ned2enu trivial all 0s"

inputVector = [[0, 0, 0], [0, 0, 0]]
expectedVector = [[0, 0, 0], [0, 0, 0]]
resultVector = Rotations.ned2enu(inputVector)
resultVector = mm.transpose(inputVector)
expectedVector = mm.transpose(expectedVector)
if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")


cur_test = "ned2enu standard use case no repeating values"

inputVector = [[1, 2, 3], [1, 2, 3]]
expectedVector = [[2, 1, -3], [2, 1, -3]]
resultVector = Rotations.ned2enu(inputVector)
resultVector = mm.transpose(inputVector)
expectedVector = mm.transpose(expectedVector)
if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")


cur_test = "ned2enu standard with repeating values"

inputVector = [[1, 1, 3], [1, 1, 3]]
expectedVector = [[1, 1, -3], [1, 1, -3]]
resultVector = Rotations.ned2enu(inputVector)
resultVector = mm.transpose(inputVector)
expectedVector = mm.transpose(expectedVector)
if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")


cur_test = "ned2enu negative roll at start"

inputVector = [[1, 1, -3], [1, 1, -3]]
expectedVector = [[1, 1, 3], [1, 1, 3]]
resultVector = Rotations.ned2enu(inputVector)
resultVector = mm.transpose(inputVector)
expectedVector = mm.transpose(expectedVector)
if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
	print(f"{expectedVector} != {resultVector}")
#-----------------------------------------------------------------------------------
# #getNewPoints tests
#
# print("Beginning testing of getNewPoints")
#
# cur_test = "get new points trivial all 0s"
#
# x = 0
# y = 0
# z = 0
# yaw = 0
# pitch = 0
# roll = 0
# self = 0
# resultPoints = VG.getNewPoints()
# #resultPoints = VG.getNewPoints(x, y, z, yaw, pitch, roll)
# expectedPoints = [0, 0, 0]
# if not evaluateTest(cur_test, compareVectors(expectedVector, resultVector)):
# 	print(f"{expectedVector} != {resultVector}")



#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]