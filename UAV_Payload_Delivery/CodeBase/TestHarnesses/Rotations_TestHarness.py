"""
.. modle:: RotationsTestHarness.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Compares output from RotationsGeneration.py with students'  
    function implementations
    RotationsTestHarness.py
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import os
import sys
sys.path.insert(0, os.path.abspath('..'))
import math

import ece163.Utilities.Rotations as Rotations
import ece163.Utilities.MatrixMath as MatrixMath
import argparse
import random
import pickle
import traceback


red ="\u001b[31m"
green = "\u001b[32m"
yellow = "\u001b[33m"
blue="\u001b[34m"
magenta="\u001b[35m"
cyan="\u001b[36m"
white = "\u001b[37m"


isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)


# Private methods
def matrixCompare(A, B):
    """
    Compare the elements of two matrices

    :param A: matrix (list of lists) of [m x n]
    :param B: matrix (list of lists) of [n x r]
    :return: [True, expTot, resTot]: True or False if the matrices match, the
    number of expected matching elements, and the number of resulting matching
    elements between A and B
    """
    [m, r] = MatrixMath.size(A)
    [m_c, r_c] = MatrixMath.size(B)

    expTot = (m_c * r_c)
    resTot = 0

    if (m == m_c) and (r == r_c):
        for row in range(m):
            for col in range(r):
                if isclose(A[row][col], B[row][col]) is not True:
                    print("Element [{0},{1}] is incorrect".format(row, col))
                    return [False, expTot, resTot]
                else:
                    resTot += 1
        if expTot != resTot:
            print("\r\nResulting matrix dimensions match the expected matrix dimensions")
            return [True, expTot, resTot]
    else:
        print("Error: Resulting matrix dimensions do not match the expected matrix dimensions")
        return [False, expTot, resTot]

    return [True, expTot, resTot]

#  these two functions allow for more standardized output, they should be copied to each test harness and customized
def printTestBlockResult(function, testsPassed, testCount):
    if testsPassed != testCount:
        addendum = " (TESTS FAILED)"
    else:
        addendum = ""
    print("{}/{} tests passed for {}{}".format(testsPassed, testCount, function.__name__, addendum))

def printTestFailure(function, inputs, outputs, expectedoutputs):
    print("Test Failed for {}. Please find repr version of the inputs below for testing".format(function.__name__))
    print("Inputs: {}".format(repr(inputs)))
    print("Outputs: {}".format(repr(outputs)))
    print("Expected Outputs: {}".format(repr(expectedoutputs)))


###############################################################################
parser = argparse.ArgumentParser()
parser.add_argument('-c','--continueMode', action='store_true', help='Runs all tests regardless of failures')
parser.add_argument('picklePath', nargs='?', default='Rotations_TestData.pickle', help='valid path to pickle for input')
arguments = parser.parse_args()

picklePath = arguments.picklePath
inContinueMode = arguments.continueMode

print("Beginning Test Harness for Rotations using file {}".format(picklePath))
try:
    with open(picklePath, 'rb') as f:
        allTests = pickle.load(f)
except FileNotFoundError:
    print('Test file not found, exiting')
    sys.exit(-1)

testBlocksPassed = 0  # we keep track of the number of test blocks passed

testBlockIterator = iter(allTests)  # we hard code the tests as well so we need an iterator

###############################################################################
# euler2DCM()
print("Comparing outputs for {}".format(Rotations.euler2DCM.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for yaw, pitch, roll, expectedResult in curTestBlock:  # and now we can iterate through them
    try:
        result = Rotations.euler2DCM(yaw, pitch, roll)
        [matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
        if matricesEqual is True:
            testsPassed += 1
        else:
            if not inContinueMode:
                printTestFailure(Rotations.euler2DCM, (yaw, pitch, roll), result, expectedResult)
                sys.exit(-1)
    except Exception as e:  # overly broad exception clause but we want it to catch everything
        print('Test harness failed with the exception below. It will not continue')
        print(traceback.format_exc())
        sys.exit(-1)

printTestBlockResult(Rotations.euler2DCM, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
    testBlocksPassed += 1

###############################################################################
# dcm2Euler()
print("Comparing outputs for {}".format(Rotations.dcm2Euler.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for DCM, expectedResult in curTestBlock:  # and now we can iterate through them
    try:
        result = Rotations.dcm2Euler(DCM)

        if isclose(result[0], expectedResult[0]) and isclose(result[1], expectedResult[1]) and isclose(result[2], expectedResult[2]):
            testsPassed += 1
        else:
            if not inContinueMode:
                printTestFailure(Rotations.dcm2Euler, DCM, result, expectedResult)
                sys.exit(-1)
    except Exception as e:  # overly broad exception clause but we want it to catch everything
        print('Test harness failed with the exception below. It will not continue')
        print(traceback.format_exc())
        sys.exit(-1)

printTestBlockResult(Rotations.dcm2Euler, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
    testBlocksPassed += 1

###############################################################################
# ned2enu()
print("Comparing outputs for {}".format(Rotations.ned2enu.__name__))
curTestBlock = next(testBlockIterator)  # we now have all the tests for this block
testsPassed = 0
for NEDpoints, expectedResult in curTestBlock:  # and now we can iterate through them
    try:
        result = Rotations.ned2enu(NEDpoints)
        [matricesEqual, expTot, resTot] = matrixCompare(result, expectedResult)
        if matricesEqual is True:
            testsPassed += 1
        else:
            if not inContinueMode:
                printTestFailure(Rotations.ned2enu, NEDpoints, result, expectedResult)
                sys.exit(-1)
    except Exception as e:  # overly broad exception clause but we want it to catch everything
        print('Test harness failed with the exception below. It will not continue')
        print(traceback.format_exc())
        sys.exit(-1)

printTestBlockResult(Rotations.ned2enu, testsPassed, len(curTestBlock))
if testsPassed == len(curTestBlock):
    testBlocksPassed += 1

###############################################################################
if testBlocksPassed == len(allTests):
    print(f"{green}All tests Passed for Rotations{white}")
else:
    print(f"{green}{testBlocksPassed}/{len(allTests)} tests blocks passed for Rotations{white}")