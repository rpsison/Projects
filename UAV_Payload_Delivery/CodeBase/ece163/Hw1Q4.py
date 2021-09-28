
import math
import Utilities.Rotations as Rotations
import Utilities.MatrixMath as MM
# 4.a code

# calculate using the forward euler integration
"""
I'm going to use Max's terminology form the section to help me interpret how to code this
trying to find X. X is [phi,theta,psi]t0.1
given [phi,theta,psi]t0 = [0,0,0] = x0
deltaT is 0.1
need D_dtx0 which wel call xdot

find xdot with equation 3.16
"""
deltaT = 0.1
x0 = [[0],[0],[0]]

pqr = [[1], [1], [1]]

DCM = Rotations.euler2DCM(0, 0, 0)
xdot = MM.multiply(DCM, pqr)
print("xdot is =", xdot)
X = [[0], [0], [0]]

for i in range(len(xdot)):
    xdot[i][0] = deltaT * xdot[i][0]

for j in range(len(xdot)):
    X[j][0] = xdot[j][0] + x0[j][0]

print("Section A [phi,theta,psi]t0 = ", X)

#4b

"""
To determine the value here we need to find the Rotation matrices here.
 To do so we need Rdot which we can get from the DCM
"""
p = 1
q = 1
r = 1
R = Rotations.euler2DCM(0, 0, 0)
wx = [[0, -r, q],
      [r, 0, -p],
      [-q, p, 0]]
Rdot = MM.multiply(wx, R)
Rdot = MM.scalarMultiply(deltaT, Rdot)

Rt01 = MM.add(R, Rdot)

print("Rt01 = ", Rt01)
angles = Rotations.dcm2Euler(Rt01)
print("Section B [phi,theta,psi]t0 = ", angles)


# #4c
# """
# The system here would need to implement the system described in the attitude cheat sheet once again
#
# """
# p = 1
# q = 1
# r = 1
# R0 = Rotations.euler2DCM(0, 0, 0)
# wx = [[0, -r, q],
#       [r, 0, -p],
#       [-q, p, 0]]
# I = [[1, 0, 0],
#      [0, 1, 0],
#      [0, 0, 0]]
# exponent = (MM.scalarMultiply(deltaT, wx))
# result = (math.e)^(exponent)
# result = MM.multiply(result, R0)
# print("Section C [phi,theta,psi]t0 = ", angles)