

import math
from . import MatrixMath

#Given the DCM find the yaw pitch and roll
#use the equations:
#
# psi = arctan(M0.1/M0.0)
# theta = arcsin(M0.2)
# phi = arctan(M1.2/M2.2)
def dcm2Euler(DCM):
    psi = 0
    theta = 0
    phi = 0

    psi =  math.atan2(DCM[0][1], DCM[0][0])
    theta = math.asin(DCM[0][2]) * -1
    phi = math.atan2(DCM[1][2], DCM[2][2])
    points =  [psi, theta, phi]
    return psi, theta, phi

def euler2DCM(yaw, pitch, roll):
    DCM = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]
    DCM[0][0] = math.cos(pitch) * math.cos(yaw)
    DCM[0][1] = math.cos(pitch) * math.sin(yaw)
    DCM[0][2] = -1 * math.sin(pitch)

    DCM[1][0] = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
    DCM[1][1] = math.sin(roll)*math.sin(pitch)*math.sin(yaw) + math.cos(roll)*math.cos(yaw)
    DCM[1][2] = math.sin(roll)*math.cos(pitch)

    DCM[2][0] = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
    DCM[2][1] = math.cos(roll)*math.sin(pitch)*math.sin(yaw) - math.sin(roll)*math.cos(yaw)
    DCM[2][2] = math.cos(roll)*math.cos(pitch)

    return DCM



def ned2enu(points):

    tempE = 0
    tempN = 0
    tempU = 0
    for i in range(len(points)):
        tempN = points[i][0]
        tempE = points[i][1]
        tempU = points[i][2] * -1

        points[i][0] = tempE
        points[i][1] = tempN
        points[i][2] = tempU
    return points