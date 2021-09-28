
import random
from matplotlib import pyplot as plt
import ece163.Utilities.MatrixMath as mm
import numpy as np
import math
n = 1000
u_series = [random.gauss(0, 1) for i in range (n)]

naive = np.zeros(1000)
# 4.a
for i in range(n):
    naive[i] = 0.059 * u_series[i]


forward = np.zeros(1000)
discretized = np.zeros(1000)
x =  [[0], [0]]
xp1 = [[0], [0]]
#find the xi vector to put it together
Va = 25
dT = 0.01
Lv = 200
sigmav = 1.06

A = [[-2*Va/Lv, -1 * (Va ** 2)/(Lv ** 2)], [1, 0]]
B = [[1], [0]]
C = [1, (Va **2)/(math.sqrt(3) * Lv)]

C[0] = ((sigmav)*math.sqrt((3*Va)/(math.pi * Lv))) * C[0]
C[1] = ((sigmav)*math.sqrt((3*Va)/(math.pi * Lv))) * C[1]

for i in range(1000):
    xp1 = mm.add(x,  (mm.scalarMultiply(dT, mm.add(mm.multiply(A, x), mm.scalarMultiply(u_series[i], B)) ) ) )
    x = xp1
    forward[i] = C[0] * x[0][0] + C[1] * x[1][0]


# ----------------------------------------------------------------
# 4c

# Set up necessary variables
x =  [[0], [0]]
xp1 = [[0], [0]]

e = math.e

phiMat = [[1-(Va/Lv)*dT, -((Va/Lv) ** 2)*dT], [dT, 1+(Va/Lv)*dT]]

gammaMat =[[dT], [((Lv/Va)**2) * ((e **((Va/Lv)*dT)) - 1) - (Lv)/(Va) * dT]]

Hmat = [1, (Va)/(math.sqrt(3)*Lv)]


phiMat = mm.scalarMultiply(e **( (-Va/Lv) *dT), phiMat)

scale = (e **( (-Va/Lv) *dT))

gammaMat[0][0] = gammaMat[0][0] * scale
gammaMat[1][0] = gammaMat[1][0] * scale


Hmat[0] = (sigmav*math.sqrt((3*Va)/(math.pi * Lv))) * Hmat[0]
Hmat[1] = (sigmav*math.sqrt((3*Va)/(math.pi * Lv))) * Hmat[1]

# Execute the for loop
for i in range(1000):
    xp1 = mm.add(mm.multiply(phiMat, x),   mm.scalarMultiply(u_series[i], gammaMat))
    x = xp1
    discretized[i] = Hmat[0] * x[0][0] + Hmat[1] * x[1][0]



plt.plot(naive)
plt.plot(forward)
plt.plot(discretized)
plt.xlabel("Time Step")
plt.ylabel("Wv")
plt.legend(["naive","forward", "discretized"], loc="lower left")
plt.title("")
plt.show()