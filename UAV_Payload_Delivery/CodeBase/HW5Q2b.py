from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math
import numpy as np
import random

a_1 = 0.79
a_2 = 4.19
b_2 = 4.19

A = [[-a_1 , -a_2 ],[1 , 0]]
B = [[1] , [0]]
C = [[0 , b_2 ]]

dT = 0.001
T_tot = 16
n_steps = int ( T_tot /dT)

t_data = [i*dT for i in range ( n_steps )]
yaw_data = [0 for i in range ( n_steps )]
yaw_estimate = [0 for i in range ( n_steps )]
rudderData = [0 for i in range ( n_steps )]

wind_data = [ (0 if t < 1 else 10* math .pi /180) for t in t_data ]
Kp = -15
x = [[0] ,[0]]

# add in the code to get the gauss markov drift
n = 16000
n_series = [random.gauss(0, .0013) for i in range(n)]
dT = 0.001
T_tot = 16
n_steps = int ( T_tot /dT)

t_data = [i*dT for i in range ( n_steps )]
vdata = [0 for i in range ( n_steps )]
tau = 400
x = [[0] ,[0]]
for i in range ( n_steps - 1):
    vdata[i+1] = (math.e ** (-dT/tau))*vdata[i] + n_series[i]



for i in range ( n_steps ):
    # record our data
    yaw_data [i] = mm.multiply(C, x) [0][0]
    # get the yaw estimate
    yaw_estimate[i] = yaw_data[i] + vdata[i]

    # find u(t):
    rudderData[i] = Kp * (0 - yaw_estimate[i])
    u = wind_data[i]  + rudderData[i] * ((VPC.CndeltaR)/(VPC.Cnbeta))
    # calculate derivative :
    x_dot = mm. add (
    mm. multiply (A,x),
    mm. scalarMultiply (u,B))
    # forward euler update :
    x = mm. add (
    x,
    mm. scalarMultiply (dT , x_dot ))

plt.close ("all")
plt.figure(1)
plt.plot ( t_data , yaw_estimate , label = "Yaw Estimate")
plt.plot ( t_data , yaw_data , label = " True Yaw")
plt.xlabel (" time (s)")
plt.ylabel (" angle (rad )")
plt.legend ()



plt.show()