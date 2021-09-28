from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math
from ece163.Constants import VehiclePhysicalConstants as VPC

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
rudderData = [0 for i in range ( n_steps )]

wind_data = [ (0 if t < 1 else 10* math .pi /180) for t in t_data ]
Kp = -10
x = [[0] ,[0]]

Ki = -.75

accumulator = 0

Iterm = accumulator * Ki



for i in range ( n_steps ):
    # record our data
    yaw_data [i] = mm. multiply (C, x) [0][0]
    # find u(t):
    accumulator += yaw_data[i]
    Iterm = accumulator * Ki
    print(Iterm)
    if Iterm < -100:
        Iterm = -100
    if Iterm > 50:
        Iterm = 50
    rudderData[i] = Kp * (0 - yaw_data[i]) + (Iterm * (0 - yaw_data[i]))


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
fig, (ax, ax2) = plt.subplots(2,1)
ax.plot ( t_data , wind_data , label = " wind angle ")
ax.plot ( t_data , yaw_data , label = " yaw response ")
ax.set_xlabel (" time (s)")
ax.set_ylabel (" angle (rad )")
ax.legend ()


ax2.plot ( t_data , rudderData , label = " rudder deflection ")
ax2.set_xlabel (" time (s)")
ax2.set_ylabel (" angle (rad )")
ax2.legend()

plt.show()