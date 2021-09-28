from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math

a_1 = 0.79
a_2 = 4.19
b_2 = 4.19

A = [[-a_1 , -a_2 ],[1 , 0]]
B = [[1] , [0]]
C = [[0 , b_2 ]]

dT = 0.01
T_tot = 16
n_steps = int ( T_tot /dT)

t_data = [i*dT for i in range ( n_steps )]
yaw_data = [0 for i in range ( n_steps )]

wind_data = [ (0 if t < 1 else 10* math .pi /180) for t in t_data ]


x = [[0] ,[0]]
for i in range ( n_steps ):
    # record our data
    yaw_data [i] = mm. multiply (C, x) [0][0]
    # find u(t):
    u = wind_data [i]
    # calculate derivative :
    x_dot = mm. add (
    mm. multiply (A,x),
    mm. scalarMultiply (u,B))
    # forward euler update :
    x = mm. add (
    x,
    mm. scalarMultiply (dT , x_dot ))

plt.close ("all")
fig, ax = plt.subplots()
ax.plot ( t_data , wind_data , label = " wind angle ")
ax.plot ( t_data , yaw_data , label = " yaw response ")
ax.set_xlabel (" time (s)")
ax.set_ylabel (" angle (rad )")
ax.legend ()
plt.show()