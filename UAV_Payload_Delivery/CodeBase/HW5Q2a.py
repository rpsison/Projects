from ece163 . Utilities import MatrixMath as mm
from matplotlib import pyplot as plt
import math
import numpy as np
import random

n = 16000
n_series = [random.gauss(0, .0013) for i in range(n)]
dT = 0.001
T_tot = 16
n_steps = int ( T_tot /dT)

t_data = [i*dT for i in range ( n_steps )]
vdata = [0 for i in range ( n_steps )]
tau = 400
x = 0
for i in range ( n_steps - 1):
    vdata[i+1] = (math.e ** (-dT/tau))*vdata[i] + n_series[i]


plt.close ("all")
fig, ax = plt.subplots()
ax.plot ( t_data , vdata , label = "Gauss Markov Bias")
ax.set_xlabel (" time (s)")
ax.set_ylabel ("bias")
ax.legend ()
plt.show()








