
from matplotlib import pyplot as plt
import math
import numpy as np
import scipy.optimize


errors = np.load("ErrorsArray.npy")
altitudes = np.load("targetAltitudes.npy")
airspeeds = np.load("targetAirspeeds.npy")

#print(errors, altitudes, airspeeds)



def planeDif(x):
    a = x[0]
    b = x[1]
    c = x[2]
    d = x[3]
    e = x[4]
    difference = 0
    for i in range(10):
        for j in range(10):
            dif = np.abs((airspeeds[i]**2*a + airspeeds[i]*b+ altitudes[j]**2*c+ altitudes[j]*d+ e) - errors[i][j])
            difference = difference+dif


    return difference
x0 = np.array([1,1,1,1,1])
AB = scipy.optimize.minimize(planeDif,x0, method='SLSQP').x

AltVSsAirspeedVSError = plt.figure()

ax = plt.axes(projection='3d')


for i in range(10):
    for j in range(10):
        
        ax.scatter(airspeeds[i],altitudes[j],errors[i][j])
ax.set_xlabel('Airspeed')
ax.set_ylabel('Altitude')
ax.set_zlabel('Error')


print(AB)
print(planeDif(AB)/100)
xx, yy = np.meshgrid(airspeeds, altitudes)
z = AB[0]*xx**2 + AB[1]*xx + AB[2]*yy**2 + AB[3]*yy + AB[4]

#plot the plane
ax.plot_surface(xx, yy, z, alpha=0.5)

plt.show()



