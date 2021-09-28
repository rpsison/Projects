from matplotlib import pyplot as plt
import numpy as np
import math

def sigma(a, a0, M):
    num = (1 + np.exp(-M * (a - a0)) + np.exp(M * (a + a0)))
    den = (1 + np.exp(-M * (a - a0))) * (1 + np.exp(M * (a + a0)))
    return num / den

blend_fig = plt.figure("Sigma  vs  Alpha  for  several  values  of M")
# plot  100  points  from  -3.2 to 3.2:
a = np.linspace(-3.2, 3.2, 100)
# plot  several  values  of M from  0.1 to 100:



for M in np.logspace(-1, 2, 5):
    a0 = 1.5
    sigma_vals = [sigma(ai, a0, M) for ai in a]
    plt.plot(a, sigma_vals, label=f"M={M:0.2f}")

plt.legend(loc = "lower left")

blend_fig2 = plt.figure("Sigma  vs  Alpha  for  several  values  of a0")

for a0 in np.linspace(0, 7, 15):
    M = 3
    sigma_vals = [sigma(ai, a0, M) for ai in a]
    plt.plot(a, sigma_vals, label=f"a0={a0:0.2f}")

plt.legend(loc = "lower left")

#-----------------------------------------------------------------------------------------------------------------------
""" graph the real data"""
alpha = [
0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

C_L = [
0.0, 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77, 0.8504, 0.9387,
1.0141, 1.0686, 1.0971, 1.0957, 1.0656, 1.0145, 0.9356, 0.8996, 0.8566, 0.8226,
0.8089, 0.8063, 0.8189, 0.8408, 0.8668, 0.9023, 0.9406, 0.9912, 0.855, 0.98,
1.035, 1.05, 1.02, 0.955, 0.875, 0.76, 0.63, 0.5, 0.365, 0.23,
0.09, -0.05, -0.185, -0.32, -0.45, -0.575, -0.67, -0.76, -0.85, -0.93,
-0.98, -0.9, -0.77, -0.67, -0.635, -0.68, -0.85, -0.66, 0.0 ]
C_D = [
0.0074, 0.0075, 0.0076, 0.0079, 0.0083, 0.0091, 0.0101, 0.0111, 0.0126, 0.0138,
0.0152, 0.0168, 0.0186, 0.0205, 0.0225, 0.0249, 0.0275, 0.0303, 0.145, 0.26,
0.282, 0.305, 0.329, 0.354, 0.379, 0.405, 0.432, 0.46, 0.57, 0.745,
0.92, 1.075, 1.215, 1.345, 1.47, 1.575, 1.665, 1.735, 1.78, 1.8,
1.8, 1.78, 1.75, 1.7, 1.635, 1.555, 1.465, 1.35, 1.225, 1.085,
0.925, 0.755, 0.575, 0.42, 0.32, 0.23, 0.14, 0.055, 0.025]


dataFig, axs = plt.subplots(nrows=2, ncols=1)
dataFig.suptitle('Actual Data given to us', fontsize=16)

# This is the actual data I'm going to move this up here so it gets plotted earlier than the guesses.
# Also since its the actual data it gets red and green, seems appropriate
axs[0].plot(alpha, C_L,'g-o' )
axs[1].plot(alpha, C_D, 'r-o')
axs[0].set(ylabel='CL')
axs[1].set(xlabel='alpha', ylabel='CD')

# ----------------------------------------------------------------------------------
"""
Making the model to graph my guess
"""

# My first guess
CL0 = 0
CLA = .1
CDp = 0
M = 1
a0 = 20
eAR = 500

"""I don't want to deal with numpy functoins like np.append. I know what the length of all these vecotrs will be anyway.
They'll all be the same length as alpha so I'm just gonna copy the alpha data into them and it'll be overwritten as needed
anyway. Is there a prettier way to do this? Yes.
Am I going to learn a whole new library worth of functions just to make it a little prettier? No, not at this hour.
"""

myCL = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

myCD = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

mySigma = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]


CLlam = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

CDlam = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

CLturb = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]

CDturb = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 30.0, 35.0,
40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0,
90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0,
140.0, 145.0, 150.0, 155.0, 160.0, 165.0, 170.0, 175.0, 180.0]


for i in range(len(alpha)):
    mySigma[i] = sigma(alpha[i], a0, M)

for i in range(len(alpha)):
    CLlam[i] = CL0 + CLA * alpha[i]
    CDlam[i] = CDp + ((CL0 + CLA * alpha[i])**2)/((math.pi)*eAR)

    CLturb[i] = 2*math.sin(math.radians(alpha[i]))*math.cos(math.radians(alpha[i]))
    CDturb[i] = 2*(math.sin(math.radians(alpha[i])))**2

for i in range(len(mySigma)):
    myCL[i] = (1-mySigma[i])*CLlam[i] + mySigma[i]*CLturb[i]
    myCD[i] = (1-mySigma[i])*CDlam[i] + mySigma[i]*CDturb[i]




guessFig1, axes = plt.subplots(nrows=2, ncols=1)
guessFig1.suptitle('First Guesses', fontsize=16)

#these guesses will probably be kinda ugly so they get the ugly colors cyan and magenta.
axes[0].plot(alpha, myCL, 'c-o')
axes[1].plot(alpha, myCD, 'm-o')
axes[0].set(ylabel='CL')
axes[1].set(xlabel='alpha', ylabel='CD')


#refined guess
"""
If I was smarter than I am I would've made this a function that I could call and just pass it the values of the guesses.
But this works and I'm tired so for now I'm just gonna copy and paste it so it'll run again. Yes I recognize that
theres a prettier way to do this, but once again I am sleepy and this works fine for now. At least python is nicer
than MATLAB.
"""
CL0 = 0
CLA = .1
CDp = 0
M = .5
a0 = 15
eAR = 1000


for i in range(len(alpha)):
    mySigma[i] = sigma(alpha[i], a0, M)

for i in range(len(alpha)):
    CLlam[i] = CL0 + CLA * alpha[i]
    CDlam[i] = CDp + ((CL0 + CLA * alpha[i])**2)/((math.pi)*eAR)

    CLturb[i] = 2*math.sin(math.radians(alpha[i]))*math.cos(math.radians(alpha[i]))
    CDturb[i] = 2*(math.sin(math.radians(alpha[i])))**2

for i in range(len(mySigma)):
    myCL[i] = (1-mySigma[i])*CLlam[i] + mySigma[i]*CLturb[i]
    myCD[i] = (1-mySigma[i])*CDlam[i] + mySigma[i]*CDturb[i]




guessFig2, axes = plt.subplots(nrows=2, ncols=1)
guessFig2.suptitle('Refined Guesses', fontsize=16)

#these will hopefully look pretty good so they get my favorite colors blue and orange cause I made them
axes[0].plot(alpha, myCL, 'b-o')
axes[1].plot(alpha, myCD, '-o', color='orange')
axes[0].set(ylabel='CL')
axes[1].set(xlabel='alpha', ylabel='CD')




plt.show()
