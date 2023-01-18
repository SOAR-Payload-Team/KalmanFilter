"""
Refrences/Research

https://www.kalmanfilter.net/stateextrap.html

https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/kleeman_understanding_kalman.pdf

https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html

https://github.com/rlabbe/filterpy
"""


import matplotlib.pyplot as plt
import numpy as np

# Eest = Error in estimate
# Emea = Error in measurement
# Pest = previous estimate
# Mval = measured value
# gain = kalman gain

def kalmanGain(Eest,Emea):
    gain = float(Eest / (Eest + Emea))
    return gain 

def currentEst(Pest,Mval,gain):
    return Pest + gain*(Mval-Pest) #+ 1*0.5*(0.1**2)

def updateErrorEstimate(Pest,gain):
    return (1-gain)*(Pest)

def filter(data):
    Eest = 10
    Emea = 2
    
    estimates = [data[0]]
    
    i = 1
    while i < len(data):
        gain = kalmanGain(Eest,Emea)
        estimates.append(currentEst(estimates[i-1],data[i],gain))
        Eest = updateErrorEstimate(estimates[i-1],gain)
        i += 1
    
    return estimates,Eest

def filterRepeat(data,estimates,Eest):
    Emea = Eest
    #gain = kalmanGain(Eest,Emea)
    newEstimates = []
    gain = kalmanGain(Eest,Emea)
    newEstimates.append(currentEst(estimates[0],data[0],gain))
    Eest = updateErrorEstimate(newEstimates[0],gain)
    
    i = 1
    while i < len(data):
        gain = kalmanGain(Eest,Emea)
        newEstimates.append(currentEst(estimates[i-1],data[i],gain))
        Eest = updateErrorEstimate(newEstimates[i-1],gain)
        i += 1
    
    return newEstimates,Eest

plt.close('all')
data = np.genfromtxt('andromedaAltitude.csv',delimiter=',', dtype = float)
#data = np.genfromtxt('sampleAltitude.csv',delimiter=',', dtype = float)


estimates,Eest = filter(data)
print(Eest)
estimates1,Eest = filterRepeat(data,estimates,Eest)
#estimates2,Eest = filterRepeat(estimates,estimates1,Eest)

for i in range(1000):
    estimates1,Eest = filterRepeat(data,estimates1,Eest)
    #estimates1,Eest = filterRepeat(estimates1,estimates2,Eest)
    #estimates2,Eest = filterRepeat(estimates2,estimates1,Eest)

plt.plot(data)
#plt.plot(estimates)
plt.plot(estimates1)
#plt.plot(estimates2)


plt.show()

