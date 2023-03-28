'''
SOAR Payload Model Iris I
Kalman Filter using filterpy by rlabbe
Implemented by: Findlay Brown



Working version as of: 21-01-2023 4:10pm

Filterpy documentation: https://github.com/rlabbe/filterpy

'''

import matplotlib.pyplot as plt
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise, Saver

r_std, q_std = 0.003, 1
dt = 0.02
altitudeData = np.genfromtxt(
    'data/fixedAltitudeData.csv', delimiter=',', dtype=np.float16, skip_header=True)
accelerationData = np.genfromtxt(
    'data/fixedAccelerationData.csv', delimiter=',', dtype=np.float16, skip_header=True)

cv = KalmanFilter(dim_x=2, dim_z=1, dim_u=1)

cv.x = np.array([7000., 0.])  # position, velocity  # Current state estimate
cv.F = np.array([[1, dt],
                 [0, 1]])              # State Transition matrix
# Measurement noise covariance matrix
cv.R = np.array((r_std**2))
cv.H = np.array([[1., 0.]])                     # Measurement function
# Current state covariance matrix
cv.P = np.diag([.1**2, .03**2])
# Process noise covariance matrix
cv.Q = Q_discrete_white_noise(dim=2, dt=dt, var=q_std**2)

cv.B = np.array([[0.5*(dt**2)],  # Control-input matrix
                 [dt]])

saver = Saver(cv)
for i, z in enumerate(altitudeData):
    cv.predict(u=[accelerationData[i]*9.807/1000])
    cv.update([z])
    saver.save()  # save the filter's state
saver.to_array()
plt.plot(altitudeData, label='Measurements')
plt.plot(saver.x[:, 0], label='Estimates')
# plot all of the priors
plt.plot(saver.x_prior[:, 0], label='Prev Estimates')
plt.legend()
# plot mahalanobis distance
'''plt.figure()
percentage = []
for i in range(np.size(altitudeData)):
    if altitudeData[i] == 0:
        continue
    else:
        percentage.append((saver.x[:, 0][i]/altitudeData[i])*100)
print(percentage)
'''
# plt.plot(percentage,label ='Percentage')
# plt.yticks([x+1 for x in range(100)])
# plt.plot(saver.mahalanobis)
# plt.legend()
plt.show()


"""
import numpy as np
n = 100     # n: number of iterations to run the filter for
dt = 0.1    # dt: time interval for updates
# v: velocity = a*dt + velocity_prev
p_a = 5     # p_a: uncertainty in acceleration
            # q: process noise variance (uncertainty in the system's dynamic model)
r = 2       # r: measurement uncertainty
# Z: list of position estimates derived from sensor measurements

# Zd: list of altitude measurements (m x10)
Zd = np.genfromtxt('andromedaAltitude.csv',delimiter=',', dtype = np.float16,skip_header=True)
# Ad: list of acceleration measurements (mg)
Ad = np.genfromtxt('andromedaAcceleration.csv',delimiter=',', dtype = np.float16,skip_header=True)
# Vd: list of velocity calculated from acceleration
Vd = [0]
# v[i+1] = a[i]*dt + v[i]
[Vd.append(np.float16((Ad[index]*9.81*dt)/1000)) for index in range(np.size(Ad))]
print(Vd)


def initialize():
    x = 0 
    p = 0.5
    return x, p

def predict(x, p):
    # Prediction 
    x = x + (dt**2)*a                    # State Transition Equation (Dynamic Model or Prediction Model)
    p = p + (dt**2 * p_v) + q       # Predicted Covariance equation
    return x, p

def measure():
    z = Z[i]
    return z

def update(x, p, z):
    k = p / ( p + r)                # Kalman Gain
    x = x + k * (z - x)             # State Update
    p = (1 - k) * p                 # Covariance Update
    return x, p

def runKalmanFilter():
    x, p = initialize()

    for j in range(1, n):
        x, p = predict(x, p)
        z = measure()
        x, p = update(x, p, z) 
"""
