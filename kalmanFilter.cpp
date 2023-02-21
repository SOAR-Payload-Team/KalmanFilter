/*
 * File name: kalmanFilter.cpp
 *
 * Contains: Functions for kalman filter  
 * 
 * Author: Findlay Brown
 * Language: C++
 */

#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "kalmanFilter.h"

using namespace std;
/*kalmanFilter::kalmanFilter(double initialX){
    // Default constructor
    X[0] = initialX;
}*/

void kalmanFilter::predict(double u){
/*
    Function: predict
    ----------
    Predict next state (prior) using the Kalman filter state propagation
    equations.
    Parameters
    ----------
    u : double, Control value: acceleration
    ----------
    B : double [2][2], or None
        Optional control transition matrix; a value of None
        will cause the filter to use `self.B`.
    F : np.array(dim_x, dim_x), or None
        Optional state transition matrix; a value of None
        will cause the filter to use `self.F`.
    Q : np.array(dim_x, dim_x), scalar, or None
        Optional process noise matrix; a value of None will cause the
        filter to use `self.Q`.
*/
    int tempX[2];
    int tempP[2][2];
    // X = FX + Bu
    // X = dot(F, X) + dot(B, u)
    tempX[0] = F[0][0]*X[0] + F[0][1]*X[1] + B[0]*u;
    tempX[1] = F[1][0]*X[0] + F[1][1]*X[1] + B[0]*u;

    // P = FPF' + Q
    // F' = F transposed
    // P = dot(dot(F, P), F') + Q

    // dot(F, P)
    tempP[0][0] = F[0][0] * P[0][0] + F[0][1] * P[1][0];
    tempP[0][1] = F[0][0] * P[0][1] + F[0][1] * P[1][1];
    tempP[1][0] = F[1][0] * P[0][0] + F[1][0] * P[1][0];
    tempP[1][1] = F[1][0] * P[0][1] + F[1][1] * P[1][1];

    // dot(dot(F, P), F')
    tempP[0][0] = tempP[0][0] * F[0][0] + tempP[0][1] * F[0][1];
    tempP[0][1] = tempP[0][0] * F[1][0] + tempP[0][1] * F[1][1];
    tempP[1][0] = tempP[1][0] * F[0][0] + tempP[1][0] * F[0][1];
    tempP[1][1] = tempP[1][0] * F[1][0] + tempP[1][1] * F[1][1];

    tempP[0][0] += Q[0][0];

    // self.x_prior = self.x.copy()
    // self.P_prior = self.P.copy()



}

void kalmanFilter::update(){
/*
    Function: update
    ----------
    Add a new measurement (z) to the Kalman filter.
    If z is None, nothing is computed. However, x_post and P_post are
    updated with the prior (x_prior, P_prior), and self.z is set to None.
    Parameters
    ----------
    z : (dim_z, 1): array_like
        measurement for this update. z can be a scalar if dim_z is 1,
        otherwise it must be convertible to a column vector.
        If you pass in a value of H, z must be a column vector the
        of the correct size.
    kalFil.R : np.array, scalar, or None
        Optionally provide R to override the measurement noise for this
        one call, otherwise  self.R will be used.
    kalFil.H : np.array, or None
        Optionally provide H to override the measurement function for this
        one call, otherwise self.H will be used.
*/
}

void kalmanFilter::writePredicted(){
/*
    Writes altitude and velocity predictions to the textfile 'outputData.txt'

*/
    std::fstream outFile;
    outFile.open("outputData.txt", std::ios_base::app);
    outFile << "\n" << X[0] << ",\t\t\t" << X[1];
    outFile.close();
}