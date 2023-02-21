/*
 * File name: main.cpp
 *
 * Header File  
 * 
 * Author: Findlay Brown
 * Date: January 28, 2023
 * Language: C++
 */

#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Eigen/Dense"
#include "kalmanFilter.h"

#define intialX 7000.0  // Set inital altitude
#define dt 0.02         // Set time between measurements

#define r_std 0.003         // Measurement standard deviation
#define q_std 1.0           // Noise standard deviation
#define r_var pow(r_std,2)  // Measurement noise variance
#define q_var pow(q_std,2)  // Process noise variance

void initialize(struct kalmanFilter& kalFil);

int main(int argc, char const *argv[])
{
    kalmanFilter kalFil = kalmanFilter(intialX);
    kalFil.setTimeInterval(dt);
    //printf("%.2f, %.2f",kalFil.X[0],kalFil.X[1]);
    printf("%.2f",kalFil.getTI());
    //initialize(kalFil);
    
    double u = 0.0;
    
    kalFil.writePredicted();



    return 0;
}

void initialize(struct kalmanFilter& kalFil ){
    kalFil.Q[0][0] = 0.25*pow(dt,4)*q_var;
    kalFil.Q[0][1] = 0.5*pow(dt,3)*q_var;
    kalFil.Q[1][0] = 0.50*pow(dt,3)*q_var;
    kalFil.Q[1][1] = 1.0*pow(dt,2)*q_var;
}
/*
// Initalize all parameters of kalmanFilter
void initialize(struct kalmanFilter& kalFil){
    // Initial state estimate        
    kalFil.X[0] = intialX; // Altitude
    kalFil.X[1] = 0;       // Velocity
    //printf("X: %.2f, %.2f\n",kalFil.X[0],kalFil.X[1]);

    // Initalize State Transition matrix
    kalFil.F[0][0] = 1;
    kalFil.F[0][1] = dt;
    kalFil.F[1][0] = 0;
    kalFil.F[1][1] = 1;
    //printf("F: %.2f, %.2f\n   %.2f, %.2f\n", kalFil.F[0][0],kalFil.F[0][1],kalFil.F[1][0],kalFil.F[1][1]); 
    
    // Measurement noise variance
    kalFil.R = r_var;
    //printf("R:  %.2f\n", kalFil.R);                          

    // Measurement function
    kalFil.H[0] = 1.0; 
    kalFil.H[1] = 0;       
    //printf("H: %.2f, %.2f\n", kalFil.X[0],kalFil.X[1]);

    // Current state covariance matrix
    kalFil.P[0][0] = 1;
    kalFil.P[0][1] = dt;
    kalFil.P[1][0] = 0;
    kalFil.P[1][1] = 1;
    //printf("P: %.2f, %.2f\n   %.2f, %.2f\n", kalFil.P[0][0],kalFil.P[0][1],kalFil.P[1][0],kalFil.P[1][1]);

    // Process noise covariance matrix
    kalFil.Q[0][0] = 0.25*pow(dt,4)*q_var;
    kalFil.Q[0][1] = 0.5*pow(dt,3)*q_var;
    kalFil.Q[1][0] = 0.50*pow(dt,3)*q_var;
    kalFil.Q[1][1] = 1.0*pow(dt,2)*q_var;
    //printf("Q: %.2f, %.2f\n   %.2f, %.2f\n", kalFil.Q[0][0],kalFil.Q[0][1],kalFil.Q[1][0],kalFil.Q[1][1]);


    kalFil.B[0] = 0.5*pow(dt,2);
    kalFil.B[1] = dt;
    //printf("B: %.2f, %.2f\n", kalFil.B[0],kalFil.B[1]);

}*/
