/*
 * File name: Kalman.cpp
 *
 * Contains: Functions for kalman filter
 *
 * Authors: Findlay Brown and Aiden Ballard
 * Language: C++
 */

#include "Kalman.h"
using namespace std;

/*
 * Constructor
 * dim_x: number of state variables
 *      For example if tracking altitude and velocity
 *      in 2 dimensions, dim_x would be 4
 * dim_z: number of measurement inputs
 * dim_u: control vector dimension (if there is no input, set to zero)
 */
Kalman::Kalman(int dim_x, int dim_z, int dim_u)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    this->dim_u = dim_u;
}

/*
 * Set Fixed Matrix
 * Must be run before first update() call
 * Saves F, H, Q, R, and B matrices
 * Set dimensions of all matrices passed to function accoring to guide in Kalman.h
 */
void Kalman::setFixed(MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Q_in, MatrixXf &B_in)
{
    F = &F_in;
    H = &H_in;
    Q = &Q_in;
    B = &B_in;
    I = I.Identity(dim_x, dim_x);
    K.resize(dim_x, dim_z);
    K.Constant(0);
}

/* Set Initial state Matrices + point to control matrices that update with measurements
 * Must also be run before first update() call*/
void Kalman::setControl(VectorXf &Z_in, MatrixXf &R_in, double X0, double P0)
{
    Z = &Z_in;
    R = &R_in;
    Xpred.resize(dim_x);
    Xpred.Constant(X0);
    Ppred.resize(dim_x, dim_x);
    Ppred.Constant(P0);
}

// /* Do prediction based off physical system */
// void Kalman::predict()
// {
// }

// /* Correct the prediction, using mesaurement*/
// void Kalman::correct(VectorXf Z)
// {
//     K = (P * H->transpose()) * (*H * P * H->transpose() + *R).inverse();

//     X = X + K * (Z - *H * X);

//     P = (I - K * *H) * P;

//     X0 = X;
//     P0 = P;
// }

void Kalman::update()
{
    K = (Ppred * H->transpose()).cross((*H * Ppred * H->transpose() + *R).inverse);
    X = Xpred + K.cross(Z)

}