/*
 * File name: kalmanFilter.cpp
 *
 * Contains: Functions for kalman filter
 *
 * Author: Findlay Brown
 * Language: C++
 */

#include <iostream>
#include "KalmanFilter.h"
using namespace std;

/*
 * Constructor
 * dim_x: number of state variables
 *      For example if tracking altitude and velocity
 *      in 2 dimensions, dim_x would be 4
 * dim_z: number of measurement inputs
 * dim_u: control vector dimension (if there is no input, set to zero)
 */
KalmanFilter::KalmanFilter(int dim_x, int dim_z, int dim_u)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    this->dim_u = dim_u;
}

/*
 * Set Fixed Matrix (NO INPUT)
 * Saves F, H, Q, and R matrices
 */
void KalmanFilter::setFixed(MatrixXf F, MatrixXf H, MatrixXf Q, MatrixXf R)
{
    this->F = F;
    this->H = H;
    this->Q = Q;
    this->R = R;
    I = I.Identity(dim_x, dim_x);
}

/*
 * Set Fixed Matrix (WITH INPUT)
 * Saves F, H, Q, R, and B matrices
 */
void KalmanFilter::setFixed(MatrixXf F, MatrixXf H, MatrixXf Q, MatrixXf R, MatrixXf B)
{
    this->F = F;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->I = I.Identity(dim_x, dim_x);
}

/* Set Initial Matrix */
void KalmanFilter::setInitial(VectorXf X0, MatrixXf P0)
{
    this->X0 = X0;
    this->P0 = P0;
}

/* Do prediction based of physical system (No external input)
 */
void KalmanFilter::predict(void)
{
    X = (F * X0);
    P = (F * P0 * F.transpose()) + Q;
}

/* Do prediction based of physical system (with external input)
 * U: Control vector
 */
void KalmanFilter::predict(VectorXf U)
{
    X = (F * X0) + (B * U);
    P = (F * P0 * F.transpose()) + Q;
}

/* Correct the prediction, using mesaurement
 *  Z: measure vector
 */
void KalmanFilter::correct(VectorXf Z)
{
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();

    X = X + K * (Z - H * X);

    P = (I - K * H) * P;

    X0 = X;
    P0 = P;
}
