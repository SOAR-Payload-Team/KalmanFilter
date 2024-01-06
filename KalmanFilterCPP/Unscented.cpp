/*
Implementations for unscented kalman filter
See Unscented.h for function help
*/

#include "Unscented.h"

KalmanU::KalmanU(int dim_x, int dim_z)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
}

void KalmanU::init()
{

    // Calculate weights
    k = 3 - dim_x;
    w0 = new float;
    *w0 = k / (dim_x + k);
    w1 = new float;
    *w1 = 0.5 * (dim_x + k);
    W.Zero(dim_x);
    W1 = new VectorXf;
    W1->Constant(dim_x - 1, *w1);
    W << *w0, *W1;
    delete w0;
    delete w1;
    delete W1;
}

void KalmanU::uTransform()
{
    // Generate sigma points
    sPoints.setZero((2 * dim_x) + 1, dim_x);
    sPoints.col(0) = X;
    sigma = new MatrixXf;
    *sigma = ((dim_x + k) * P).llt().matrixL();
    for (int i = 1; i <= dim_x; i++)
    {
        sPoints.col(i) = X + sigma->col(i - 1);
    }
    for (int i = dim_x; i <= (2 * dim_x); i++)
    {
        sPoints.col(i) = X - sigma->col(i - dim_x - 1);
    }
    delete sigma;

    // Propigate points
    for (int i = 0; i < (2 * dim_x) + 1; i++)
    {
        sPoints.col(i) = predict(sPoints.col(i));
    }

    // Calculate weighted mean for points
    Xpred = sPoints * W;
    projError = new MatrixXf;
    projError->resize(dim_x, (2 * dim_x) + 1);
    for (int i = 0; i < dim_x; i++)
    {
        projError->row(i) = (sPoints.row(i).array() - Xpred.row(i).value()).matrix();
    }
    Ppred = *projError * W.asDiagonal() * projError->transpose();
    delete projError;    
}
