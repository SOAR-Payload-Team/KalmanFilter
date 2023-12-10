/*
 * File name: Kalman.h
 *
 * Contains: Function prototypes
 *
 * Authors: Findlay Brown and Aiden Ballard
 * Language: C++
 */

// include path for eigen
// #include <eigen-path>
#include "eigen-3.4.0\\eigen-3.4.0\\Eigen\\Eigen"
#ifndef Kalman_h
#define Kalman_h

using namespace Eigen;

/*
 *	Matrix Dimension must be:
 * 
 * 
 *
 *
 */

class Kalman
{
    /*
     * Constructor
     * dim_x: number of state variables
     *      For example if tracking altitude and velocity
     *      in 2 dimensions, dim_x would be 4
     * dim_z: number of measurement inputs
     * dim_u: control vector dimension (if there is no input, set to zero)
     */
    Kalman(int dim_x, int dim_z, int dim_u);

    /*
     * Set Fixed Matrix (NO INPUT)
     * Saves F, H, Q, and R matrices
     */
    virtual void setFixed(MatrixXf* F, MatrixXf* H, MatrixXf* Q, MatrixXf* R);

    /*
     * Set Fixed Matrix (WITH INPUT)
     * Saves F, H, Q, R, and B matrices
     */
    virtual void setFixed(MatrixXf &F_in, MatrixXf &H_in, MatrixXf &Q_in, MatrixXf &R_in, MatrixXf &B_in);

    /* Set Initial Value */
    virtual void setControl(VectorXf &X_in, MatrixXf &P_in, double X0, double P0);

    /* Do prediction*/
    void predict(void);

    /* Do correction */
    void correct(VectorXf Z);    
    
    /* Problem Dimension */
    int dim_x; // State vector dimension
    int dim_z;
    int dim_u; // Control vector (input) dimension (if there is not input, set to zero)

    /* Fixed Matrices */
    MatrixXf *F; // State Transition matrix
    MatrixXf *B; // Control-input matrix
    MatrixXf *H; // Mesaurement function
    MatrixXf *Q; // Process Noise Covariance matrix
    MatrixXf *R; // Measurement Noise Covariance matrix
    MatrixXf I;  // Identity matrix

    /* Variable Matrices */
    VectorXf X; // Current State vector
    MatrixXf P; // Current State Covariance matrix
    VectorXf Xpred; //Predicted state matrix
    MatrixXf Ppred; //Predicted Coviariance matrix
    MatrixXf K; // Kalman Gain matrix

    /* Input Matrices */
    VectorXf Z; //measurement Matrix
    MatrixXf R; // measurement Covariance matrix

};
#endif