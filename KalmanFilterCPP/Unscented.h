/*
Class for unscented Kalman filters with no control input support.
See notes on pure virtual functions predict() and observe() which MUST be overwritten for implementation
See Kalman.h for additional information.
*/

#ifndef KalmanU_h
#define KalmanU_h

#include "Kalman.h"

class KalmanU : public Kalman
{
public:
    /*
     * Constructor
     * dim_x: number of state variables (size of state vector X)
     * dim_z: number of measurement inputs (size of measurement vector Z)
     */
    KalmanU(int dim_x, int dim_z);

    /*Initalize the Kalman filter.
     */
    virtual void init();

    /*Initialize the Kalman filter.
     */
    // virtual void init(int dim_x, int dim_z);

    /*Update the filter with new measurements
    Note that this reads the variables Z and R passed to init() for new data.*/
    virtual void update();

    /*Set K for filter tuning.
    K is by default set to 3-N, the value generally best for gaussian-distributed data */
    void setK(float k) { this->k = k; };

protected:

    /*Takes Measurement Z and calculates state X besed off measurements. Must be overwritten with 
    non-linear observation equations. Use H matrix passed to init() function if observation equations are linear.*/
    virtual VectorXf observe(VectorXf &Z) = 0;

    /*Takes state vector X and calculates state based off of previous state, to be stored in Xpred.  Must be overwritten with
    non-linear system dynamics. Use F matrix passed to init() if system dynamics are linear 
    X will be a dim_x X 1 matrix, output must also be a dim_x X 1 matrix.*/
    virtual MatrixXf predict(MatrixXf X) = 0;

    /*Function to perform the unscented transform*/
    void uTransform();

    float k; // Tuning parameter for linearization.
    MatrixXf sPoints; //to be assigned an array of sigma points at runtime
    MatrixXf *sigma; //Decomposed matrix used for finding sigma points
    VectorXf W; //weight matrix / vector for sigma points
    VectorXf *W1; //Matrix for filling W
    float *w0, *w1; //weights for W matrix
    MatrixXf *projError; //Matrix for calculating projected covariance

};
#endif
