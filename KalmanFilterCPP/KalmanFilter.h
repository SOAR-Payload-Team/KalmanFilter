/*
 * File name: kalmanFilter.h
 *
 * Contains: Function prototypes
 *
 * Author: Findlay Brown
 * Language: C++
 */

// include path for eigen
// #include <eigen-path>
#include <C:\Users\calga\OneDrive\Documents\GitHub\KalmanFilter\Eigen\Eigen\Dense>

using namespace Eigen;

/*
 *	Matrix Dimension must be:
 *
 *	A: n x n
 *	B: n x m
 *	H: n x n
 *	Q: n x n
 *	R: n x n
 *	I: n x n
 *	X: n x 1
 *	U: m x 1
 *	Z: n x 1
 *	P: n x n
 *	K: n x n
 *
 *  X: dim_x * 1
 *  A: dim_x * dim_x
 *  P: dim_x * dim_x
 *  R: dim_z * dim_z
 *  Q: dim_x * dim_x
 *  H: dim_z * dim_x
 *  F: dim_x * dim_x
 *  K: dim_x * dim_z
 *
 *  z: dim_z * 1
 *
 */

class KalmanFilter
{

public:
    /* Problem Dimension */
    int dim_x; // State vector dimension
    int dim_z;
    int dim_u; // Control vector (input) dimension (if there is not input, set to zero)

    /* Fixed Matrix */
    MatrixXf F; // State Transition matrix
    MatrixXf B; // Control-input matrix
    MatrixXf H; // Mesaurement function
    MatrixXf Q; // Process Noise Covariance matrix
    MatrixXf R; // Measurement Noise Covariance matrix
    MatrixXf I; // Identity matrix

    /* Variable Matrix */
    VectorXf X; // Current State vector
    MatrixXf P; // Current State Covariance matrix
    MatrixXf K; // Kalman Gain matrix

    /* Inizial Value */
    VectorXf X0; // Initial State vector
    MatrixXf P0; // Initial State Covariance matrix

    /*
     * Constructor
     * dim_x: number of state variables
     *      For example if tracking altitude and velocity
     *      in 2 dimensions, dim_x would be 4
     * dim_z: number of measurement inputs
     * dim_u: control vector dimension (if there is no input, set to zero)
     */
    KalmanFilter(int dim_x, int dim_z, int dim_u);

    /*
     * Set Fixed Matrix (NO INPUT)
     * Saves F, H, Q, and R matrices
     */
    void setFixed(MatrixXf F, MatrixXf H, MatrixXf Q, MatrixXf R);

    /*
     * Set Fixed Matrix (WITH INPUT)
     * Saves F, H, Q, R, and B matrices
     */
    void setFixed(MatrixXf F, MatrixXf B, MatrixXf H, MatrixXf Q, MatrixXf R);

    /* Set Initial Value */
    void setInitial(VectorXf X0, MatrixXf P0);

    /* Do prediction (NO INPUT) */
    void predict(void);

    /* Do prediction (INPUT) */
    void predict(VectorXf U);

    /* Do correction */
    void correct(VectorXf Z);
};
