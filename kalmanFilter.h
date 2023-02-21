/*
 * File name: kalmanFilter.h
 *
 * Contains: Function prototypes 
 *             
 * Author: Findlay Brown
 * Language: C++
 */


#ifndef KALMANFILTER_H
#define KALMANFILTERC_H

class kalmanFilter
{
    public:
        kalmanFilter() = default;
        kalmanFilter(double initalX){X[0]=initalX;};

        /*
         * Predict next state (prior) using the Kalman filter state propagation equations.
         * 
         * Parameters:
         * ----------
         * u : double, Control value: acceleration
         */
        void predict(double u);

        // Add a new measurement (z) to the Kalman filter.
        void update();

        // Writes data to a text file
        void writePredicted();

        // Set time between measurements
        void setTimeInterval(double dt){this->dt=dt;};

        // Get time between measurements
        double getTI(){
            return dt;
        };

        /* 
         * Current state estimate
         * {Altitude, Velocity} 
         * 1x2 matrix
         */
        double X[2] = {0,0};

        /* 
         * State Transition matrix
         * 2x2 matrix
         */
        double F[2][2] = {{0,0},{0,0}};

        /* 
         * Measurement noise variance
         */
        double R;

        /*
         * Measurement function
         * 1x2 matrix
         */
        double H[2] = {0,0};

        /* 
         * Current state covariance matrix
         * 2x2 matrix
         */
        double P[2][2] = {{0,0},{0,0}};
        
        /*
         * Process noise covariance matrix
         * 2x2 matrix
         */
        double Q[2][2] = {{0.0,0.0},{0.0,0.0}};
        /*
         * Control-input matrix
         * 1x2 matrix
         */
        double B[2] = {0,0};

        private:
            double dt;


};



#endif