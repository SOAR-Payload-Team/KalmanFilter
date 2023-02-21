/*
 * File name: kalmanFilter.h
 *
 * Contains: Function prototypes 
 *             
 * Author: Findlay Brown
 * Language: C++
 */
#include "Eigen\Dense"

#ifndef KALMANFILTER_H
#define KALMANFILTERC_H
using namespace Eigen;
class kalmanFilter
{
    public:
        kalmanFilter(){
            F.fill(0.0);
            P.fill(0.0);
            Q.fill(0.0);
        };
        kalmanFilter(double initalX, double dt){
            this->dt = dt;
            F.fill(0.0);
            P.fill(0.0);
            Q.fill(0.0);
            P_prior.fill(0.0);
            X(0)=initalX;
        };

        /*
         * Predict next state (prior) using the Kalman filter state propagation equations.
         * 
         * Parameters:
         * ----------
         * u : double, Control value: acceleration
         */
        void predict(double u);

        // Add a new measurement (z) to the Kalman filter.
        void update(double z);

        // Writes data to a text file
        void writePrev();

        // Set time between measurements
        void setTimeInterval(double dt){this->dt=dt;};

        // Get time between measurements
        double getTI(){return dt;};

        /* 
         * Current state estimate
         * {Altitude, Velocity} 
         * 1x2 matrix
         */
        Vector2d X{0.0, 0.0};
        

        /* 
         * State Transition matrix
         * 2x2 matrix
         */
        Matrix2d F;
        

        /* 
         * Measurement noise variance
         */
        double R;

        /*
         * Measurement function
         * 1x2 matrix
         */
        Vector2d H{0.0, 0.0};
        

        /* 
         * Current state covariance matrix
         * 2x2 matrix
         */
        Matrix2d P;

        
        /*
         * Process noise covariance matrix
         * 2x2 matrix
         */
        Matrix2d Q;


        /*
         * Control-input matrix
         * 1x2 matrix
         */
        Vector2d B{0.0, 0.0};


        /*
         * Prior state estimate
         * {Altitude, Velocity}
         * 1x2 matrix
         */
        Vector2d X_prior{0.0, 0.0};

        /*
         * Prior state covariance matrix
         * 1x2 matrix
         */
        Matrix2d P_prior;


        private:
            double dt;


};



#endif