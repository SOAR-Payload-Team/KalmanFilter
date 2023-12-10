/*
 * File name: KalmanFilterTest.cpp
 *
 * Contains: Testing for Kalman Filter
 *
 * Author: Findlay Brown
 * Language: C++
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Kalman.h"

using namespace std;

int main(int argc, char const *argv[])
{
    std::fstream outFileA("KalmanFilterOuput.txt");
    outFileA.clear();

    std::ifstream inFileAlti("fixedAltitudeData.txt"); // Altitude data input file

    std::ifstream inFileAccel("fixedAccelerationData2.txt"); // Acceleration data input file

    const float dt = 0.02; // time between measurements

    /* Set Matrix and Vector for Kalman Filter: */
    MatrixXf F(2, 2);
    F << 1, dt, 0, 1;
    MatrixXf B(2, 1);
    B << (0.5 * (dt * dt)), dt;
    MatrixXf H(1, 2);
    H << 1, 0;
    MatrixXf Q(2, 2);
    Q << 0.25 * pow(dt, 4), 0.5 * pow(dt, 3),
        0.50 * pow(dt, 3), 1.0 * pow(dt, 2);
    MatrixXf R(1, 1);
    R << (0.003 * 0.003);
    VectorXf X0(2);
    X0 << 7000, 0;
    Eigen::MatrixXf P0(2, 2);
    P0 << (0.1 * 0.1), 0, 0, (0.03 * 0.03);

    /* Create The Filter */
    Kalman filter1(2, 1, 1);

    /* Initialize the Filter*/
    filter1.setFixed(F, H, Q, R, B);
    filter1.setInitial(X0, P0);

    /* Create measure vector, and store measure value */
    VectorXf Z(1);
    VectorXf U(1);

    float readAlt;
    float readAcc;
    while (true)
    {
        inFileAccel >> readAcc;
        inFileAlti >> readAlt;
        if (inFileAccel.eof() || inFileAlti.eof())
        {
            break;
        }

        float Vi = filter1.X0(1);
        float Z0 = readAlt;
        float Z1 = (readAcc * 9.807 / 1000);

        U << Z1;

        /* This loop simulate the measure/prediction process */

        filter1.predict(U); // Predict phase
        Z << Z0;            //, (filter1.X(1) + Z1 * dt);
        filter1.correct(Z); // Correction phase

        // cout << filter1.X(0) << endl;
        outFileA << filter1.X(0) << endl;
    }
    outFileA.close();
    inFileAccel.close();
    inFileAlti.close();
    return 0;
}