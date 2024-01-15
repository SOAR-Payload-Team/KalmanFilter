# KalmanFilter
Filter class for general implementation. No control input support.

"Kalman.h" -> General Multivariate Kalman filter class. Works only for systems where linear approximation is efffective. 
"Unscented.h" -> General unscented Kalman filter class. Works for non-linear systems.

"test.cpp" contains baisic implementations of both classes as examples.

Implementation notes for Kalman class:
 * Dimensions for filter MUST be set either with contructor or init().
 * init() must be called before any calls to update().
 * See comments on init() function for important details.

Implementation notes for KalmanU class:
 * dim_x and dim_z MUST be defined with the constructor
 * Functions predict() and observe() need to be overwritten with non-linear system dynamics. Linear implementations are present if needed. See notes above each funtion prototype for more details.
