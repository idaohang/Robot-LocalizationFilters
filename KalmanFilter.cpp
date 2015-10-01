#include "KalmanFilter.h"

template<unsigned int dimension>
KalmanFilter<dimension>::KalmanFilter()
{ 
}

template<unsigned int dimension>
KalmanFilter<dimension>::~KalmanFilter() 
{}

/**
 * Set initial state veriable
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetState(Vector vec) 
{ 
    assert(vec.size() == dimension && "dimension of state variable is not correct!");
    this->x = vec; 
}

/**
 * Set initial state veriable
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetMotionVector(Vector vec) 
{ 
    assert(vec.size() == dimension && "dimension of motion vector is not correct!");
    this->u = vec; 
}

/**
 * Set state transition matrix
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetTransitionMatrix(Matrix mat) 
{ 
    assert(mat.rows() == dimension && "number of rows in transition matrix is not correct!");
    assert(mat.cols() == dimension && "number of cols in transition matrix is not correct!");
    this->F = mat; 
    this->Ftransp = mat.transpose();
}

/**
 *
 * Set state transition covariance
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetUncertaintyCovariance(Matrix mat) 
{ 
    assert(mat.rows() == dimension && "number of rows in covariance matrix is not correct!");
    assert(mat.cols() == dimension && "number of cols in covariance matrix is not correct!");
    this->P = mat; 
}

/**
 * Set measurement matrix
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetMeasureMatrix(Matrix mat) 
{ 
    this->H = mat; 
    this->Htransp = mat.transpose();
}

/**
 * Set measurement noise matrix
 */
template<unsigned int dimension>
void KalmanFilter<dimension>::SetMeasureNoise(Matrix mat) 
{ 
    this->H = mat; 
}

/**
 * Predict the value at the next time stamp
 * before incorporating further evidence/
 * measurement using the current best estiamte
 * */
template<unsigned int dimension>
void KalmanFilter<dimension>::Predict()
{
    // update the current state by state transition matrix
    x = F * x + u;
    P = F * P * Ftransp;
}

/**
 * Feed the new evidence/measurement into 
 * calculation to obtain the new best estimate
 * */
template<unsigned int dimension>
void KalmanFilter<dimension>::Update(Vector z)
{
    Vector y = z - H * x;
    Matrix S = H * P * Htransp + R;
    Matrix K = P * Htransp * S.inverse();
    x = x + K * y;
    P = (Matrix::Identity() - K * H) * P;
}
