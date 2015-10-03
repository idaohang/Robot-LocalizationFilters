#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

/*********************************************************************************************
 *  url: https://www.udacity.com/wiki/cs373/kalman-filter-matrices                           * 
 *  url: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/#mjx-eqn-matrixgain    *
 *********************************************************************************************/

using namespace Eigen;

// N = dimension, M = number of measurements
template<unsigned int N, unsigned int M>
class KalmanFilter
{
    typedef Eigen::Matrix< double , N , 1> VectorN;
    typedef Eigen::Matrix< double , M , 1> VectorM;
    typedef Eigen::Matrix< double , N , N> MatrixNN;
    typedef Eigen::Matrix< double , M , N> MatrixMN;
    typedef Eigen::Matrix< double , N , M> MatrixNM;
    typedef Eigen::Matrix< double , M , M> MatrixMM;
public:
    // constructor and destructor
    KalmanFilter() {}
    virtual ~KalmanFilter() {}

    // parameters setup
    void SetState(VectorN vec) 
    {
        assert(vec.size() == N && "N of state variable is not correct!");
        this->x = vec; 
    }
    void SetMotionVector(VectorN vec)
    {
        assert(vec.size() == N && "N of motion vector is not correct!");
        this->u = vec; 
    }
    void SetTransitionMatrix(MatrixNN mat)
    {
        assert(mat.rows() == N && "number of rows in transition matrix mismatch!");
        assert(mat.cols() == N && "number of cols in transition matrix mismatch!");
        this->F = mat; 
        this->Ftransp = mat.transpose();
    }
    void SetUncertaintyCovariance(MatrixNN mat)
    {
        assert(mat.rows() == N && "number of rows in covariance matrix mismatch!");
        assert(mat.cols() == N && "number of cols in covariance matrix mismatch!");
        this->P = mat; 
    }
    void SetMeasureMatrix(MatrixMN mat)
    { 
        assert(mat.rows() == M && mat.cols() == N && "Measurement extraction matrix dimension mismatch!");
        this->H = mat; 
        this->Htransp = mat.transpose();
    }
    void SetMeasureNoise(MatrixMM mat)
    {
        assert(mat.rows() == M && mat.cols() == M && "Measurement noise covariance dimension mismatch!");
        this->R = mat; 
    }
    void Update(VectorM z)
    {
        // update the current state by state transition matrix
        x = F * x + u;
        P = F * P * Ftransp;
        VectorM y = z - H * x;
        MatrixMM S = H * P * Htransp + R;
        MatrixNM K = P * Htransp * S.inverse();
        x = x + K * y;
        P = (MatrixNN::Identity() - K * H) * P;
    }
    VectorN GetCurrentState() const { return x; }
private:
    VectorN x;     // state variable
    VectorN u;     // motion vector

    MatrixNN F, Ftransp;     // state transition function
    MatrixNN P;              // state uncertainty covariance
    MatrixMN H;              // measurement extraction matrix
    MatrixNM Htransp;        //measurement extraction matrix transpose
    MatrixMM R;              // measurement noise
};

#endif /* end of include guard: KALMANFILTER_H */
