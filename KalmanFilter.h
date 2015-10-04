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
    /**
     * State variable priori (N x 1)
     * keep record of the best current belief state
     * */
    void SetState(VectorN vec) 
    {
        this->x = vec; 
    }
    /**
     * Move vector
     * */
    void SetMoveVector(VectorN vec)
    {
        this->u = vec; 
    }
    /**
     * Transition function in matrix form (N x N)
     * calculate the belief propagation to next state
     * */
    void SetStateTransition(MatrixNN mat)
    {
        this->F = mat; 
        this->Ftransp = mat.transpose();
    }
    /**
     * Covariance matrix of the state transition (N x N)
     * */
    void SetStateCovariance(MatrixNN mat)
    {
        this->P = mat; 
    }
    /**
     * Measure extraction matrix (M x N)
     * used to extract state features from measurement input
     * */
    void SetMeasureExtraction(MatrixMN mat)
    { 
        this->H = mat; 
        this->Htransp = mat.transpose();
    }
    /**
     * Covariance matrix of the measurement, noise (M x M)
     * */
    void SetMeasureCovariance(MatrixMM mat)
    {
        this->R = mat; 
    }
    /**
     * Calculate the update from model, and correct the output
     * using the measurement inputs
     * z: measurement (M x 1)
     * */
    void Update(VectorM z)
    {
        // update the current state by state transition matrix
        x = F * x + u;
        P = F * P * Ftransp;
        // compute difference between measurement and prediction
        VectorM y = z - H * x;
        MatrixMM S = H * P * Htransp + R;
        // calculate kalman gain
        MatrixNM K = P * Htransp * S.inverse();
        // update state variable and covariance
        x = x + K * y;
        P = (MatrixNN::Identity() - K * H) * P;
    }
    /**
     * Get the current state variable
     * */
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
