#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

/*********************************************************************************************
 *  url: https://www.udacity.com/wiki/cs373/kalman-filter-matrices                           * 
 *  url: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/#mjx-eqn-matrixgain    *
 *********************************************************************************************/

template<unsigned int dimension>
class KalmanFilter
{
    typedef Eigen::Matrix< double , dimension , 1> Vector;
    typedef Eigen::Matrix< double , dimension , dimension> Matrix;
public:
    // constructor and destructor
    KalmanFilter() {}
    virtual ~KalmanFilter() {}

    // parameters setup
    void SetState(Vector vec) 
    {
        assert(vec.size() == dimension && "dimension of state variable is not correct!");
        this->x = vec; 
    }
    void SetMotionVector(Vector vec)
    {
        assert(vec.size() == dimension && "dimension of motion vector is not correct!");
        this->u = vec; 
    }
    void SetTransitionMatrix(Matrix mat)
    {
        assert(mat.rows() == dimension && "number of rows in transition matrix is not correct!");
        assert(mat.cols() == dimension && "number of cols in transition matrix is not correct!");
        this->F = mat; 
        this->Ftransp = mat.transpose();
    }
    void SetUncertaintyCovariance(Matrix mat)
    {
        assert(mat.rows() == dimension && "number of rows in covariance matrix is not correct!");
        assert(mat.cols() == dimension && "number of cols in covariance matrix is not correct!");
        this->P = mat; 
    }
    void SetMeasureMatrix(Matrix mat)
    { 
        // need to check dimension
        this->H = mat; 
        this->Htransp = mat.transpose();
    }
    void SetMeasureNoise(Matrix mat)
    {
        // need to check dimension
        this->R = mat; 
    }
    // predict and update
    void Predict()
    {
        // update the current state by state transition matrix
        x = F * x + u;
        P = F * P * Ftransp;
#if DEBUG
        std::cout << "PREDICT: " << x(0) << ", " << x(1) << std::endl;
#endif
    }
    void Update(Vector z)
    {
#if DEBUG
        auto tmp1 = H * x;
        // std::cout << "H: " << H << std::endl;
        std::cout << "pre x: " << x.transpose() << std::endl;
        std::cout << "H*x: " << tmp1.transpose() << std::endl;
#endif
        Vector y = z - H * x;
#if DEBUG
        std::cout << "measure: " << z.transpose() << std::endl;
        std::cout << "y: " << y.transpose() << std::endl;
#endif
        Matrix S = H * P * Htransp + R;
        Matrix K = P * Htransp * S.inverse();
        x = x + K * y;
        P = (Matrix::Identity() - K * H) * P;
#if DEBUG
        std::cout << "after x: " << x.transpose() << std::endl;
        std::cout << "-----------------------" << std::endl;
#endif
    }
    Vector GetCurrentState() const { return x; }
private:
    Vector x;     // state variable
    Vector u;     // motion vector

    Matrix F, Ftransp;     // state transition function
    Matrix P;              // state uncertainty covariance
    Matrix H, Htransp;     // measurement matrix
    Matrix R;              // measurement noise
};

#endif /* end of include guard: KALMANFILTER_H */
