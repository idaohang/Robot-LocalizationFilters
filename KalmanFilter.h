#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

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
        this->H = mat; 
    }
    // predict and update
    void Predict()
    {
        // update the current state by state transition matrix
        x = F * x + u;
        P = F * P * Ftransp;
    }
    void Update(Vector z)
    {
        Vector y = z - H * x;
        Matrix S = H * P * Htransp + R;
        Matrix K = P * Htransp * S.inverse();
        x = x + K * y;
        P = (Matrix::Identity() - K * H) * P;
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
