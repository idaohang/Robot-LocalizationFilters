#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>

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
    KalmanFilter();
    virtual ~KalmanFilter();

    // parameters setup
    void SetState(Vector vec);
    void SetMotionVector(Vector vec);
    void SetTransitionMatrix(Matrix mat);
    void SetUncertaintyCovariance(Matrix mat);
    void SetMeasureMatrix(Matrix mat);
    void SetMeasureNoise(Matrix mat);

    // predict and update
    void Predict();
    void Update(Vector z);

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
