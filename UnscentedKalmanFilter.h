#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues> 
#include <iostream>
#include <cmath>

// http://www.cslu.ogi.edu/nsel/ukf/node6.html

template<unsigned int dimension>
class UnscentedKalmanFilter
{
    typedef Eigen::Matrix< double , dimension , 1> Vector;
    typedef Eigen::Matrix< double , dimension , dimension> Matrix;
public:
    // constructor and destructor
    UnscentedKalmanFilter() {}
    virtual ~UnscentedKalmanFilter() {}

    // parameters setup
    void SetState(Vector vec) 
    {
        assert(vec.size() == dimension && "dimension of state variable is not correct!");
        this->x = vec; 
    }
    void SetTransitionFunction(Vector (*func)(Vector))
    {
        this->F = func;
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
    void SetUnscentedParameters(double kappa, double alpha, double beta = 2)
    {
        this->alpha = alpha;
        this->beta = beta;
        this->kappa = kappa;
        this->lambda = alpha * alpha * (dimension + kappa) - dimension;

        double factor1 = lambda / (dimension + lambda);
        double factor2 = lambda / (2 * (dimension + lambda));

        /* set up the weights coefficients */
        m_weights << factor1;
        c_weights << factor1 + (1 - alpha * alpha + beta);
        for (int i = 0; i < 2 * dimension + 1; ++i)
        {
            m_weights << factor2;
            c_weights << factor2;
        }
    }
    // predict and update
    void Predict()
    {
        /* 1. compute square root covariance */
        Eigen::EigenSolver<Matrix> es(P);
        auto V = es.pseudoEigenvectors();
        auto D = es.pseudoEigenvalueMatrix();
        // don't know better way to do element-wise sqrt
        for (int i = 0; i < dimension; ++i)
            D(i, i) = std::sqrt(D(i, i));
        auto S = V * D * V.inverse();

        /* 2. set up the transitioned vectors */
        // use 2 * dimension + 1 sigma points to find a Gaussian approximation
        Vector sigma_points[2 * dimension + 1];
        // set up the first: mean
        sigma_points[0] = (*F)(x);
        // set up the 1 to dimension
        for (int i = 1; i < dimension + 1; ++i) 
        {
            Vector t = x + std::sqrt(dimension + lambda) * S.col(i);
            sigma_points[i] = (*F)(t);
        }
        // set up the 1 to dimension
        for (int i = dimension + 1; i < 2 * dimension + 1; ++i) 
        {
            Vector t = x - std::sqrt(dimension + lambda) * S.col(i - dimension);
            sigma_points[i] = (*F)(t);
        }

        /* 3. compute mean and covariance */
        x = Vector::Zero();
        for (int i = 0; i < 2 * dimension + 1; ++i)
        {
            x += sigma_points[i] * m_weights(i);
        }
        P = Matrix::Zero();
        for (int i = 0; i < 2 * dimension + 1; ++i)
        {
            Vector diff = sigma_points[i] - x;
            P += c_weights(i) * diff * diff.transpose();
        }
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
    double alpha, beta, kappa, lambda;

    Vector (*F)(Vector);   // state transition function
    Matrix P;              // state uncertainty covariance
    Matrix H, Htransp;     // measurement matrix
    Matrix R;              // measurement noise

    Eigen::Matrix< double, 2 * dimension + 1, 1 > m_weights, c_weights;
};

#endif /* end of include guard: KALMANFILTER_H */
