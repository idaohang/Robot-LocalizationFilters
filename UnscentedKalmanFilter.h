#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues> 
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>

/******************************************************
 *  url: http://www.cslu.ogi.edu/nsel/ukf/node6.html  * 
 ******************************************************/

using namespace Eigen;

// N = dimension, M = number of measurements
template<unsigned int N, unsigned int M>
class UnscentedKalmanFilter
{
    typedef Eigen::Matrix< double , N , 1> VectorN;
    typedef Eigen::Matrix< double , M , 1> VectorM;
    typedef Eigen::Matrix< double , N , N> MatrixNN;
    typedef Eigen::Matrix< double , M , N> MatrixMN;
    typedef Eigen::Matrix< double , N , M> MatrixNM;
    typedef Eigen::Matrix< double , M , M> MatrixMM;

public:
    // constructor and destructor
    UnscentedKalmanFilter() {}
    virtual ~UnscentedKalmanFilter() {}

    /**
     * State variable priori (N x 1)
     * keep record of the best current belief state
     * */
    void SetState(VectorN vec) 
    {
        this->x = vec; 
    }
    /**
     * Transition function
     * calculate the belief propagation to next state
     * */
    void SetStateTransition(VectorN (*func)(VectorN))
    {
        this->F = func;
    }
    /**
     * Covariance matrix of the state transition (N x N)
     * */
    void SetStateCovariance(MatrixNN mat)
    {
        this->P = mat;
    }
    /**
     * Measure extraction function
     * used to extract state features from measurement input
     * */
    void SetMeasureExtraction(VectorM (*func)(VectorN))
    {
        this->H = func;
    }
    /**
     * Covariance matrix of the measurement, noise (M x M)
     * */
    void SetMeasureCovariance(MatrixMM mat)
    {
        this->R = mat;
    }
    /**
     * Set up relevant parameters for Unscented Transform
     * kappa > 0
     * 0 < alpha < 1
     * beta = 2 (optimal)
     * */
    void SetUnscentedParameters(double kappa, double alpha, double beta = 2)
    {
        this->alpha = alpha;
        this->beta = beta;
        this->kappa = kappa;
        this->lambda = alpha * alpha * (N + kappa) - N;

        double factor1 = lambda / (N + lambda);
        double factor2 = 0.5 * (N + lambda);

        /* set up the weights coefficients */
        m_weights(0) = factor1;
        c_weights(0) = factor1 + (1 - alpha * alpha + beta);
        for (int i = 1; i < 2 * N + 1; ++i)
        {
            m_weights(i) = factor2;
            c_weights(i) = factor2;
        }
    }
    /**
     * Calculate the update from model, and correct the output
     * using the measurement inputs
     * z: measurement (M x 1)
     * */
    void Update(VectorM z)
    {
        /***********************
         *  Update prediction  *
         ***********************/

        /* 1. compute square root covariance */
        auto Sqrt = P.sqrt();

        /* 2. set up the transitioned vectors */
        VectorN sigma_points[2 * N + 1];
        {
            /* BLOCK FOR SIGMA POINTS CALCULATION */
            // set up the first: mean
            sigma_points[0] = (*F)(x);
            // set up the 1 to N
            for (int i = 1; i < N + 1; ++i)
                sigma_points[i] = (*F)(x + std::sqrt(N + lambda) * Sqrt.row(i - 1).transpose());
            // set up the 1 to N
            for (int i = N + 1; i < 2 * N + 1; ++i) 
                sigma_points[i] = (*F)(x - std::sqrt(N + lambda) * Sqrt.row(i - N - 1).transpose());
        }

        /* 3. compute mean and covariance */
        VectorN xk = VectorN::Zero();
        {
            /* BLOCK FOR SIGMA POINTS MEAN */
            for (int i = 0; i < 2 * N + 1; ++i) 
                xk += sigma_points[i] * m_weights(i);
        }
        MatrixNN Pk = MatrixNN::Zero();
        {
            /* BLOCK FOR SIGMA POINTS COVARIANCE */
            for (int i = 0; i < 2 * N + 1; ++i)
            {
                VectorN diff = sigma_points[i] - x;
                Pk += c_weights(i) * diff * diff.transpose();
            }
        }
        Pk += MatrixNN::Identity() * 2;

        // --------------------------------------------------------

        /* 4. measurement extraction */
        VectorM points[2 * N + 1];
        {
            /* BLOCK FOR SIGMA POINTS TRANSFORMATION */
            for (int i = 0; i < 2 * N + 1; ++i)
                points[i] = (*H)(sigma_points[i]);
        }

        /* 5. compute new mean and covariance */
        VectorM xm = VectorM::Zero();
        {
            /* BLOCK FOR POINTS MEAN */
            for (int i = 0; i < 2 * N + 1; ++i) 
                xm += points[i] * m_weights(i);
        }
        MatrixMM Pyy = MatrixMM::Zero();
        {
            /* BLOCK FOR POINTS COVARIANCE */
            for (int i = 0; i < 2 * N + 1; ++i)
            {
                VectorM diff = points[i] - xm;
                Pyy += c_weights(i) * diff * diff.transpose();
            }
        }
        MatrixNM Pxy = MatrixNM::Zero();
        {
            /* BLOCK FOR POINTS COVARIANCE */
            for (int i = 0; i < 2 * N + 1; ++i)
            {
                VectorN x_diff = sigma_points[i] - xk;
                VectorM y_diff = points[i] - xm;
                Pxy += c_weights(i) * x_diff * y_diff.transpose();
            }
        }

        // std::cout << P << std::endl;
        // std::cout << "---------------------------" << std::endl;
        // std::cout << Sqrt << std::endl;
        // std::cout << "---------------------------" << std::endl;

        /* 6. compute new Gaussian distributions */
        MatrixNM K = Pxy * Pyy.inverse();
        x = xk + K * (z- xm);
        P = Pk - K * Pyy * K.transpose();

        // std::cout << x.transpose() << std::endl;
        // std::cout << "---------------------------" << std::endl;
        std::cout << Pk << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << K * Pyy * K.transpose() << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << P << std::endl;
        std::cout << "===========================" << std::endl;
    }
    /**
     * Get the current state variable
     * */
    VectorN GetCurrentState() const { return x; }
private:
    double alpha, beta, kappa, lambda;
    VectorN x;     // state variable

    VectorN (*F)(VectorN);   // state transition function
    VectorM (*H)(VectorN);   // measurement extracton function
    MatrixNN P;              // state uncertainty covariance
    MatrixMM R;              // measurement noise

    Eigen::Matrix< double, 2 * N + 1, 1 > m_weights, c_weights;
};

#endif /* end of include guard: KALMANFILTER_H */
