#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Core>

using Eigen::MatrixXd;

class KalmanFilter
{
public:
    KalmanFilter(int dimension);
    virtual ~KalmanFilter();

    void SetTransferMatrix(MatrixXd mat);

private:
    int dimension;
};


#endif /* end of include guard: KALMANFILTER_H */

