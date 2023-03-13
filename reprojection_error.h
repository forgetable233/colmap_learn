//
// Created by dcr on 22-8-3.
//

#ifndef TEST_REPROJECTION_ERROR_H
#define TEST_REPROJECTION_ERROR_H

#include <algorithm>
#include <iostream>
#include <cmath>

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace sfm {

    class ReprojectionError {
    private:
        double observed_x_{};
        double observed_y_{};
    public:
        ReprojectionError();

        ReprojectionError(double observation_x, double observation_y) : observed_x_(observation_x),
                                                                        observed_y_(observation_y) {}

        virtual ~ReprojectionError();

        template<typename T>
        bool operator()(const T *const camera, const T *const point, T *residuals) const {
            T predictions[2];
            Eigen::Matrix<T, 3, 1> predict_point;
            Eigen::Matrix<T, 3, 3> K;
            Eigen::Matrix<T, 3, 4> t;
            Eigen::Matrix<T, 3, 1> world_point;
            K << T(camera[0]), T(camera[1]), T(camera[2]),
                    T(camera[3]), T(camera[4]), T(camera[5]),
                    T(camera[6]), T(camera[7]), T(camera[8]);
            t << T(camera[9]), T(camera[10]), T(camera[11]), T(camera[12]),
                    T(camera[13]), T(camera[14]), T(camera[15]), T(camera[16]),
                    T(camera[17]), T(camera[18]), T(camera[19]), T(camera[20]);
            world_point << T(point[0]), T(point[1]), T(point[2]);
            predict_point = K * t * world_point.homogeneous();
            predict_point.x() /= predict_point.z();
            predict_point.y() /= predict_point.z();
            predictions[0] = T(predict_point.x());
            predictions[1] = T(predict_point.y());
            residuals[0] = predictions[0] - T(observed_x_);
            residuals[1] = predictions[1] - T(observed_y_);
            return true;
        }

        static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 21, 3>
                    (new ReprojectionError(observed_x, observed_y)));
        }

        double computeError();
    };

} // sfm

#endif //TEST_REPROJECTION_ERROR_H
