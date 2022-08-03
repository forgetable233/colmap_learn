//
// Created by dcr on 22-8-3.
//

#ifndef TEST_REPROJECTION_ERROR_H
#define TEST_REPROJECTION_ERROR_H

#include <algorithm>
#include <iostream>
#include <cmath>

#include <ceres/ceres.h>
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
            Eigen::Vector3d predict_point;
            Eigen::Matrix<T, 3, 3> K;
            Eigen::Matrix<T, 3, 4> t;
            Eigen::Matrix<T, 3, 1> world_point;
            K << camera[0], camera[1], camera[2],
                    camera[3], camera[4], camera[5],
                    camera[6], camera[7], camera[8];
            t << camera[9], camera[10], camera[11], camera[12],
                    camera[13], camera[14], camera[15], camera[16],
                    camera[17], camera[18], camera[19], camera[20];
            world_point << point[0], point[1], point[2];
            predict_point = K * t * world_point.homogeneous();
            predict_point.x() /= predict_point.z();
            predict_point.y() /= predict_point.z();
            predictions[0] = T(predict_point.x());
            predictions[1] = T(predict_point.y());
            residuals[0] = predictions[0] - T(observed_x_);
            residuals[1] = predictions[1] - T(observed_y_);
            return true;
        }

        template<typename T>
        static inline bool ComputePredictPoint(const T *camera, const T *point, T *predictions) {

            return true;
        }

        static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 21, 3>
                    (new ReprojectionError(observed_x, observed_y)));
        }
    };

} // sfm

#endif //TEST_REPROJECTION_ERROR_H
