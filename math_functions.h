//
// Created by D1456 on 2022/7/5.
//

#ifndef TEST_MATH_FUNCTIONS_H
#define TEST_MATH_FUNCTIONS_H

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <cstdlib>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include "img_loader.h"
#include "threholds.h"

namespace sfm {
class MathFunction {
public:
    MathFunction() = default;

    ~MathFunction() = default;

    static void EpipolarGeometry();

    static void ComputeEssentialMatrix(std::vector<Eigen::Vector3d> &key_points_1,
                                std::vector<Eigen::Vector3d> &key_points_2,
                                Eigen::Matrix3d &E);

    /**
     * 用于计算对应的内点的数量
     * @param key_points_1
     * @param key_points_2
     * @param E
     * @return
     */
    static int GetInliersNumber(std::vector<Eigen::Vector3d> &key_points_1,
                         std::vector<Eigen::Vector3d> &key_points_2,
                         Eigen::Matrix3d &E);
};

};

#endif //TEST_MATH_FUNCTIONS_H
