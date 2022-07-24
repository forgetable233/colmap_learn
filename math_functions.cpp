//
// Created by D1456 on 2022/7/5.
//

#include "math_functions.h"

namespace sfm {
    void MathFunction::EpipolarGeometry() {

    }

    void MathFunction::ComputeEssentialMatrix(std::vector<Eigen::Vector3d> &key_points_1,
                                              std::vector<Eigen::Vector3d> &key_points_2,
                                              Eigen::Matrix3d &E) {
        const int iterate_number = 10;
        const int size = key_points_1.size();
        int max = 0;
        int local_max = 0;

        Eigen::Matrix<double, 8, 9, 0> A;
        Eigen::Matrix<double, 8, 1, 0> b;
        Eigen::Matrix<double, 9, 1> e;
        Eigen::Matrix3d temp_E;

        std::unique_ptr<bool[]> used(new bool[size]);

        std::default_random_engine random_number;
        random_number.seed(time(0));

        for (int i = 0; i < iterate_number; ++i) {
            /** 构建对应的方程 **/
            for (int i; i < 8; i++) {
                while (true) {
                    int index = random_number() % size;
                    if (!used[index]) {
                        used[index] = true;
                        A(i, 0) = key_points_2[index].x() * key_points_1[index].x();
                        A(i, 1) = key_points_2[index].x() * key_points_1[index].y();
                        A(i, 2) = key_points_1[index].x();
                        A(i, 3) = key_points_2[index].y() * key_points_1[index].x();
                        A(i, 4) = key_points_2[index].y() * key_points_1[index].y();
                        A(i, 5) = key_points_2[index].y();
                        A(i, 6) = key_points_1[index].x();
                        A(i, 7) = key_points_1[index].y();
                        A(i, 8) = 1.0f;
                        break;
                    }
                }
            }
            /** 使用QR分解进行求解 **/
            temp_E(0, 0) = e(0, 0);
            temp_E(0, 1) = e(1, 0);
            temp_E(0, 2) = e(2, 0);
            temp_E(1, 0) = e(3, 0);
            temp_E(1, 1) = e(4, 0);
            temp_E(1, 2) = e(5, 0);
            temp_E(2, 0) = e(6, 0);
            temp_E(2, 1) = e(7, 0);
            temp_E(2, 2) = e(8, 0);

            local_max = GetInliersNumber(key_points_1, key_points_2, temp_E);
            if (local_max > max) {
                max = local_max;
                E = temp_E;
            }
//            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
//        std::cout << "The rank of the parameter matrix is " << svd.rank();
        }
    }

    int MathFunction::GetInliersNumber(std::vector<Eigen::Vector3d> &key_points_1,
                                       std::vector<Eigen::Vector3d> &key_points_2,
                                       Eigen::Matrix3d &E) {
        int number = 0;
        for (int i = 0; i < key_points_1.size(); ++i) {
            Eigen::Matrix<double, 1, 1> test;
            test = key_points_2[i].transpose() * E * key_points_1[i];
            if (test(0, 0) <= ESSENTIAL_INLIER_THRESHOLD) {
                number++;
            }
        }
        return number;
    }

    //  use RANSAC to compute the fundamental matrix
    void
    MathFunction::ComputeFundamentalMatrix(std::vector<Eigen::Vector2d> &points1,
                                           std::vector<Eigen::Vector2d> &points2,
                                           Eigen::Matrix3d &F) {
        if (points2.size() != points1.size()) {
            std::cerr << "The size of the points should be equal" << std::endl;
            return;
        }
        const int size = points1.size();
        std::cout << "The size of the point is " << size << std::endl;
        int used_num = 0;
        int local_inliers_num = 0;
        int global_inliers_num = 0;
        Eigen::Matrix3d local_F;
        std::shared_ptr<bool[]> used(new bool[size]);
        std::vector<Eigen::Vector2d> temp_points1;
        std::vector<Eigen::Vector2d> temp_points2;

        temp_points1.reserve(8);
        temp_points2.reserve(8);

        for (; used_num  < size * 0.9; used_num +=8) {
            temp_points1.clear();
            temp_points2.clear();
            for (int i = 0; i < 8; ++i) {
                int index;
                while (true) {
                    index = rand() % size;
                    if (!used[index]) {
                        used[index] = true;
                        break;
                    }
                }
                temp_points1.emplace_back(points1[index]);
                temp_points2.emplace_back(points2[index]);
            }
            EightComputeFundamentalMatrix(temp_points1, temp_points2, local_F);
            for (int i = 0; i < size; ++i) {
                Eigen::Vector3d temp_point2;
                Eigen::Vector3d temp_point1;
                temp_point2 = temp_points2[i].homogeneous();
                temp_point1 = temp_points1[i].homogeneous();
                if (temp_point2.transpose() * local_F * temp_point1 < FUNDAMENTAL_INLIER_THRESHOLD) {
                    local_inliers_num ++;
                }
            }
            if (local_inliers_num > global_inliers_num) {
                F = local_F;
            }
        }
    }

    // use eight points to compute the fundamental matrix
    void
    MathFunction::EightComputeFundamentalMatrix(std::vector<Eigen::Vector2d> &points1,
                                                std::vector<Eigen::Vector2d> &points2,
                                                Eigen::Matrix3d &F) {
        if (points1.size() != points2.size()) {
            std::cerr << "The size of the points should be equal" << std::endl;
            return;
        }
        const int size = points1.size();
        std::vector<Eigen::Vector2d> points1_norm;
        std::vector<Eigen::Vector2d> points2_norm;
        points1_norm.reserve(points1.size());
        points2_norm.reserve(points1.size());

        Eigen::Matrix3d trans_m1;
        Eigen::Matrix3d trans_m2;

        CenterAndNormalizePoints(points1, points1_norm, trans_m1);
        CenterAndNormalizePoints(points2, points2_norm, trans_m2);
        Eigen::Matrix<double, Eigen::Dynamic, 9> param(size, 9);

        // 构建 x2T * F * x1 的多项式方程，并使用SVD分解
        for (int i = 0; i < size; ++i) {
            param.block<1, 3>(i, 0) = points2[i].homogeneous();
            param.block<1, 3>(i, 0) *= points1[i](0);
            param.block<1, 3>(i, 3) = points2[i].homogeneous();
            param.block<1, 3>(i, 3) *= points1[i](1);
            param.block<1, 3>(i, 6) = points2[i].homogeneous();
        }
        Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(param, Eigen::ComputeFullV);
        const Eigen::VectorXd null_space = svd.matrixV().col(8);
        const Eigen::Map<const Eigen::Matrix3d> F_temp(null_space.data());

        // 对F进行奇异值约束，也就是最后一个必为0
        Eigen::JacobiSVD<Eigen::Matrix3d> F_svd(F_temp, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::Vector3d S = F_svd.singularValues();
        S(2) = 0;
        F = F_svd.matrixU() * S.asDiagonal() * F_svd.matrixV();
        F = trans_m2.transpose() * F * trans_m1;
    }

    void MathFunction::CenterAndNormalizePoints(std::vector<Eigen::Vector2d> &points,
                                                std::vector<Eigen::Vector2d> &points_normed,
                                                Eigen::Matrix3d &trans_m) {
        // 将所有点投影到质心在原点，半径为根号二的圆内，提升稳定性.
        Eigen::Vector2d centroid;
        for (const auto &point: points) {
            centroid += point;
        }
        centroid /= static_cast<double>(points.size());
        double dis = centroid.squaredNorm();
        double zoom_rate = sqrt(2 / dis);
        trans_m << zoom_rate, 0.0f, -zoom_rate * centroid(0),
                0.0f, zoom_rate, -zoom_rate * centroid(1),
                0.0f, 0.0f, 1.0f;
        for (const auto &point: points) {
            points_normed.emplace_back((trans_m * point.homogeneous()).hnormalized());
        }
    }

}