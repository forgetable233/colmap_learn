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
            for (int i ; i < 8; i++) {
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

}