//
// Created by dcr on 22-7-25.
//

#ifndef TEST_POINT3D_H
#define TEST_POINT3D_H
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>

#include <Eigen/Core>
namespace sfm {
    class Point3d {
    private:
        Eigen::Vector3d world_point_;

        int key_;

    public:
        Point3d() = default;

        Point3d(int key, Eigen::Vector3d _point);

        Point3d(Eigen::Vector3d _point);

        Eigen::Vector3d GetPoint();

        ~Point3d() = default;
    };
}

#endif //TEST_POINT3D_H
