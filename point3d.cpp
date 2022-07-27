//
// Created by dcr on 22-7-25.
//

#include "point3d.h"

namespace sfm {
    Point3d::Point3d(int key, Eigen::Vector3d _point) {

    }

    Point3d::Point3d(Eigen::Vector3d _point) {
        world_point_ = std::move(_point);
    }

    Eigen::Vector3d Point3d::GetPoint() {
        return this->world_point_;
    }

    void Point3d::AddRelatedPoint(std::vector<cv::Point3f> &world_points) {
        world_points.emplace_back(world_point_.x(), world_point_.y(), world_point_.z());
    }
}