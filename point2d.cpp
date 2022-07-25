//
// Created by D1456 on 2022/7/6.
//

#include "point2d.h"

namespace sfm {
    Point2d::Point2d() {

    }

    Point2d::~Point2d() {
    }

    Point2d::Point2d(int key, Eigen::Vector2d _points2d) {
    }

    void Point2d::AddWoldPoint(Eigen::Vector3d _world_point) {
    }

    Point2d::Point2d(int key, cv::Point2i &_points2d ) {
    }

    void Point2d::AddCorrPoint(int image_id, int point_id) {
        correspondence_.insert(std::pair<int, int>(image_id, point_id));
    }
}