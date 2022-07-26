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

    Point2d::Point2d(int key, cv::Point2i &_points2d) {
    }

    void Point2d::AddCorrPoint(int image_id, int point_id) {
        correspondence_.insert(std::pair<int, int>(image_id, point_id));
    }

    bool Point2d::FindPoint(int image_id) {
        if (correspondence_.find(image_id) == correspondence_.end()) {
            return true;
        }
        return false;
    }

    int Point2d::GetCorrNumber() {
        return correspondence_.size();
    }

    int Point2d::ComputePointKey(int camera, int index) {
        return camera * 10000 + index;
    }

    void Point2d::AddWorldPoints(std::shared_ptr<Point3d> point_ptr) {
        this->registered = true;
        this->world_point_ = point_ptr;
    }

    std::shared_ptr<int[]> Point2d::GetCorrs() {
        if (correspondence_.empty()) {
            return nullptr;
        }
        std::shared_ptr<int[]> corrs(new int[correspondence_.size()]);
        int i = 0;
        for (const auto &corr: correspondence_) {
            corrs[i] = ComputePointKey(corr.first, corr.second);
        }
        return corrs;
    }
}