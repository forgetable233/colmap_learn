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

    bool Point2d::AddCorrPoint(int image_id, int point_id) {
        if (correspondence_.find(image_id) == correspondence_.end()) {
            correspondence_.insert(std::pair<int, int>(image_id, point_id));
            return true;
        } else {
            return false;
        }
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
        return (camera * 100000 + index);
    }

    int Point2d::GetCameraKey(int point_key) {
//        return point_key /
    }

    void Point2d::AddWorldPoints(std::shared_ptr<Point3d> point_ptr) {
        this->registered = true;
        this->world_point_ = point_ptr;
    }

    void Point2d::GetCorrs(std::vector<int> &corrs) {
        int i = 0;
        for (const auto &corr: correspondence_) {
            corrs[i] = ComputePointKey(corr.first, corr.second);
            i++;
        }
    }

    bool Point2d::HasRegistered() const {
        return this->registered;
    }

    void Point2d::SetRegistered(int point_key) {
        this->registered = true;
    }

    void Point2d::AddRelatedPoint(std::vector<cv::Point3f> &world_points,
                                  std::vector<cv::Point2f> &image_points) {
        if (world_point_ == nullptr) {
            std::cerr << "The pointer of the world point is nullptr" << std::endl;
        }
        world_point_->AddRelatedPoint(world_points);
        image_points.emplace_back(pixel_point_.x(), pixel_point_.y());
    }

    void Point2d::GetPixelPoint(Eigen::Vector2d &point) {
        point = pixel_point_;
    }

    void Point2d::GetPixelAndWorldPoint(double *pixel_point, double *world_point) {
        pixel_point[0] = this->pixel_point_.x();
        pixel_point[1] = this->pixel_point_.y();
        world_point_->GetWorldPoint(world_point);
    }

    void Point2d::RefreshWorldPoint(double *new_world_point) {
        world_point_->RefreshWorldPoint(new_world_point);
    }
}