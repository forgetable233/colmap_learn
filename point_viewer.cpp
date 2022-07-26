//
// Created by dcr on 22-7-26.
//

#include "point_viewer.h"

namespace sfm {
    point_viewer::point_viewer() {}

    point_viewer::~point_viewer() {

    }

    point_viewer::point_viewer(const std::vector<Eigen::Vector3d> &world_points) {
        for (const auto &point: world_points) {
            pcl::PointXYZRGB temp;
            temp.x = static_cast<float>(point.x());
            temp.y = static_cast<float>(point.y());
            temp.z = static_cast<float>(point.z());
            temp.r = 255;
            temp.g = 255;
            temp.b = 255;
            cloud_->emplace_back(temp);
        }
    }

    void point_viewer::ViewPoints() {
        pcl::visualization::CloudViewer viewer("sfm results");
        viewer.showCloud(cloud_);
        while (!viewer.wasStopped()) {

        }
    }
} // sfm