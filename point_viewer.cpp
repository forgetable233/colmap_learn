//
// Created by dcr on 22-7-26.
//

#include "point_viewer.h"

namespace sfm {
    PointViewer::PointViewer() {
        this->cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    PointViewer::~PointViewer() {

    }

    PointViewer::PointViewer(const std::vector<Eigen::Vector3d> &world_points) {
        NormalizePoints(world_points);
        for (const auto &point: normed_points_) {
            pcl::PointXYZRGB temp;
            temp.x = static_cast<float>(point.x());
            temp.y = static_cast<float>(point.y());
            temp.z = static_cast<float>(point.z());
            temp.rgb = 0xff0000;
            cloud_->emplace_back(temp);
        }
    }

    void PointViewer::ViewPoints() {
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_, "world points");
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "world points");
        viewer_->addCoordinateSystem(1000);
        viewer_->setCameraPosition(1, 1, 1,
                                   0, 0, 4,
                                   0, 1, 5);
        std::cout << "The size of the point cloud is " << cloud_->size() << std::endl;
        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(100);
        }
    }

    void PointViewer::StorePCD() {
        if (!cloud_->empty()) {
            pcl::PCDWriter writer;
            writer.write("../pointClouds/InitialData.pcb", *cloud_);
        } else {
            std::cerr << "The size of the point clouds is zero" << std::endl;
        }
    }

    void PointViewer::NormalizePoints(const std::vector<Eigen::Vector3d> &points) {
        double x;
        double y;
        double z;
        for (const auto &point: points) {
            x += point.x();
            y += point.y();
            z += point.z();
            this->points_.emplace_back(point);
        }
        x /= static_cast<double>(points.size());
        y /= static_cast<double>(points.size());
        z /= static_cast<double>(points.size());
        for (const auto &point: points) {
            this->normed_points_.emplace_back(point.x() - x, point.y() - y, point.z() - z);
        }
    }

} // sfm