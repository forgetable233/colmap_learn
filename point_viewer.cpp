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
        this->cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        viewer_.setBackgroundColor(.0, .0, .0);
        for (const auto &point: world_points) {
            pcl::PointXYZRGB temp;
            temp.x = static_cast<float>(point.x());
            temp.y = static_cast<float>(point.y());
            temp.z = static_cast<float>(point.z());
            temp.rgb = 0xff0000;
            cloud_->emplace_back(temp);
        }
    }

    void PointViewer::ViewPoints() {
        viewer_.addPointCloud<pcl::PointXYZRGB>(cloud_, "world points");
        std::cout << "The size of the world points is " << this->cloud_->size() << std::endl;
        viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "world points");
        viewer_.addCoordinateSystem(1);
       /* viewer_.setCameraPosition(1, 1, 1,
                                  0, 0, 4,
                                  0, 1, 5);*/
        while (!viewer_.wasStopped()) {
            viewer_.spinOnce(100);
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

} // sfm