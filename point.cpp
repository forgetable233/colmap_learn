//
// Created by D1456 on 2022/7/6.
//

#include "point.h"

namespace sfm {

    void Points::AddCloudPoint(std::vector<cv::Point3f> &points) {
        for (auto point: points) {
            pcl::PointXYZRGB temp_point{};
            temp_point.x = point.x;
            temp_point.y = point.y;
            temp_point.z = point.z;

            temp_point.r = 255;
            temp_point.g = 0;
            temp_point.b = 0;
            cloud_->push_back(temp_point);

//            cout << "X: " << point.x << ' ' << "Y: " << point.y << ' ' << "Z: " << point.z << std::endl;
        }
    }

    // TODO 点云失踪事件
    void Points::ViewPoints() {
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGB> rgb(this->cloud_);

        viewer->addPointCloud<pcl::PointXYZRGB>(this->cloud_, "The initial cloud");
        viewer->setBackgroundColor(1.0f, 1.0f, 1.0f);
        viewer->addCoordinateSystem(1);
//        viewer->setCameraPosition(0, 0, 0, 1, 1, 1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                 10,
                                                 "The initial cloud");

//        std::cout << this->cloud_->size() << std::endl;
        ComputeCenter();
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }

    Points::Points() {
        this->cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    Points::~Points() {
        this->cloud_->clear();
    }

    void Points::ComputeCenter() const {
        double x = 0.0f;
        double y = 0.0f;
        double z = 0.0f;

        for (int i = 0; i < cloud_->size(); ++i) {
            x += cloud_->at(i).x;
            y += cloud_->at(i).y;
            z += cloud_->at(i).z;
        }

        x /= static_cast<double>(cloud_->size());
        z /= static_cast<double>(cloud_->size());
        y /= static_cast<double>(cloud_->size());

        std::cout << "The x is " << x << std::endl;
        std::cout << "The z is " << y << std::endl;
        std::cout << "The y is " << z << std::endl;
    }
}