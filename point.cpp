//
// Created by D1456 on 2022/7/6.
//

#include "point.h"

namespace sfm {
    Points::Points() {
        this->cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    Points::~Points() {
        this->cloud_->clear();
    }

    /** 完成了三角化后新加入的点理论上不会有重复 **/
    void Points::AddCloudPoint(const std::shared_ptr<Edge> _edge,const std::vector<cv::Point3f> &points) {
        Eigen::Vector3d temp_point;
        int i = 0;
        for (auto point: points) {
            temp_point.x() = point.x;
            temp_point.y() = point.y;
            temp_point.z() = point.z;
            points_.push_back(std::make_shared<Point>(temp_point));
            while (!_edge->point1_pass_[i] || !_edge->point2_pass_[i]) {
                i++;
            }
            points_.back()->AddPair(_edge->camera1_->key_, _edge->points1_index_[i]);
            points_.back()->AddPair(_edge->camera2_->key_, _edge->points2_index_[i]);
        }
    }

    void Points::ViewPoints() {
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGB> rgb(this->cloud_);

        viewer->addPointCloud<pcl::PointXYZRGB>(this->cloud_, "The initial cloud");
        viewer->setBackgroundColor(.0f, .0f, .0f);
//        viewer->setCameraPosition(0, 0, 0, 1, 1, 1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                 10,
                                                 "The initial cloud");
        viewer->addCoordinateSystem(1);

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }

    void Points::ComputeCenter() const {
        double x = 0.0f;
        double y = 0.0f;
        double z = 0.0f;
        std::cout << cloud_->size() << std::endl;

        for (auto &i: *cloud_) {
            x += i.x;
            y += i.y;
            z += i.z;

        }

        x /= static_cast<double>(cloud_->size());
        z /= static_cast<double>(cloud_->size());
        y /= static_cast<double>(cloud_->size());

        std::cout << "The x is " << x << std::endl;
        std::cout << "The z is " << y << std::endl;
        std::cout << "The y is " << z << std::endl;
    }
}