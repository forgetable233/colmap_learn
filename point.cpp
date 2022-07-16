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
    void Points::AddCloudPoint(const std::shared_ptr<Edge>& _edge, const std::vector<cv::Point3f> &points) {
        Eigen::Vector3d temp_point;
        int i = 0;
        for (auto point: points) {
            temp_point.x() = point.x;
            temp_point.y() = point.y;
            temp_point.z() = point.z;
            points_.push_back(std::make_shared<Point>(temp_point));
            /** 确定此时这个点对应图片的索引 **/
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
        EigenToCloud();
        std::cout << "The size of the cloud point is " << this->cloud_->size() << std::endl;
        pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGB> rgb(this->cloud_);

        viewer->addPointCloud<pcl::PointXYZRGB>(this->cloud_, "The initial cloud");
        viewer->setBackgroundColor(.0f, .0f, .0f);
//        viewer->setCameraPosition(0, 0, 0, 10, 10, 10);
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
        std::cout << "The size of the cloud point is " << cloud_->size() << std::endl;

        for (auto &i: *cloud_) {
            x += i.x;
            y += i.y;
            z += i.z;
            std::cout << "The x is " << x << std::endl;
            std::cout << "The y is " << y << std::endl;
            std::cout << "The z is " << z << std::endl;
            std::cout << "i.x is " << i.x << std::endl;
            std::cout << "i.z is " << i.y << std::endl;
            std::cout << "i.y is " << i.z << std::endl;
            std::cout << std::endl;
        }

        x /= static_cast<double>(cloud_->size());
        z /= static_cast<double>(cloud_->size());
        y /= static_cast<double>(cloud_->size());

        std::cout << "The x is " << x << std::endl;
        std::cout << "The y is " << y << std::endl;
        std::cout << "The z is " << z << std::endl;
    }

    // TODO 感觉有问题
    void Points::GetLinkPoint(const std::shared_ptr<std::vector<cv::Point3f>> &world_point,
                              const std::shared_ptr<Edge> &edge,
                              SearchType type) {
        std::map<int, int>::iterator target;
        int key;
        if (type == kQuery) {
            key = edge->camera1_->key_;
            /** 遍历全体点寻找 **/
            for (const auto &point: points_) {
                target = point->key_.find(key);
                if (target != point->key_.end()) {
                    int i = 0;
                    for (; i < edge->points1_index_.size(); ++i) {
                        if (edge->points1_index_[i] == target->second) {
                            if (edge->point1_pass_[i]) {
                                world_point->emplace_back(point->position.x(),
                                                          point->position.y(),
                                                          point->position.z());
                            }
                            break;
                        }
                    }
                }
            }
        } else {
            key = edge->camera2_->key_;

            for (const auto &point: points_) {
                target = point->key_.find(key);
                if (target != point->key_.end()) {
                    int i = 0;
                    for (; i < edge->points2_index_.size(); ++i) {
                        if (edge->points2_index_[i] == target->second) {
                            if (edge->point2_pass_[i]) {
                                world_point->emplace_back(point->position.x(),
                                                          point->position.y(),
                                                          point->position.z());
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    void Points::EigenToCloud() {
        pcl::PointXYZRGB temp_point;
        temp_point.r = 255;
        temp_point.g = 0;
        temp_point.b = 0;
        for (const auto& point: this->points_) {
            temp_point.x = static_cast<float>(point->position.x());
            temp_point.y = static_cast<float>(point->position.y());
            temp_point.z = static_cast<float>(point->position.z());
            this->cloud_->push_back(temp_point);
        }
        temp_point.r = 0;
        temp_point.g = 255;
        temp_point.b = 0;
        temp_point.x = 1.0f;
        temp_point.y = 1.0f;
        temp_point.z = 1.0f;
        this->cloud_->push_back(temp_point);
    }
}