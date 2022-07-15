//
// Created by D1456 on 2022/7/6.
//

#ifndef TEST_POINT_H
#define TEST_POINT_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "edge.h"
#include "camera_model.h"

namespace sfm {
    struct CameraPlace {
        int key;

        int index;

        CameraPlace(int _key, int _index) : key(_key), index(_index) {}
    };
    enum SearchType {
        kQuery = 0,
        kTrain = 1
    };
    struct Point {
        /** 保存目前这个点是从那张图片获得的,以及对应的索引 **/
        std::map<int, int> key_;

        Eigen::Vector3d position;

        explicit Point(Eigen::Vector3d &_p) {
            position = std::move(_p);
        }

        void AddPair(int camera, int index) {
            key_.insert(std::pair<int, int>(camera, index));
        }
    };
    class Points {

    public:
        std::vector<std::shared_ptr<Point>> points_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{};

        Points();

        ~Points();

        void AddCloudPoint(const std::shared_ptr<Edge> _edge, const std::vector<cv::Point3f> &points);

        void GetLinkPoint(const std::shared_ptr<std::vector<cv::Point3f>>& world_point, const std::shared_ptr<Edge>& edge, enum SearchType);

        void ViewPoints();

        void ComputeCenter() const;
    };
}

#endif //TEST_POINT_H
