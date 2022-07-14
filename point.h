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

namespace sfm {
    struct Point {
        /** 保存目前这个点是从那张图片获得的 **/
        int key;

        Eigen::Vector3d position;

        Point(int _key, Eigen::Vector3d &_p) : key(_key) {
            position = _p;
        }
    };
    class Points {

    public:
        std::vector<Point> points_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{};

        Points();

        ~Points();

        void AddCloudPoint(std::vector<cv::Point3f> &points);

        void ViewPoints();

        void ComputeCenter() const;
    };
}

#endif //TEST_POINT_H
