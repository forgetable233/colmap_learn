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
    class Points {
        struct Point {
            int key;
            cv::Point3f position;

            Point(int _key, cv::Point3f &_p) : key(_key) {
                position = _p;
            }
        };
    public:
        int key_{};

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{};

        Points();

        ~Points();

        void AddCloudPoint(std::vector<cv::Point3f> &points);

        void ViewPoints();
    };
}

#endif //TEST_POINT_H
