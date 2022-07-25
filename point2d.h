//
// Created by D1456 on 2022/7/6.
//

#ifndef TEST_POINT2D_H
#define TEST_POINT2D_H

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "edge.h"
#include "camera_model.h"

namespace sfm {
    class Point2d {
    private:
        // 此处的key为对应点在每张图片的位置，包含一个key和一个像素坐标
        Eigen::Vector2d pixel_point_;

        std::unordered_map<int, int> correspondence_;

        int world_point_id_ = -1;

        bool registered = false;
    public:
        Point2d();

        Point2d(int key, Eigen::Vector2d _points2d);

        Point2d(int key, cv::Point2i &_points2d);

        void AddWoldPoint(Eigen::Vector3d _world_point);

        inline void AddCorrPoint(int image_id, int point_id);

        ~Point2d();
    };
}

#endif //TEST_POINT2D_H
