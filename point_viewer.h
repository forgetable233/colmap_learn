//
// Created by dcr on 22-7-26.
//

#ifndef TEST_POINT_VIEWER_H
#define TEST_POINT_VIEWER_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


namespace sfm {

    class point_viewer {
    private:
        std::vector<pcl::PointXYZRGB> points_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    public:
        point_viewer();

        point_viewer(const std::vector<Eigen::Vector3d> &world_points);

        void ViewPoints();

        virtual ~point_viewer();
    };
} // sfm

#endif //TEST_POINT_VIEWER_H
