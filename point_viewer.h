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
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


namespace sfm {

    class PointViewer {
    private:
        std::vector<Eigen::Vector3d> points_;

        std::vector<Eigen::Vector3d> normed_points_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};

        pcl::visualization::PCLVisualizer::Ptr viewer_{new pcl::visualization::PCLVisualizer("3D Viewer")};
    public:
        PointViewer();

        explicit PointViewer(const std::vector<Eigen::Vector3d> &world_points);

        void ViewPoints();

        void StorePCD();

        void NormalizePoints(const std::vector<Eigen::Vector3d> &points);

        virtual ~PointViewer();
    };
} // sfm

#endif //TEST_POINT_VIEWER_H
