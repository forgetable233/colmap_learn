//
// Created by dcr on 22-7-15.
//

#ifndef TEST_INCREMENTAL_REBUILD_H
#define TEST_INCREMENTAL_REBUILD_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "camera_model.h"
#include "edge.h"
#include "point2d.h"
#include "point3d.h"
#include "thresholds.h"
#include "correspondence_graph.h"
#include "point_viewer.h"

namespace sfm {
    enum TrianguleType {
        kInitial = 0,
        kNormal = 1,
        kMultiView = 2
    };
    class IncrementalRebuild {
    private:
        std::shared_ptr<CorrespondenceGraph> scene_graph_;

        std::unordered_map<int, std::shared_ptr<Point3d>> world_points_;

        std::vector<int> joined_images_;
    public:
        IncrementalRebuild() = default;

        ~IncrementalRebuild() = default;

        IncrementalRebuild(CorrespondenceGraph* _graph);

        bool Init(int index);

        void SingulViewTriangulation(int index, TrianguleType type);

        void MultiViewTriangulation(int camera_key);

        static void PixToCam(cv::Mat &K,
                             std::vector<cv::Point2f> &input_points,
                             std::vector<cv::Point2f> &output_points);

        void BA();

        void BeginRebuild();

        void ViewAllPoints();

        void RegisterImage(Eigen::Matrix3d &R, Eigen::Vector3d &t, int camera_key);

        void ShowMatchResult(int begin_index);

        void CheckZDepthAndAddWorldPoints(const std::shared_ptr<CameraModel>& camera1,
                                          const std::shared_ptr<CameraModel>& camera2,
                                          cv::Mat &pst_4d);

        void ComputeScore(int camera_key);

        void CleanOutliers(int index,
                           std::vector<cv::Point2f> &clean_points_1,
                           std::vector<cv::Point2f> &clean_points_2);

        void GetUnregisteredPoints(int old_camera_key, int new_camera_key,
                                   std::vector<std::vector<Eigen::Vector2d>> &new_iamge_pixel_points,
                                   std::vector<std::vector<Eigen::Vector2d>> &old_iamge_pixel_points);

        int GetBestBeginEdge(int &second_max);

        // 直接从一张图片开始进行的穷举
        int GetNextBestEdge();

        int ComputeWorldPointKey();
    };
}

#endif //TEST_INCREMENTAL_REBUILD_H
