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
#include "reprojection_error.h"

namespace sfm {
    enum TrianguleType {
        kInitial = 0,
        kNormal = 1,
        kMultiView = 2
    };
    enum MultiViewType {
        kSVD = 0,
        kRANSAC = 1
    };
    class IncrementalRebuild {
    private:
        std::shared_ptr<CorrespondenceGraph> scene_graph_;

        std::unordered_map<int, std::shared_ptr<Point3d>> world_points_;

        std::vector<int> joined_images_;
    public:
        IncrementalRebuild() = default;

        ~IncrementalRebuild() = default;

        explicit IncrementalRebuild(CorrespondenceGraph* _graph);

        bool Init(int index);

        void SingalViewTriangulation(int index, TrianguleType type);

        void MultiViewTriangulation(int camera_key, MultiViewType type);

        static void PixToCam(cv::Mat &K,
                             std::vector<cv::Point2f> &input_points,
                             std::vector<cv::Point2f> &output_points);

        void BA();

        Eigen::Vector3d SVDComputeWorldPoint(int point_key,
                                             const std::unordered_map<int, std::vector<int>> &image_points,
                                             const std::map<int, Eigen::Matrix<double, 3, 4>> &P);

        void BeginRebuild();

        void ViewAllPoints();

        void RegisterImage(int camera_key);

        void ShowMatchResult(int begin_index);

        void CheckZDepthAndAddWorldPoints(const std::shared_ptr<CameraModel>& camera1,
                                          const std::shared_ptr<CameraModel>& camera2,
                                          cv::Mat &pst_4d);

        void CheckZDepthAndAddWorldPoints(const std::unordered_map<int, std::vector<int>> &points,
                                          const std::vector<Eigen::Vector3d> &world_points);

        /**
         * 计算对应图片的key值，作为之后寻找最佳图片的依据
         * @param camera_key
         */
        void ComputeScore(int camera_key);

        void CleanOutliers(int index,
                           std::vector<cv::Point2f> &clean_points_1,
                           std::vector<cv::Point2f> &clean_points_2);

        void GetUnregisteredPoints(int old_camera_key, int new_camera_key,
                                   std::unordered_map<int, std::vector<int>> &points);

        int GetBestBeginEdge(int &second_max);

        int ComputeWorldPointKey();
    };
}

#endif //TEST_INCREMENTAL_REBUILD_H
