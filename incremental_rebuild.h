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
    public:
        IncrementalRebuild() = default;

        ~IncrementalRebuild() = default;

        IncrementalRebuild(CorrespondenceGraph* _graph);

        bool Init(int index);

        void Triangulation(int index, TrianguleType type);

        static void PixToCam(cv::Mat &K,
                             std::vector<cv::Point2f> &input_points,
                             std::vector<cv::Point2f> &output_points);

        void BA();

        void BeginRebuild();

        void ViewAllPoints();

        void RegisterImage(Eigen::Matrix3d &R, Eigen::Vector3d &t, int edge_key);

        void ShowMatchResult(int begin_index);

        void CheckZDepthAndAddWorldPoints(const std::shared_ptr<CameraModel>& camera1,
                                          const std::shared_ptr<CameraModel>& camera2,
                                          cv::Mat &pst_4d);

        void CleanOutliers(int index,
                           std::vector<cv::Point2f> &clean_points_1,
                           std::vector<cv::Point2f> &clean_points_2);

        int GetBestBeginEdge(int &second_max);

        int GetNextBestEdge(const std::shared_ptr<bool[]>& used_list,
                            const std::shared_ptr<bool[]>& joined_list,
                            const std::shared_ptr<std::vector<int>>& index_list,
                            int last_edge_index);

        int ComputeWorldPointKey();
    };
}

#endif //TEST_INCREMENTAL_REBUILD_H
