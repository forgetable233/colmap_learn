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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "camera_model.h"
#include "edge.h"
#include "point.h"
#include "threholds.h"

namespace sfm {
    enum TrianguleType {
        kInitial = 0,
        kNormal = 1,
    };
    class IncrementalRebuild {
    private:
        int scene_graph[IMAGE_NUMBER][IMAGE_NUMBER];

        std::vector<std::shared_ptr<Edge>> edges_;

        std::shared_ptr<Points> points_;

        std::map<int, int> keys_;
    public:
        IncrementalRebuild() = default;

        ~IncrementalRebuild() = default;

        IncrementalRebuild(const std::vector<std::shared_ptr<Edge>> &_edges,
                           const std::shared_ptr<Points> &_points,
                           const int _scene_graph[IMAGE_NUMBER][IMAGE_NUMBER]);

        bool Init(int index);

        void Triangulation(int index, TrianguleType type);

        void PixToCam(cv::Mat &K, std::vector<cv::Point2f> &input_points, std::vector<cv::Point2f> &output_points);

        void BA();

        void BeginRebuild();

        void ShowMatchResult(int begin_index);

        void CleanOutliers(int index,
                           std::vector<cv::Point2f> &clean_points_1,
                           std::vector<cv::Point2f> &clean_points_2);

        int GetBestBeginEdge();

        int GetNextBestEdge(const std::shared_ptr<bool[]>& used_list,
                            const std::shared_ptr<bool[]>& joined_list,
                            const std::shared_ptr<std::vector<int>>& index_list,
                            int last_edge_index);
    };
}

#endif //TEST_INCREMENTAL_REBUILD_H
