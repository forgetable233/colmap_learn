//
// Created by dcr on 22-7-24.
//

#ifndef TEST_CORRESPONDENCE_GRAPH_H
#define TEST_CORRESPONDENCE_GRAPH_H

#include <unordered_map>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "edge.h"
#include "point2d.h"
#include "thresholds.h"

namespace sfm {
    enum CameraChoice {
        kCamera1 = 0,
        kCamera2 = 1
    };

    class CorrespondenceGraph {
    private:
        // 保存每一张图片的相关性，用来选择最佳起始图片，同时确定图片是否已经注册过
        struct Image {
            explicit Image(unsigned int _corr) : correspondence_number(_corr) {}

            int key;

            unsigned int correspondence_number = 0;

            unsigned int score = 0;

            bool registered = false;
        };
        /** 保存当前加入的图片的数量 **/
        int joined_number_ = 2;

        /** 对每一个点信息的保存，此处有一层封装 **/
        std::unordered_map<int, std::shared_ptr<Point2d>> points_;

        /** 对每一个edge的保存，此处的edge的key是经过自定以的 **/
        std::unordered_map<int, std::shared_ptr<Edge>> edges_;

        /** 对每一张图片的定义，此处的key是对应初始化时相机的key **/
        std::unordered_map<int, Image> images_;

        /** 可能以后会有用的一个场景图 **/
        std::unordered_multimap<int, int> scene_graph_;
    public:
        CorrespondenceGraph() = default;

        explicit CorrespondenceGraph(CorrespondenceGraph *_graph);

        explicit CorrespondenceGraph(std::vector<std::shared_ptr<CameraModel>> &cameras);

        ~CorrespondenceGraph() = default;

        void IncreaseSearch();

        void BuildPointKey();

        void GetInliers(int index,
                        std::vector<cv::Point2f> &clean_points1,
                        std::vector<cv::Point2f> &clean_points2);

        void FindTransitiveCorrespondences(int point_key, int camera_key);

        void ComputeScore(int camera_key);

        int GetEdgeSize();

        int GetBestBeginPair(int &second_max);

        int GetBestNextImage();

        std::shared_ptr<CameraModel> GetCameraModel(int edge_key, CameraChoice choice);

        std::shared_ptr<CameraModel> GetCameraModel(int camera_key);

        std::shared_ptr<Edge> GetEdge(int index);

        std::unordered_map<int, std::shared_ptr<Edge>> &GetEdges();

        int ComputePointKey(int camera_key, int point_index);

        static inline int ComputeEdgeKey(int camera1, int camera2);

        int ComputeWorldPointKey(int camera1, int camera2, int index);

        void AddWorldPoints(int camera1, int camera2, int index, const std::shared_ptr<Point3d>& point_ptr);

        void RebuildPointRelation(int point_key);

        bool GetRelatedPoints(int camera_key,
                              std::vector<cv::Point3f> &world_points,
                              std::vector<cv::Point2f> &image_points);

        void SetImageRegistered(int index);

        bool ImageHaveRegistered(int index);

        void SetPairJoined(int index);
    };
}

#endif //TEST_CORRESPONDENCE_GRAPH_H
