//
// Created by dcr on 22-7-24.
//

#ifndef TEST_CORRESPONSE_GRAPH_H
#define TEST_CORRESPONSE_GRAPH_H
#include <unordered_map>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "edge.h"
#include "point2d.h"
#include "threholds.h"

namespace sfm {
    class CorrespondenceGraph {
    private:
        // 保存每一张图片的相关性，用来选择最佳起始图片
        struct Image {
            explicit Image(unsigned int _corr): correspondence_number(_corr){}

            unsigned int correspondence_number = 0;
        };
        // 此处为对所有的点构建对应的key，以方便进行查找
        std::unordered_map<int, std::shared_ptr<Point2d>> points_;

        std::unordered_map<int, std::shared_ptr<Edge>> edges_;

        std::unordered_map<int, Image> images_;
    public:
        CorrespondenceGraph() = default;

        explicit CorrespondenceGraph(std::vector<std::shared_ptr<CameraModel>> &cameras);

        ~CorrespondenceGraph() = default;

        void IncreaseSearch();

        void BuildPointKey();

        static inline int ComputePointKey(int camera_key, int point_index);

        static inline int ComputeEdgeKey(int camera1, int camera2);
    };
}

#endif //TEST_CORRESPONSE_GRAPH_H
