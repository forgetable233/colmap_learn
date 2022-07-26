//
// Created by dcr on 22-7-24.
//

#include "correspondence_graph.h"

namespace sfm {
    CorrespondenceGraph::CorrespondenceGraph(std::vector<std::shared_ptr<CameraModel>> &cameras) {
        for (int i = 0; i < IMAGE_NUMBER; ++i) {
            for (int j = i + 1; j < IMAGE_NUMBER; ++j) {
                if (i == j) {
                    continue;
                }
                int key = ComputeEdgeKey(i, j);
                scene_graph_.insert(std::pair<int, int>(key / 100, key % 100));
                edges_.insert(std::pair<int, std::shared_ptr<Edge>>
                                      (key, std::make_shared<Edge>(cameras[i], cameras[j])));
            }
        }
        std::cout << "Have found " << edges_.size() << "edges" << std::endl;
        BuildPointKey();
//        IncreaseSearch();
    }

    CorrespondenceGraph::CorrespondenceGraph(CorrespondenceGraph *_graph) {
        this->images_ = std::move(_graph->images_);
        this->edges_ = std::move(_graph->edges_);
        this->points_ = std::move(_graph->points_);
        this->scene_graph_ = std::move(_graph->scene_graph_);
    }

    inline int CorrespondenceGraph::ComputePointKey(int camera_key, int point_index) {
        return camera_key * 10000 + point_index;
    }

    inline int CorrespondenceGraph::ComputeEdgeKey(int camera1, int camera2) {
        int max = camera1 > camera2 ? camera1 : camera2;
        int min = camera1 + camera2 - max;
        return max * 100 + min;
    }

    // 对每个点构建对应的所有的相关性
    // 这个目前还不是很重要
    void CorrespondenceGraph::IncreaseSearch() {
        for (auto &pair: scene_graph_) {
            // 此时这两张图片之间的关系已经建立
            // 下面需要迭代搜索所有的相关点
            int camera1 = pair.first;
            int camera2 = pair.second;
            auto next_graph = scene_graph_.find(camera2);
            while (next_graph != scene_graph_.end()) {
                int key = ComputeEdgeKey(camera2, next_graph->second);
                for (const auto &match: edges_[key]->matches_) {
                    // 这个的位置是固定的，queryIndx和trainIdx在两个edge下是相同的
                    int temp_point_key = ComputePointKey(camera2, match.queryIdx);

                }
            }
        }
    }

    // 保存内容有三层，分别是key（图片与位置），图片中的索引，对应的点的坐标
    // 此处已经完成构建相关性，IncreaseBuild目前不需要
    void CorrespondenceGraph::BuildPointKey() {
        for (const auto &edge: edges_) {
            int camera1 = edge.second->camera1_->key_;
            int camera2 = edge.second->camera2_->key_;
            if (images_.find(camera1) == images_.end()) {
                images_.insert(std::pair<int, Image>(camera1, Image(0)));
            }
            if (images_.find(camera2) == images_.end()) {
                images_.insert(std::pair<int, Image>(camera2, Image(0)));
            }
            images_.at(camera1).correspondence_number += edge.second->matches_.size();
            images_.at(camera2).correspondence_number += edge.second->matches_.size();
            for (const auto &match: edge.second->matches_) {
                int point_key1 = ComputePointKey(camera1, match.queryIdx);
                int point_key2 = ComputePointKey(camera2, match.trainIdx);
                if (points_.find(point_key1) == points_.end()) {
                    Eigen::Vector2d temp_point;
                    temp_point
                            << edge.second->key_points_1_[match.queryIdx].x, edge.second->key_points_1_[match.queryIdx].y;
                    std::shared_ptr<Point2d> temp = std::make_shared<Point2d>(match.queryIdx, temp_point);
                    points_.insert(std::pair<int, std::shared_ptr<Point2d>>(point_key1, temp));
                    points_[point_key1]->AddCorrPoint(camera2, match.trainIdx);
                    images_.at(camera1).correspondence_number++;
                } else {
                    images_.at(camera1).correspondence_number++;
                    points_[point_key1]->AddCorrPoint(camera2, match.trainIdx);
                }
                if (points_.find(point_key2) == points_.end()) {
                    Eigen::Vector2d temp_point;
                    temp_point
                            << edge.second->key_points_2_[match.trainIdx].x, edge.second->key_points_1_[match.trainIdx].y;
                    std::shared_ptr<Point2d> temp = std::make_shared<Point2d>(match.trainIdx, temp_point);
                    points_.insert(std::pair<int, std::shared_ptr<Point2d>>(point_key2, temp));
                    points_[point_key2]->AddCorrPoint(camera1, match.queryIdx);
                    images_.at(camera2).correspondence_number++;
                } else {
                    points_[point_key2]->AddCorrPoint(camera1, match.queryIdx);
                    images_.at(camera2).correspondence_number++;
                }
            }
        }
    }

    void CorrespondenceGraph::FindTransitiveCorrespondences(int point_key, int camera_key) {

    }

    int CorrespondenceGraph::GetEdgeSize() {
        return this->edges_.size();
    }

    int CorrespondenceGraph::GetBestBeginPair() {
        int index = 0;
        unsigned max = 0;
        for (const auto &edge: edges_) {
            int camera1 = edge.first / 100;
            int camera2 = edge.first % 100;
            unsigned int local_max =
                    images_.at(camera1).correspondence_number + images_.at(camera2).correspondence_number;
            if (local_max > max) {
                max = local_max;
                index = edge.first;
            }
        }
        edges_[index]->joined = true;
        return index;
    }

    std::shared_ptr<CameraModel> CorrespondenceGraph::GetCameraModel(int index, CameraChoice choice) {
        if (choice == CameraChoice::kCamera1) {
            return this->edges_[index]->camera1_;
        } else {
            return this->edges_[index]->camera2_;
        }
    }

    std::shared_ptr<Edge> CorrespondenceGraph::GetEdge(int index) {
        return this->edges_[index];
    }

    // 目前这里先不写场景图的增强，先写个找下一个最大覆盖的
    int CorrespondenceGraph::GetNextBestPair() {
        if (joined_number_ == edges_.size()) {
            return -1;
        }
        joined_number_++;
        int index = -1;
        int max = -1;
        for (const auto &edge: edges_) {
            if (!edge.second->joined) {
                int camera1 = edge.first / 100;
                int camera2 = edge.first % 100;
                unsigned int local_max =
                        images_.at(camera1).correspondence_number + images_.at(camera2).correspondence_number;
                if (local_max > max) {
                    max = local_max;
                    index = edge.first;
                }
            }
        }
        return index;
    }

    void CorrespondenceGraph::GetInliers(int index,
                                         std::vector<cv::Point2f> &clean_points1,
                                         std::vector<cv::Point2f> &clean_points2) {
        auto edge = edges_.at(index);
        auto point1 = edge->key_points_1_.begin();
        auto point2 = edge->key_points_2_.begin();
        for (; point1 != edge->key_points_1_.end() && point2 != edge->key_points_2_.end(); point2++, point1++) {
            cv::Mat temp_point2 = (cv::Mat_<double>(3, 1) << point2->x, point2->y, 1.0f);
            cv::Mat temp_point1 = (cv::Mat_<double>(3, 1) << point1->x, point1->y, 1.0f);
            cv::Mat check = temp_point2.t() * edge->f_m_ * temp_point1;
            if (check.at<double>(0, 0) <= FUNDAMENTAL_INLIER_THRESHOLD) {
                clean_points1.emplace_back(point1->x, point1->y);
                clean_points2.emplace_back(point2->x, point2->y);
            }
        }
    }
}
