//
// Created by dcr on 22-7-24.
//

#include "correspondence_graph.h"

#include <utility>

namespace sfm {
    CorrespondenceGraph::CorrespondenceGraph(std::vector<std::shared_ptr<CameraModel>> &cameras) {
        for (int i = 0; i < IMAGE_NUMBER; ++i) {
            for (int j = i + 1; j < IMAGE_NUMBER; ++j) {
                if (cameras[i]->key_ == cameras[j]->key_) {
                    continue;
                }
                int key = ComputeEdgeKey(cameras[i]->key_, cameras[j]->key_);
                scene_graph_.insert(std::pair<int, int>(key / 100, key % 100));
                edges_.insert(std::pair<int, std::shared_ptr<Edge>>
                                      (key, std::make_shared<Edge>(cameras[i], cameras[j])));
                edges_.at(key)->key_ = key;
            }
        }
        std::cout << "Have found " << edges_.size() << " edges" << std::endl;
        BuildPointKey();
    }

    CorrespondenceGraph::CorrespondenceGraph(CorrespondenceGraph *_graph) {
        this->images_ = std::move(_graph->images_);
        this->edges_ = std::move(_graph->edges_);
        this->points_ = std::move(_graph->points_);
        this->scene_graph_ = std::move(_graph->scene_graph_);
    }

    int CorrespondenceGraph::ComputePointKey(int camera_key, int point_index) {
        int temp = sfm::Point2d::ComputePointKey(camera_key, point_index);
        return temp;
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
            // 目前不需要迭代搜索，因为查找方式本质上还是穷举查找，在之后数据集增加后可能需要改变，同时存在外点的干扰
            // TODO 这里有对外点的过滤问题
            for (const auto &match: edge.second->matches_) {
                int point_key1 = ComputePointKey(camera1, match.queryIdx);
                int point_key2 = ComputePointKey(camera2, match.trainIdx);
                if (points_.find(point_key1) == points_.end()) {
                    Eigen::Vector2d temp_point;
                    temp_point
                            << edge.second->key_points_1_[match.queryIdx].x, edge.second->key_points_1_[match.queryIdx].y;
                    std::shared_ptr<Point2d> temp = std::make_shared<Point2d>(match.queryIdx, temp_point);
                    points_.insert(std::pair<int, std::shared_ptr<Point2d>>(point_key1, temp));
                }
                points_.at(point_key1)->AddCorrPoint(camera2, match.trainIdx);
                if (points_.find(point_key2) == points_.end()) {
                    Eigen::Vector2d temp_point;
                    temp_point
                            << edge.second->key_points_2_[match.trainIdx].x, edge.second->key_points_1_[match.trainIdx].y;
                    std::shared_ptr<Point2d> temp = std::make_shared<Point2d>(match.trainIdx, temp_point);
                    points_.insert(std::pair<int, std::shared_ptr<Point2d>>(point_key2, temp));
                }
                points_.at(point_key2)->AddCorrPoint(camera1, match.queryIdx);
            }
        }
    }

    void CorrespondenceGraph::FindTransitiveCorrespondences(int point_key, int camera_key) {

    }

    int CorrespondenceGraph::GetEdgeSize() {
        return this->edges_.size();
    }

    int CorrespondenceGraph::GetBestBeginPair(int &second_max) {
        int index = 0;
        unsigned max = 0;
        second_max = max;
        for (const auto &edge: edges_) {
            int camera1 = edge.first / 100;
            int camera2 = edge.first % 100;
            unsigned int local_max =
                    images_.at(camera1).correspondence_number + images_.at(camera2).correspondence_number;
            if (local_max > max) {
                second_max = max;
                max = local_max;
                index = edge.first;
            }
        }
        edges_[index]->joined = true;
        return index;
    }

    std::shared_ptr<CameraModel> CorrespondenceGraph::GetCameraModel(int edge_key, CameraChoice choice) {
        if (choice == CameraChoice::kCamera1) {
            return this->edges_[edge_key]->camera1_;
        } else {
            return this->edges_[edge_key]->camera2_;
        }
    }

    std::shared_ptr<Edge> CorrespondenceGraph::GetEdge(int index) {
        if (edges_.find(index) == edges_.end()) {
            std::cerr << "Unable to find the target edge" << std::endl;
            std::cerr << "OUT OF RANGE" << std::endl;
        }
        return this->edges_.at(index);
    }

    /**
     * 目前这里先不写场景图的增强，先写个找下一个最大覆盖的
     * 这里先不用了，毕竟是穷举，可以只写一个按照序列加入edge
     * @param new_camera 选择出的下一个选出的照片
     * @param last_camera 上一次加入优化的图片
     * @param last_max 上一次计算出的待定加入的最大值
     * @return
     */
    int CorrespondenceGraph::GetBestNextImage() {
        if (joined_number_ == IMAGE_NUMBER) {
            return -1;
        }
        joined_number_++;
        unsigned max_score = 0;
        int index = -1;
        for (const auto &image: images_) {
            if (!image.second.registered) {
                if (image.second.score > max_score) {
                    max_score = image.second.score;
                    index = image.first;
                }
            }
        }
        return index;
    }

    /**
     * 用F过滤掉全部的外点
     * @param index
     * @param clean_points1
     * @param clean_points2
     */
    void CorrespondenceGraph::GetInliers(int index,
                                         std::vector<cv::Point2f> &clean_points1,
                                         std::vector<cv::Point2f> &clean_points2) {
        auto edge = edges_.at(index);
        auto point1 = edge->key_points_1_.begin();
        auto point2 = edge->key_points_2_.begin();
        int sum = 0;
        for (; point1 != edge->key_points_1_.end() && point2 != edge->key_points_2_.end(); point2++, point1++) {
            cv::Mat temp_point2 = (cv::Mat_<double>(3, 1) << point2->x, point2->y, 1.0f);
            cv::Mat temp_point1 = (cv::Mat_<double>(3, 1) << point1->x, point1->y, 1.0f);
            cv::Mat check = temp_point2.t() * edge->f_m_ * temp_point1;
            if (check.at<double>(0, 0) <= FUNDAMENTAL_INLIER_THRESHOLD) {
                sum++;
                clean_points1.emplace_back(point1->x, point1->y);
                clean_points2.emplace_back(point2->x, point2->y);
            }
        }
        std::cout << "The initial inliers is " << sum << std::endl;
    }

    /**
     * 计划是计算对应世界点的hash的key，目前感觉还不需要，没有完全完成
     * @param camera1
     * @param camera2
     * @param index
     * @return
     */
    int CorrespondenceGraph::ComputeWorldPointKey(int camera1, int camera2, int index) {
        int camera_key = ComputePointKey(camera1, camera2);
        int key1 = ComputePointKey(camera1, edges_[camera_key]->matches_[index].queryIdx);
        int key2 = ComputePointKey(camera2, edges_[camera_key]->matches_[index].trainIdx);
        if (points_.at(key1)->GetCorrNumber() != points_.at(key2)->GetCorrNumber()) {
            std::cerr << "Error occurred the two related points correspondence is not equal" << std::endl;
            return -1;
        } else {
            return points_.at(key1)->ComputePointKey(camera1, edges_[camera_key]->matches_[index].queryIdx);
        }
    }

    /**
     * 将计算出的世界点与二维点联系上
     * @param camera1
     * @param camera2
     * @param index
     * @param point_ptr
     */
    void CorrespondenceGraph::AddWorldPoints(int camera1, int camera2, int index,
                                             const std::shared_ptr<Point3d> &point_ptr) {
        int camera_key = ComputeEdgeKey(camera1, camera2);
        int point_key1 = ComputePointKey(camera1, edges_.at(camera_key)->matches_[index].queryIdx);
        int point_key2 = ComputePointKey(camera2, edges_.at(camera_key)->matches_[index].trainIdx);
        int size1 = points_.at(point_key1)->GetCorrNumber();
        int size2 = points_.at(point_key2)->GetCorrNumber();
//        if (points_.at(point_key1)->GetCorrNumber() != points_.at(point_key2)->GetCorrNumber()) {
//            std::cerr << "The size of two corr is not equal" << std::endl;
//            std::cerr << "There are outliers in the match" << std::endl;
//            std::cerr << points_.at(point_key1)->GetCorrNumber() << ' ' << points_.at(point_key2)->GetCorrNumber()
//                      << std::endl << std::endl;
//        }
        if (size1 == 0 || size2 == 0) {
            std::cerr << "The size is zero" << std::endl;
        }
        points_.at(point_key1)->AddWorldPoints(point_ptr);
        std::vector<int> corrs1;
        std::vector<int> corrs2;
        corrs1.resize(size1);
        corrs2.resize(size2);
        points_.at(point_key1)->GetCorrs(corrs1);
        points_.at(point_key2)->GetCorrs(corrs2);
        for (int i = 0; i < size1; ++i) {
            points_.at(corrs1[i])->AddWorldPoints(point_ptr);
        }
        for (int i = 0; i < size2; ++i) {
            points_.at(corrs2[i])->AddWorldPoints(point_ptr);
        }
    }

    void CorrespondenceGraph::AddWorldPoints(int camera_key,
                                             const std::vector<int> &corrs,
                                             std::shared_ptr<Point3d> world_ptr) {
        points_.at(camera_key)->AddWorldPoints(world_ptr);
        for (const auto &corr: corrs) {
            points_.at(corr)->AddWorldPoints(world_ptr);
        }
    }

    void CorrespondenceGraph::RebuildPointRelation(int point_key) {

    }

    void CorrespondenceGraph::SetPairJoined(int index) {
        this->edges_[index]->joined = true;
    }

    std::unordered_map<int, std::shared_ptr<Edge>> &CorrespondenceGraph::GetEdges() {
        return edges_;
    }

    void CorrespondenceGraph::SetImageRegistered(int index) {
        images_.at(index).registered = true;
    }

    bool CorrespondenceGraph::ImageHasRegistered(int index) {
        return images_.at(index).registered;
    }

    bool CorrespondenceGraph::GetRelatedPoints(int camera_key,
                                               std::vector<cv::Point3f> &world_points,
                                               std::vector<cv::Point2f> &image_points) {
        int related_number = 0;
        for (const auto &point: points_) {
            if (point.first / 100000 == camera_key) {
                if (point.second->HasRegistered()) {
                    point.second->AddRelatedPoint(world_points, image_points);
                    related_number++;
                }
            }
        }
        if (related_number == 0) {
            return false;
        }
        return true;
    }

    /**
     * 计算每个图片的score，计算目前新加入的图片与没有加入的图片之间的score
     * @param camera_key
     */
    void CorrespondenceGraph::ComputeScore(int camera_key) {
        for (auto &image: images_) {
            if (image.first != camera_key) {
                int key = ComputeEdgeKey(image.first, camera_key);
                image.second.score += edges_.at(key)->matches_.size();
            }
        }
    }

    std::shared_ptr<CameraModel> CorrespondenceGraph::GetCameraModel(int camera_key) {
        for (const auto &image: images_) {
            if (image.first != camera_key) {
                int key = ComputeEdgeKey(image.first, camera_key);
                if (camera_key == key / 100) {
                    return edges_.at(key)->camera1_;
                } else {
                    return edges_.at(key)->camera2_;
                }
            }
        }
        return nullptr;
    }

    bool CorrespondenceGraph::PointHasRegistered(int point_key) {
        return points_.at(point_key)->HasRegistered();
    }

    void CorrespondenceGraph::GetP(int camera_key, Eigen::Matrix<double, 3, 4> &P) {
        auto camera = GetCameraModel(camera_key);
        cv::Mat temp_P = camera->K_ * camera->T_;
        cv::cv2eigen(temp_P, P);
    }

    bool CorrespondenceGraph::GetPixelPoint(int point_key, Eigen::Vector2d &point) {
        if (points_.find(point_key) == points_.end()) {
            std::cerr << "Unable to find the target point" << std::endl;
            return false;
        }
        points_.at(point_key)->GetPixelPoint(point);
        return true;
    }

    int CorrespondenceGraph::GetCameraKeyByPoint(int point_key) {
        return point_key / 100000;
    }

    void CorrespondenceGraph::GetCameraPoints(std::unordered_map<int, std::vector<std::shared_ptr<Point2d>>> &points) {
        int camera;
        for (int i = 0; i < IMAGE_NUMBER; ++i) {
            std::vector<std::shared_ptr<Point2d>> temp;
            points.insert(std::pair<int, std::vector<std::shared_ptr<Point2d>>>(i, temp));
        }
        for (const auto &point: points_) {
            if (point.second->HasRegistered()) {
                camera = point.first / 100000;
                points.at(camera).push_back(point.second);
            }
        }
    }
}
