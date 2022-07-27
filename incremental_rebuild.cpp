//
// Created by dcr on 22-7-15.
//

#include "incremental_rebuild.h"

namespace sfm {

    IncrementalRebuild::IncrementalRebuild(CorrespondenceGraph *_graph) {
        scene_graph_ = std::make_shared<CorrespondenceGraph>(_graph);
    }

    void IncrementalRebuild::BeginRebuild() {
        int max = 0;
        int second_max = 0;
        int current_pair = this->GetBestBeginEdge(second_max);
        int last_pair = current_pair;

        std::shared_ptr<CameraModel> camera1;
        std::shared_ptr<CameraModel> camera2;

        std::shared_ptr<std::vector<cv::Point3f>> world_point = std::make_shared<std::vector<cv::Point3f>>();

        if (Init(current_pair)) {
            std::cout << "Have finished the initialize" << std::endl;
//            ViewAllPoints();
        } else {
            std::cerr << "Error occurred unable to initialize" << std::endl;
        }
        /*while ((current_pair = scene_graph_->GetBestNextPair()) != -1) {
            last_pair = current_pair;
            auto next_best_edge = scene_graph_->GetEdge(current_pair);
        }*/
        auto edges = scene_graph_->GetEdges();
        for (const auto &edge: edges) {
            if (!edge.second->joined) {
                Eigen::Matrix3d R;
                Eigen::Vector3d t;
                RegisterImage(R, t, edge.second->key_);
//                Triangulation(edge.second->key_, kNormal);
            }
        }
        std::cout << "Have finished the rebuild" << std::endl;
    }

    int
    IncrementalRebuild::GetNextBestEdge(const std::shared_ptr<bool[]> &used_list,
                                        const std::shared_ptr<bool[]> &joined_list,
                                        const std::shared_ptr<std::vector<int>> &index_list,
                                        int last_edge_index) {
        /** 这里需要进行场景图的增强操作，待定目前先用个简单的方法进行搜索 **/
        return -1;
//        return scene_graph_->GetBestNextPair();
    }

    int IncrementalRebuild::GetBestBeginEdge(int &second_max) {
        return scene_graph_->GetBestBeginPair(second_max);
    }

    bool IncrementalRebuild::Init(int index) {
        this->Triangulation(index, kInitial);
        return true;
    }

    void IncrementalRebuild::PixToCam(cv::Mat &K,
                                      std::vector<cv::Point2f> &input_points,
                                      std::vector<cv::Point2f> &output_points) {
        if (K.cols != 3 || K.rows != 3) {
            std::cerr << "the col or the row of the input K isn't equal to 3" << std::endl;
            return;
        }
        cv::Mat inverted_K;
        cv::invert(K, inverted_K);

        for (auto point: input_points) {
            cv::Mat temp_input_point = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0f);
            cv::Mat temp_output_point = inverted_K * temp_input_point;
            output_points.emplace_back(temp_output_point.at<double>(0, 0),
                                       temp_output_point.at<double>(1, 0));
        }
    }

    void IncrementalRebuild::Triangulation(int index, TrianguleType type) {
        auto camera1 = scene_graph_->GetCameraModel(index, CameraChoice::kCamera1);
        auto camera2 = scene_graph_->GetCameraModel(index, CameraChoice::kCamera2);
        auto edge = scene_graph_->GetEdge(index);

        if (camera1 == nullptr || camera2 == nullptr || camera2 == camera1) {
            std::cerr << "Fail to find the target camera" << std::endl;
            return;
        }
        if (type == kInitial) {
            // initial the pose of the second camera
            camera2->SetCameraPose(edge->R_, edge->t_);
            scene_graph_->SetImageRegistered(camera1->key_);
            scene_graph_->SetImageRegistered(camera2->key_);
            /** 坐标系没有发生变化 **/
            camera1->T_.at<double>(0, 0) = 1.0f;
            camera1->T_.at<double>(1, 1) = 1.0f;
            camera1->T_.at<double>(2, 2) = 1.0f;
            camera2->T_.at<double>(0, 3) = camera2->T_.at<double>(0, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(1, 3) = camera2->T_.at<double>(1, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(2, 3) = 1.0f;
            /** 进行清外点以及向相机坐标系的转变 **/
            std::vector<cv::Point2f> clean_point_1;
            std::vector<cv::Point2f> clean_point_2;
            std::vector<cv::Point2f> camera_point_1;
            std::vector<cv::Point2f> camera_point_2;
            std::vector<Eigen::Vector3d> world_points;

            CleanOutliers(index, clean_point_1, clean_point_2);

            /** 将像素坐标转化到相机坐标系下 **/
            PixToCam(camera1->K_, clean_point_1, camera_point_1);
            PixToCam(camera2->K_, clean_point_2, camera_point_2);

            cv::Mat pst_4d;

            cv::triangulatePoints(camera1->T_, camera2->T_,
                                  camera_point_1, camera_point_2,
                                  pst_4d);
            CheckZDepthAndAddWorldPoints(camera1, camera2, pst_4d);
            std::cout << "Have finished the initial triangulation " << std::endl;
            scene_graph_->SetPairJoined(index);
            /** 针对完成三角化的点进行索引的构建 **/
        } else if (type == kNormal) {
            if (scene_graph_->ImageHaveRegistered(camera1->key_)) {
                // 确定第一张图片没有加入进去
            } else {

            }
        }
    }

    void IncrementalRebuild::CleanOutliers(int index,
                                           std::vector<cv::Point2f> &clean_points_1,
                                           std::vector<cv::Point2f> &clean_points_2) {
        scene_graph_->GetInliers(index, clean_points_1, clean_points_2);
    }

    void IncrementalRebuild::CheckZDepthAndAddWorldPoints(const std::shared_ptr<CameraModel> &camera1,
                                                          const std::shared_ptr<CameraModel> &camera2,
                                                          cv::Mat &pst_4d) {
        cv::Mat P1 = camera1->K_ * camera1->T_;
        cv::Mat P2 = camera2->K_ * camera2->T_;
        cv::Mat temp_point = (cv::Mat_<double>(4, 1) << 0.0f, 0.0f, 0.0f, 1.0f);
        cv::Mat check1;
        cv::Mat check2;
        Eigen::Vector3d temp;
        int key = 0;
        int sum = 0;
        int negative = 0;
        for (int i = 0; i < pst_4d.cols; ++i) {
            temp_point.at<double>(0, 0) = pst_4d.at<float>(0, i);
            temp_point.at<double>(1, 0) = pst_4d.at<float>(1, i);
            temp_point.at<double>(2, 0) = pst_4d.at<float>(2, i);
            temp_point.at<double>(3, 0) = pst_4d.at<float>(3, i);
            check1 = P1.row(2) * temp_point;
            check2 = P2.row(2) * temp_point;
            if (check1.at<double>(0, 0) >= 0 && check2.at<double>(0, 0) >= 0) {
                sum++;
                temp.x() = temp_point.at<double>(0, 0) / temp_point.at<double>(3, 0);
                temp.y() = temp_point.at<double>(1, 0) / temp_point.at<double>(3, 0);
                temp.z() = temp_point.at<double>(2, 0) / temp_point.at<double>(3, 0);
                std::shared_ptr<Point3d> world_ptr = std::make_shared<Point3d>(temp);
                // 将所有的相关点都加入到对应的相关点中
                scene_graph_->AddWorldPoints(camera1->key_, camera2->key_, i, world_ptr);
                key = ComputeWorldPointKey();
                world_points_.insert(std::pair<int, std::shared_ptr<Point3d>>(key, nullptr));
                world_points_.at(key) = world_ptr;
            }
        }
        std::cout << "The size of the initial points is " << world_points_.size() << std::endl;
    }

    void IncrementalRebuild::ViewAllPoints() {
        std::vector<Eigen::Vector3d> world_points;
        Eigen::Vector3d temp_point;
        for (const auto &point: world_points_) {
            temp_point = point.second->GetPoint();
            world_points.emplace_back(temp_point.x(), temp_point.y(), temp_point.z());
        }
        if (world_points.empty()) {
            std::cerr << "The size of the world points is zero" << std::endl;
        } else {
            std::shared_ptr<PointViewer> viewer = std::make_shared<PointViewer>(world_points);
            viewer->ViewPoints();
        }
    }

    int IncrementalRebuild::ComputeWorldPointKey() {
        return this->world_points_.size() + 1;
    }

    void IncrementalRebuild::ShowMatchResult(int begin_index) {

    }

    /**
     * 使用PnP算法计算出对应图像的世界坐标位置（这里预计会用自己写的方法计算一下）
     * @param R
     * @param t
     * @param edge_key
     */
    void IncrementalRebuild::RegisterImage(Eigen::Matrix3d &R, Eigen::Vector3d &t, int edge_key) {
        auto camera1 = scene_graph_->GetCameraModel(edge_key, CameraChoice::kCamera1);
        auto camera2 = scene_graph_->GetCameraModel(edge_key, CameraChoice::kCamera2);
        auto edge = scene_graph_->GetEdge(edge_key);
        cv::Mat R_;
        cv::Mat t_;
        cv::Mat dist_coeff = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
        std::vector<cv::Point3f> world_points;
        std::vector<cv::Point2f> image_points;
        if (scene_graph_->ImageHaveRegistered(camera1->key_)) {
            // 确定是第二张图片没有注册进去，随后计算得到所有的对应以及完成三角化的点
            scene_graph_->GetRelatedPoints(camera2->key_, world_points, image_points);
        } else if (scene_graph_->ImageHaveRegistered(camera2->key_)){
            scene_graph_->GetRelatedPoints(camera1->key_, world_points, image_points);
        } else {
            std::cerr << "all the images in this has not been registered" << std::endl;
            return;
        }
        std::cout << world_points.size() << ' ' << image_points.size() << std::endl;
        cv::solvePnPRansac(world_points, image_points, camera2->K_, dist_coeff, R_, t_);
        std::cout << R_ << std::endl << t_ << std::endl << std::endl;
    }
}