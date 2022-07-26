//
// Created by dcr on 22-7-15.
//

#include "incremental_rebuild.h"

namespace sfm {

    IncrementalRebuild::IncrementalRebuild(CorrespondenceGraph *_graph) {
        scene_graph_ = std::make_shared<CorrespondenceGraph>(_graph);
    }

    void IncrementalRebuild::BeginRebuild() {
        int begin_index = this->GetBestBeginEdge();
        int max = 0;
        int joined_number = 1;
        int next_best_index = begin_index;
        int last_pair = begin_index;

        std::shared_ptr<CameraModel> camera1;
        std::shared_ptr<CameraModel> camera2;

        std::shared_ptr<std::vector<cv::Point3f>> world_point = std::make_shared<std::vector<cv::Point3f>>();

        if (Init(begin_index)) {
            std::cout << "Have finished the initialize" << std::endl;
        } else {
            std::cerr << "Error occurred unable to initialize" << std::endl;
        }
//        while ((next_best_index = scene_graph_->GetNextBestPair()) != -1) {
//            last_pair = next_best_index;
//            auto next_best = scene_graph_->GetEdge(next_best_index);
//        }
//        std::cout << "Have finished the rebuild" << std::endl;
    }

    int
    IncrementalRebuild::GetNextBestEdge(const std::shared_ptr<bool[]> &used_list,
                                        const std::shared_ptr<bool[]> &joined_list,
                                        const std::shared_ptr<std::vector<int>> &index_list,
                                        int last_edge_index) {
        /** 这里需要进行场景图的增强操作，待定目前先用个简单的方法进行搜索 **/
        return scene_graph_->GetNextBestPair();
    }

    int IncrementalRebuild::GetBestBeginEdge() {
        return scene_graph_->GetBestBeginPair();
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
            /** 针对完成三角化的点进行索引的构建 **/
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
        int sum = 0;
        int key = 0;
        for (int i = 0; i < pst_4d.cols; ++i) {
            temp_point.at<double>(0, 0) = pst_4d.at<double>(0, i);
            temp_point.at<double>(1, 0) = pst_4d.at<double>(1, i);
            temp_point.at<double>(2, 0) = pst_4d.at<double>(2, i);
            temp_point.at<double>(3, 0) = pst_4d.at<double>(3, i);
            check1 = P1.row(2) * temp_point;
            check2 = P2.row(2) * temp_point;
            if (check1.at<double>(0, 0) >=0 && check2.at<double>(0, 0) >= 0) {
                sum++;
                temp.x() = temp_point.at<double>(0, 0) / temp_point.at<double>(3, 0);
                temp.y() = temp_point.at<double>(1, 0) / temp_point.at<double>(3, 0);
                temp.z() = temp_point.at<double>(2, 0) / temp_point.at<double>(3, 0);
                std::shared_ptr<Point3d> world_ptr = std::make_shared<Point3d>(temp);
                scene_graph_->AddWorldPoints(camera1->key_, camera2->key_, i, world_ptr);
            }
        }
        std::cout << "The size of the initial points is " << sum << std::endl;
    }
}