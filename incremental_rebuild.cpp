//
// Created by dcr on 22-7-15.
//

#include "incremental_rebuild.h"

namespace sfm {

    IncrementalRebuild::IncrementalRebuild(const std::vector<std::shared_ptr<Edge>> &_edges,
                                           const std::shared_ptr<Points> &_points,
                                           const int (*_scene_graph)[10]) {
        this->edges_.resize(_edges.size());
        std::copy(_edges.begin(), _edges.end(), this->edges_.begin());
        this->points_ = _points;
        for (int i = 0; i < IMAGE_NUMBER; ++i) {
            for (int j = 0; j < IMAGE_NUMBER; ++j) {
                this->scene_graph[i][j] = _scene_graph[i][j];
            }
        }
    }

    void IncrementalRebuild::BeginRebuild() {
        Init();
    }

    int IncrementalRebuild::GetBestBeginEdge() {
        int index = 0;
        int max = edges_.begin()->get()->f_m_inliers_;
        for (int i = 0; i < edges_.size(); ++i) {
            if (edges_[i].get()->f_m_inliers_ > max) {
                index = i;
            }
        }
        return index;
    }

    void IncrementalRebuild::Init() {
        int begin_index = this->GetBestBeginEdge();
        int next_best_index;

//        std::unique_ptr<bool> index_list{new bool []};
        this->Triangulation(begin_index, kInitial);

    }

    void IncrementalRebuild::PixToCam(cv::Mat &K, std::vector<cv::Point2f> &input_points,
                                      std::vector<cv::Point2f> &output_points) {
        if (K.cols != 3 || K.rows != 3) {
            std::cerr << "the col or the row of the input K isn't equal to 3" << std::endl;
            return;
        }
        cv::Mat inverted_K;
        cv::invert(K, inverted_K);

        for (auto point: input_points) {
            cv::Mat temp_input_point = (cv::Mat_<float>(3, 1) << point.x, point.y, 1.0f);
            cv::Mat temp_output_point = inverted_K * temp_input_point;
            output_points.emplace_back(temp_output_point.at<float>(0, 0),
                                       temp_output_point.at<float>(1, 0));
        }
    }

    void IncrementalRebuild::Triangulation(int index, TrianguleType type) {
        std::shared_ptr<CameraModel> camera1 = this->edges_[index]->camera1_;
        std::shared_ptr<CameraModel> camera2 = this->edges_[index]->camera2_;
        std::shared_ptr<Edge> edge = this->edges_[index];

        if (camera1 == nullptr || camera2 == nullptr || camera2 == camera1) {
            std::cerr << "Fail to find the target camera" << std::endl;
            return;
        }
        if (type == kInitial) {
            // initial the pose of the second camera
            camera2->SetCameraPose(edge->R_, edge->t_);
            camera2->T_.at<double>(0, 3) = camera2->T_.at<double>(0, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(1, 3) =
                    camera2->T_.at<double>(1, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(2, 3) = 1.0f;
            std::cout << "The initial camera1 pose is " << std::endl << camera2->T_ << std::endl << std::endl;
            std::cout << "The initial camera2 pose is " << std::endl << camera2->T_ << std::endl << std::endl;

            /** 进行清外点以及向相机坐标系的转变 **/
            std::vector<cv::Point2f> clean_point_1;
            std::vector<cv::Point2f> clean_point_2;
            std::vector<cv::Point2f> camera_point_1;
            std::vector<cv::Point2f> camera_point_2;

            CleanOutliers(index, clean_point_1, clean_point_2);

            std::vector<cv::Point3f> world_point;

            /** 将像素坐标转化到相机坐标系下 **/
            PixToCam(camera2->K_, clean_point_1, camera_point_1);
            PixToCam(camera2->K_, clean_point_2, camera_point_2);

            cv::Mat pst_4d;

            cv::triangulatePoints(camera2->T_, camera2->T_,
                                  camera_point_1, camera_point_2,
                                  pst_4d);
            std::cout << "Have finished the triangulation " << std::endl;
            for (int i = 0; i < pst_4d.cols; ++i) {
                world_point.emplace_back(cv::Point3f{pst_4d.at<float>(0, i) / pst_4d.at<float>(3, i),
                                                     pst_4d.at<float>(1, i) / pst_4d.at<float>(3, i),
                                                     pst_4d.at<float>(2, i) / pst_4d.at<float>(3, i)});
            }
            std::cout << "Have finished computing the position of the feature points_" << std::endl;
            std::cout << "The size of the world point is " << world_point.size() << std::endl;

            /** 针对完成三角化的点进行索引的构建 **/
            points_->AddCloudPoint(edge, world_point);
        }
    }

    void IncrementalRebuild::CleanOutliers(int index,
                                           std::vector<cv::Point2f> &clean_points_1,
                                           std::vector<cv::Point2f> &clean_points_2) {
        auto temp_edge = this->edges_[index];
        std::cout << temp_edge->key_points_1_.size() << ' ' << temp_edge->key_points_2_.size() << ' ' << std::endl;
        std::cout << temp_edge->point1_pass_.size() << ' ' << temp_edge->point2_pass_.size() << ' ' << std::endl;
        std::cout << temp_edge->points1_index_.size() << ' ' << temp_edge->points2_index_.size() << ' ' << std::endl;
        for (int i = 0; i < temp_edge->key_points_1_.size(); ++i) {
            if (temp_edge->point1_pass_[i] && temp_edge->point2_pass_[i]) {
                clean_points_1.push_back(temp_edge->key_points_1_[i]);
                clean_points_2.push_back(temp_edge->key_points_2_[i]);
            }
        }
    }
}