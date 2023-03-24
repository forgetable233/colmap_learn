#include <iostream>
#include <algorithm>
#include <string>
#include <ctime>

#include "img_loader.h"
#include "camera_model.h"
#include "edge.h"
#include "point2d.h"
#include "math_functions.h"
#include "thresholds.h"
#include "incremental_rebuild.h"
#include "correspondence_graph.h"

struct get_key {
    int key;

    explicit get_key(int _key) : key(_key) {}

    bool operator()(const sfm::CameraModel &camera) const {
        return camera.key_ == key;
    }
};

struct SortEdges {
    bool operator()(const sfm::Edge &edge1, const sfm::Edge &edge2) {
        return edge1.matches_ > edge2.matches_;
    }
} sort_edges;

void CVPoint2iToVector3d(const std::vector<cv::Point2i> &point2i, std::vector<Eigen::Vector3d> &vector3d) {
    for (auto point: point2i) {
        vector3d.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y), 1.0f);
    }
}

/** 构造一个简单的图像树，试一试效果 **/
void BuildSceneGraph(std::vector<std::shared_ptr<sfm::Edge>> &edges,
                     const std::vector<std::shared_ptr<sfm::CameraModel>> &cameras,
                     int scene_graph[IMAGE_NUMBER][IMAGE_NUMBER]) {
    /** 目前先用简单的穷举匹配法，同时计算出一个配对效果出来 **/
    int norm_type = cv::NORM_L2;

    bool cross_check = true;

    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(norm_type, cross_check);
    for (int i = 0; i < IMAGE_NUMBER; ++i) {
        for (int j = 0; j < IMAGE_NUMBER; ++j) {
            scene_graph[i][j] = -1;
        }
    }

    for (int i = 0; i < IMAGE_NUMBER; ++i) {
        for (int j = i + 1; j < IMAGE_NUMBER; ++j) {
            sfm::Edge temp_edge{cameras[i], cameras[j], false};
            if (temp_edge.PassGeometryTest()) {
                edges.push_back(std::make_shared<sfm::Edge>(temp_edge));
                scene_graph[cameras[i]->key_][cameras[j]->key_] = edges.size() - 1;
                scene_graph[cameras[j]->key_][cameras[i]->key_] = edges.size() - 1;
            }
        }
    }
}

double ComputeResidual(std::vector<Eigen::Vector2d> &points1,
                       std::vector<Eigen::Vector2d> &points2,
                       Eigen::Matrix3d &F) {
    int residual = 0;
    if (points1.size() != points2.size()) {
        std::cerr << "The size is inequal" << std::endl;
    }
    const int size = points2.size();
    for (int i = 0; i < size; i++) {
        Eigen::Vector3d temp_point1 = points1[i].homogeneous();
        Eigen::Vector3d temp_point2 = points2[i].homogeneous();
        double local_residual = temp_point2.transpose() * F * temp_point1;
        residual += local_residual * local_residual;
    }
    return residual;
}

void CheckFMatrix(std::vector<std::shared_ptr<sfm::CameraModel>> &cameras) {
    auto &camera1 = cameras[0];
    auto &camera2 = cameras[1];
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create();
    std::vector<cv::DMatch> matches;
    std::vector<cv::Point2i> points1;
    std::vector<cv::Point2i> points2;
    std::vector<Eigen::Vector2d> e_points1;
    std::vector<Eigen::Vector2d> e_points2;
    Eigen::Matrix3d F_my;

    matcher->match(camera1->descriptors_, camera2->descriptors_, matches);
    for (const auto &match: matches) {
        points1.emplace_back(camera1->key_points_[match.queryIdx].pt);
        points2.emplace_back(camera2->key_points_[match.trainIdx].pt);
        e_points1.emplace_back(camera1->key_points_[match.queryIdx].pt.x, camera1->key_points_[match.queryIdx].pt.y);
        e_points2.emplace_back(camera2->key_points_[match.trainIdx].pt.x, camera2->key_points_[match.trainIdx].pt.y);
    }
    std::cout << "Begin to compute the Fundamental matrix" << std::endl;
    cv::Mat F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC);
    sfm::MathFunction::ComputeFundamentalMatrix(e_points1, e_points2, F_my);
    std::cout << F_my << std::endl << std::endl;
    std::cout << F << std::endl << std::endl;
    Eigen::Matrix3d F_cv;
    F_cv << F.at<double>(0, 0), F.at<double>(0, 1), F.at<double>(0, 2),
            F.at<double>(1, 0), F.at<double>(1, 1), F.at<double>(1, 2),
            F.at<double>(2, 0), F.at<double>(2, 1), F.at<double>(2, 2);
    double residual1 = ComputeResidual(e_points1, e_points2, F_my);
    double residual2 = ComputeResidual(e_points1, e_points2, F_cv);
    std::cout << residual1 << ' ' << residual2 << std::endl;
}

int main() {
    int scene_graph[IMAGE_NUMBER][IMAGE_NUMBER];
//    std::string file_path_linux = "/home/dcr/codes/CorC++/colmap/testImage/gerrard-hall/newimage/";
    clock_t begin;
    clock_t end;
    std::string file_path_linux = "../newimage/";
    std::vector<std::shared_ptr<sfm::CameraModel>> cameras;

    sfm::ImgLoader loader{file_path_linux, cameras, true};
    sfm::CorrespondenceGraph correspondence_graph(cameras, false);
    std::cout << "Have Finished the correspondence graph build" << std::endl;
    sfm::IncrementalRebuild rebuild(&correspondence_graph);
    rebuild.BeginRebuild();
//    cout << "I just want to test my push " << std::endl;
    return 0;
}
