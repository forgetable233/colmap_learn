#include <iostream>
#include <algorithm>
#include <string>

#include "img_loader.h"
#include "camera_model.h"
#include "edge.h"
#include "point.h"
#include "math_functions.h"
#include "threholds.h"
#include "incremental_rebuild.h"

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

void DisplayMatchResult(std::vector<sfm::CameraModel> &cameras, std::vector<sfm::Edge> &edges) {
    for (auto edge: edges) {
        int key1 = edge.key_ / 100;
        int key2 = edge.key_ % 100;

        auto camera1 = std::find_if(cameras.begin(), cameras.end(), get_key(key1));
        auto camera2 = std::find_if(cameras.begin(), cameras.end(), get_key(key2));

        if (camera1 == camera2 || camera1 == cameras.end() || camera2 == cameras.end()) {
            std::cerr << "unable to find the target camera" << std::endl;
        }

        cv::Mat match_image;
        cv::drawMatches(camera1->image_, camera1->key_points_,
                        camera2->image_, camera2->key_points_,
                        edge.matches_, match_image);
        cv::imshow("the match image", match_image);
        cv::waitKey(0);
    }
}

void CVPoint2iToVector3d(const std::vector<cv::Point2i> &point2i, std::vector<Eigen::Vector3d> &vector3d) {
    for (auto point: point2i) {
        vector3d.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y), 1.0f);
    }
}

void CompareEssentialMatrix(sfm::Edge &edge) {
    edge.ComputeMatrix();
    double variance = 0.0f;
    Eigen::Matrix3d E;
    std::vector<Eigen::Vector3d> points1;
    std::vector<Eigen::Vector3d> points2;

    CVPoint2iToVector3d(edge.key_points_1_, points1);
    CVPoint2iToVector3d(edge.key_points_2_, points2);

    sfm::MathFunction::ComputeEssentialMatrix(points1, points2, E);
    std::cout << "The type of the essential matrix is " << edge.e_m_.type() << std::endl;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            variance += (E(i, j) - edge.e_m_.at<double>(i, j)) *
                        (E(i, j) - edge.e_m_.at<double>(i, j));
        }
    }
    std::cout << std::endl << "The opencv result is " << edge.e_m_ << std::endl << std::endl;
    std::cout << std::endl << "My version is " << E << std::endl << std::endl;

    std::cout << "The variance of the two matrix is " << variance << std::endl;
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
        for (int j = i + 1; j < IMAGE_NUMBER; ++j) {
            sfm::Edge temp_edge{cameras[i], cameras[j]};
            if (temp_edge.PassGeometryTest()) {
                edges.push_back(std::make_shared<sfm::Edge>(temp_edge));
                scene_graph[cameras[i]->key_][cameras[j]->key_] = edges.size() - 1;
            }
        }
    }
}

int main() {
//    std::string file_path_windows = "D:\\imageDataset\\gerrard-hall\\gerrard-hall\\loader\\*";
    int scene_graph[IMAGE_NUMBER][IMAGE_NUMBER];
    std::string file_path_linux = "/home/dcr/codes/CorC++/colmap/testImage/gerrard-hall/images/";

    std::vector<std::shared_ptr<sfm::CameraModel>> cameras;
    std::vector<std::shared_ptr<sfm::Edge>> edges;

//    sfm::Points points_{};

    std::shared_ptr<sfm::Points> points = std::make_shared<sfm::Points>();

    sfm::ImgLoader loader{file_path_linux, cameras};
    BuildSceneGraph(edges, cameras, scene_graph);
    std::cout << "Have found " << edges.size() << " edges_" << std::endl;

    sfm::IncrementalRebuild rebuild{edges, points, scene_graph};
    rebuild.BeginRebuild();
    return 0;
}
