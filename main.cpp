#include <iostream>
#include <algorithm>
#include <string>

#include <opencv2/core.hpp>

#include "img_loader.h"
#include "camera_model.h"
#include "edge.h"
#include "point.h"

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

bool Init(std::vector<sfm::Edge>::iterator edge,
          sfm::Points &points) {
    /** 给一个相机的初始位置 **/
    cv::Mat initial_R = (cv::Mat_<float>(3, 3) << 1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);

    cv::Mat initial_t = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);

    edge->SetInitialCameraPose(initial_R, initial_t);
    edge->Triangulation(points, sfm::kTriangulation);
}

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

int main() {
//    std::string file_path_windows = "D:\\imageDataset\\gerrard-hall\\gerrard-hall\\loader\\*";
    int pass_geometry_test_num = 0;
    std::string file_path_linux = "/home/dcr/codes/CorC++/colmap/testImage/gerrard-hall/images/";

    std::vector<sfm::CameraModel> cameras{};
    std::vector<sfm::Edge> edges{};

    sfm::Points points{};

    sfm::ImgLoader loader{file_path_linux, cameras};
    /** 目前暂时先写一个简单的配对先基本能看一看效果 **/
    // TODO 更加完善的图像树的构建
    for (auto camera1 = cameras.begin(); camera1 != cameras.end(); ++camera1) {
        for (auto camera2 = camera1 + 1; camera2 != cameras.end(); ++camera2) {
            if (camera1->key_ != camera2->key_) {
                edges.emplace_back(camera1, camera2);
            }
        }
    }

    std::cout << "The size of the edges is " << edges.size() << std::endl;
    std::sort(edges.begin(), edges.end(), sort_edges);
    for (auto &edge: edges) {
        edge.ComputeMatrix();
        edge.EstimatePose();
    }

    std::vector<sfm::Edge>::iterator begin_edge;
    for (begin_edge = edges.begin(); begin_edge != edges.end() && !begin_edge->PassGeometryTest(); begin_edge++);

    if (begin_edge == edges.end()) {
        std::cerr << "unable to find the edge that pass the geometry test" << std::endl;
        return -1;
    }

    Init(begin_edge, points);
    points.ViewPoints();
    return 0;
}
