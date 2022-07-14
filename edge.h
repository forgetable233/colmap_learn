//
// Created by dcr on 22-7-9.
//

#ifndef TEST_EDGE_H
#define TEST_EDGE_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include "camera_model.h"
#include "point.h"
#include "threholds.h"

namespace sfm {

    enum TrianguleType {
        kTriangulation = 0,
        kNormal = 1,
    };

    class Edge {
    private:
        bool InitialParameters(const bool copy);

    public:
        int key_{};

        std::vector<cv::DMatch> matches_;

        std::vector<cv::Point2i> key_points_1_{};
        std::vector<cv::Point2i> key_points_2_{};

        cv::Mat e_m_;
        cv::Mat f_m_;
        cv::Mat h_m_;
        cv::Mat R_;
        cv::Mat t_;

        std::shared_ptr<CameraModel> camera1_;
        std::shared_ptr<CameraModel> camera2_;

        Edge() = default;

        Edge(const Edge &input_edge);

        Edge(std::vector<CameraModel>::iterator _camera1, std::vector<CameraModel>::iterator _camera2);

        Edge(const std::shared_ptr<CameraModel> &_camera1, const std::shared_ptr<CameraModel> &_camera2);

        ~Edge() = default;

        static int ComputeKey(int index1, int index2);

        void ComputeMatrix();

        void GetPoints(std::vector<sfm::CameraModel>::iterator camera1,
                       std::vector<sfm::CameraModel>::iterator camera2);

        void GetPoints();

        void Triangulation(sfm::Points &points,
                           TrianguleType type = kNormal);

        void PixToCam(cv::Mat &K,
                      std::vector<cv::Point2i> &input_points,
                      std::vector<cv::Point2f> &output_points);

        void CleanOutliers(std::vector<cv::Point2f> &outliers_point1, std::vector<cv::Point2f> &outliers_point2,
                           std::vector<cv::Point2f> &inliers_point1, std::vector<cv::Point2f> &inliers_point2);

        /**
         * check if the match satisfy the epipolar geometry
         * @return
         */
        bool PassGeometryTest();

        void EstimatePose();

        void SetInitialCameraPose(const cv::Mat &_R, const cv::Mat &_t);

        void ComputeAngle();

        int GetMiddleAngle();

        int GetEInlierNumber();

        int GetFInlierNumber();

        int GetHInlierNumber();
    };

} // sfm

#endif //TEST_EDGE_H
