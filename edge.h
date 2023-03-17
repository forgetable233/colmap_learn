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
#include "thresholds.h"
#include "point2d.h"
#include "mysql.h"

namespace sfm {

    struct Point {
        Eigen::Vector2d point{};
        Eigen::Vector3i color{};
    };

    class Edge {
    private:
        bool InitialParameters(const bool copy);

    public:
        int key_ = 0;
        int e_m_inliers_ = 0;
        int f_m_inliers_ = 0;
        int h_m_inliers_ = 0;

        std::vector<cv::DMatch> matches_;

        std::vector<bool> is_F_inliers_;

        std::vector<cv::Point2i> key_points_1_;
        std::vector<cv::Point2i> key_points_2_;

        std::vector<Eigen::Vector3d> colors1_;
        std::vector<Eigen::Vector3d> colors2_;

        cv::Mat e_m_;
        cv::Mat f_m_;
        cv::Mat h_m_;
        cv::Mat R_;
        cv::Mat t_;

        std::shared_ptr<CameraModel> camera1_;
        std::shared_ptr<CameraModel> camera2_;

        bool joined = false;

        Edge() = default;

        Edge(Edge &input_edge);

        Edge(std::vector<CameraModel>::iterator _camera1, std::vector<CameraModel>::iterator _camera2);

        Edge(const std::shared_ptr<CameraModel> &_camera1, const std::shared_ptr<CameraModel> &_camera2,
             bool use_sql);

        Edge(int key1, int key2);

        ~Edge() = default;

        static int ComputeKey(int index1, int index2);

        void ComputeMatrix();

        void GetPoints(std::vector<sfm::CameraModel>::iterator camera1,
                       std::vector<sfm::CameraModel>::iterator camera2);

        void GetPoints(bool use_sql);

        void CleanOutliers();

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
