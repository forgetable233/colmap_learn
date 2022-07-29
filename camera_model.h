//
// Created by dcr on 22-7-9.
//

#ifndef TEST_CAMERA_MODEL_H
#define TEST_CAMERA_MODEL_H

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sfm {
    class CameraModel {
    private:
        bool InitialParameters(cv::Mat &_image, int _key);

    public:
        int key_{};

        std::vector<cv::KeyPoint> key_points_{};

        cv::Mat K_{cv::Size{3, 3}, CV_64F, cv::Scalar{0}};

        cv::Mat T_{cv::Size{4, 3}, CV_64F, cv::Scalar {0}};

        cv::Mat descriptors_{};

        CameraModel() = default;

        CameraModel(cv::Mat &_image, int _key);

        CameraModel(std::vector<CameraModel>::iterator iterator);

        void SetCameraPose(const cv::Mat &_R, const cv::Mat &_t);

        ~CameraModel() = default;
    };
}
#endif //TEST_CAMERA_MODEL_H
