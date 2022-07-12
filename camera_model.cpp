//
// Created by dcr on 22-7-9.
//

#include "camera_model.h"

namespace sfm {
    CameraModel::CameraModel(cv::Mat &_image, int _key) {
        if (!InitialParameters(_image, _key)) {
            std::cerr << "Initial parameters fail" << std::endl;
        }
    }

    CameraModel::CameraModel(std::vector<CameraModel>::iterator iterator) {
        if (!InitialParameters(iterator->image_, iterator->key_)) {
            std::cerr << "Initial parameters fail" << std::endl;
        }
    }

    void CameraModel::SetCameraPose(const cv::Mat &R, const cv::Mat &_t) {
        if (R.rows != 3 || R.cols != 3 || _t.cols != 1 || _t.rows != 3) {
            std::cerr << "The input mat can not satisfy the requirement" << std::endl;
            return;
        }
        T_.at<float>(0, 0) = R.at<float>(0, 0);
        T_.at<float>(0, 1) = R.at<float>(0, 1);
        T_.at<float>(0, 2) = R.at<float>(0, 2);
        T_.at<float>(0, 3) = _t.at<float>(0, 0);

        T_.at<float>(1, 0) = R.at<float>(1, 0);
        T_.at<float>(1, 1) = R.at<float>(1, 1);
        T_.at<float>(1, 2) = R.at<float>(1, 2);
        T_.at<float>(1, 3) = _t.at<float>(1, 0);

        T_.at<float>(2, 0) = R.at<float>(2, 0);
        T_.at<float>(2, 1) = R.at<float>(2, 1);
        T_.at<float>(2, 2) = R.at<float>(2, 2);
        T_.at<float>(2, 3) = _t.at<float>(2, 0);
    }

    bool CameraModel::InitialParameters(cv::Mat &_image, int _key) {
        if (_image.empty()) {
            std::cerr << "The input image is empty" << std::endl;
            return false;
        }
        _image.copyTo(this->image_);
        this->key_ = _key;

        int nfeatures{0};
        int nOctaveLayers{3};

        double contrastThreshold{0.04};
        double edgeThreshold{10};
        double sigma{1.6};

        cv::Ptr<cv::SIFT> sift = cv::SIFT::create(nfeatures,
                                                  nOctaveLayers,
                                                  contrastThreshold,
                                                  edgeThreshold,
                                                  sigma);

        sift->detect(this->image_, this->key_points_);
        sift->compute(this->image_, this->key_points_, this->descriptors_);
        if (this->key_points_.size() == 0) {
            std::cout << "this image can not find enough key points" << std::endl;
            std::cout << this->key_ << std::endl;
        }
        cv::Mat temp = (cv::Mat_<float>(3, 3) << this->image_.cols / 2, 0.0f, this->image_.cols / 2,
                0.0f, this->image_.cols / 2, this->image_.rows / 2,
                0.0f, 0.0f, 1.0f);
        temp.copyTo(this->K_);
        return true;
    }
}
