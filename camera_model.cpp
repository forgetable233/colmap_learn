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
        this->T_ = std::move(iterator->T_);
        this->K_ = std::move(iterator->K_);
        iterator->descriptors_.copyTo(this->descriptors_);
        this->key_ = iterator->key_;
        std::copy(iterator->key_points_.begin(),
                  iterator->key_points_.end(),
                  this->key_points_.begin());
    }

    void CameraModel::SetCameraPose(const cv::Mat &R, const cv::Mat &_t) {
        if (R.rows != 3 || R.cols != 3 || _t.cols != 1 || _t.rows != 3) {
            std::cerr << "The input mat can not satisfy the requirement" << std::endl;
            return;
        }
        R.col(0).copyTo(T_.col(0));
        R.col(1).copyTo(T_.col(1));
        R.col(2).copyTo(T_.col(2));
        _t.col(0).copyTo(T_.col(3));
    }

    bool CameraModel::InitialParameters(cv::Mat &_image, int _key) {
        if (_image.empty()) {
            std::cerr << "The input image is empty" << std::endl;
            return false;
        }
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

        sift->detect(_image, this->key_points_);
        sift->compute(_image, this->key_points_, this->descriptors_);
        if (this->key_points_.size() == 0) {
            std::cout << "this image can not find enough key points_" << std::endl;
            std::cout << this->key_ << std::endl;
        }
        cv::Mat temp = (cv::Mat_<float>(3, 3) << _image.cols / 2, 0.0f, _image.cols / 2,
                0.0f, _image.cols / 2, _image.rows / 2,
                0.0f, 0.0f, 1.0f);
        temp.copyTo(this->K_);
        return true;
    }
}
