//
// Created by dcr on 22-7-9.
//

#include "camera_model.h"
#include "mysql.h"


namespace sfm {
    CameraModel::CameraModel(cv::Mat &_image, int _key) {
        if (!InitialParameters(_image, _key)) {
            std::cerr << "Initial parameters fail camera builed failed in sfm::CameraModel::CameraModel" << std::endl;
        } else {
            std::cout << "camera model build succeed " << std::endl;
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

    /** 在使用sql时，CameraModel 只有key和K，T还有用，只用初始化这些，其他信息可从后面得到**/
    CameraModel::CameraModel(int key, int row_size, int col_size, cv::Mat &_image) {
        cv::Mat temp =
                (cv::Mat_<float>(3, 3) << row_size / 2, 0.0f, row_size / 2,
                        0.0f, row_size / 2, col_size / 2,
                        0.0f, 0.0f, 1.0f);
        temp.copyTo(this->K_);
        this->K_.convertTo(this->K_, 6);
        this->key_ = key;
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> size;
        std::vector<float> angle;
        std::vector<int> response;
        std::vector<int> octave;
        std::vector<int> class_id;
        std::vector<int> r;
        std::vector<int> g;
        std::vector<int> b;
        sfm::SQLHandle::getAllKeyPoints(key, x, y, size, angle, response, octave, class_id, r, g, b);
        for (int i = 0; i < x.size(); ++i) {
            key_points_.emplace_back(x[i], y[i], size[i], angle[i], response[i], octave[i], class_id[i]);
            colors_.emplace_back(r[i], g[i], b[i]);
        }
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
        sift->compute(_image, this->key_points_, this->descriptors_);
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
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> size;
        std::vector<float> angle;
        std::vector<int> response;
        std::vector<int> octave;
        std::vector<int> class_id;
        std::vector<int> r;
        std::vector<int> g;
        std::vector<int> b;

        sift->detect(_image, this->key_points_);
        sift->compute(_image, this->key_points_, this->descriptors_);
        if (this->key_points_.empty()) {
            std::cout << "this image can not find enough key points_" << std::endl;
            std::cout << this->key_ << std::endl;
        }
        for (const auto &point: this->key_points_) {
            int _x = static_cast<int>(point.pt.x);
            int _y = static_cast<int>(point.pt.y);
            colors_.emplace_back(_image.at<cv::Vec3b>(_x, _y)[2],
                                 _image.at<cv::Vec3b>(_x, _y)[1],
                                 _image.at<cv::Vec3b>(_x, _y)[0]);
            x.emplace_back(_x);
            y.emplace_back(_y);
            size.emplace_back(point.size);
            angle.emplace_back(point.angle);
            response.emplace_back(point.response);
            octave.emplace_back(point.octave);
            class_id.emplace_back(point.class_id);
            r.emplace_back(_image.at<cv::Vec3b>(_x, _y)[2]);
            g.emplace_back(_image.at<cv::Vec3b>(_x, _y)[1]);
            b.emplace_back(_image.at<cv::Vec3b>(_x, _y)[0]);
        }
        cv::Mat temp =
                (cv::Mat_<float>(3, 3) << _image.cols / 2, 0.0f, _image.cols / 2,
                        0.0f, _image.cols / 2, _image.rows / 2,
                        0.0f, 0.0f, 1.0f);
        temp.copyTo(this->K_);
        this->K_.convertTo(this->K_, 6);
//        std::cout << x.size() << std::endl;
        if (sfm::SQLHandle::addKeyPoint(_key, x, y, size, angle, response, octave, class_id, r, g, b)) {
            return true;
        } else {
            std::cerr << "add key points to sql failed" << std::endl;
            return false;
        }
        return true;
    }

    void CameraModel::RefreshCameraParam(double *new_para) {
        K_.at<double>(0, 0) = new_para[0];
        K_.at<double>(0, 1) = new_para[1];
        K_.at<double>(0, 2) = new_para[2];
        K_.at<double>(1, 0) = new_para[3];
        K_.at<double>(1, 1) = new_para[4];
        K_.at<double>(1, 2) = new_para[5];
        K_.at<double>(2, 0) = new_para[6];
        K_.at<double>(2, 1) = new_para[7];
        K_.at<double>(2, 2) = new_para[8];

        T_.at<double>(0, 0) = new_para[9];
        T_.at<double>(0, 1) = new_para[10];
        T_.at<double>(0, 2) = new_para[11];
        T_.at<double>(0, 3) = new_para[12];
        T_.at<double>(1, 0) = new_para[13];
        T_.at<double>(1, 1) = new_para[14];
        T_.at<double>(1, 2) = new_para[15];
        T_.at<double>(1, 3) = new_para[16];
        T_.at<double>(2, 0) = new_para[17];
        T_.at<double>(2, 1) = new_para[18];
        T_.at<double>(2, 2) = new_para[19];
        T_.at<double>(2, 3) = new_para[20];
    }
}
