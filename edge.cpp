//
// Created by dcr on 22-7-9.
//

#include "edge.h"

struct find_key {
    explicit find_key(int _key) : key{_key} {};

    bool operator()(const sfm::CameraModel &itr) const {
        return itr.key_ == key;
    }

    int key;
};
namespace sfm {
    Edge::Edge(std::vector<CameraModel>::iterator _camera1,
               std::vector<CameraModel>::iterator _camera2) {
        this->camera1_ = std::make_shared<CameraModel>(_camera1);
        this->camera2_ = std::make_shared<CameraModel>(_camera2);

        if (!InitialParameters(false)) {
            std::cerr << "Unable the initial the parameters!" << std::endl;
        }
    }

    Edge::Edge(Edge &input_edge) {
        camera1_ = input_edge.camera1_;
        camera2_ = input_edge.camera2_;
        this->e_m_ = std::move(input_edge.e_m_);
        this->f_m_ = std::move(input_edge.f_m_);
        this->h_m_ = std::move(input_edge.h_m_);
        this->R_ = std::move(input_edge.R_);
        this->t_ = std::move(input_edge.t_);
        this->key_points_1_ = std::move(input_edge.key_points_1_);
        this->key_points_2_ = std::move(input_edge.key_points_2_);
        this->colors1_ = std::move(input_edge.colors1_);
        this->colors2_ = std::move(input_edge.colors2_);
        if (!InitialParameters(true)) {
            std::cerr << "Unable the initial the parameters!" << std::endl;
        }
    }

    Edge::Edge(const std::shared_ptr<CameraModel> &_camera1,
               const std::shared_ptr<CameraModel> &_camera2) {
        this->camera1_ = _camera1;
        this->camera2_ = _camera2;

        if (!InitialParameters(false)) {
            std::cerr << "Unable the initial the parameters!" << std::endl;
        }
    }

    bool Edge::InitialParameters(const bool copy) {
        if (camera1_ == nullptr || camera2_ == nullptr) {
            std::cerr << " The input camera is nullptr" << std::endl;
            return false;
        }
        int norm_type = cv::NORM_L2;

        bool cross_check = true;

        cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(norm_type, cross_check);

        int index1 = camera1_->key_;
        int index2 = camera2_->key_;

        key_ = ComputeKey(index1, index2);

        if (index1 < index2) {
            std::shared_ptr<CameraModel> temp = camera2_;
            camera2_ = camera1_;
            camera1_ = temp;
        } else if (index1 == index2) {
            std::cerr << "you can not match two same image" << std::endl;
            std::cerr << index1 << ' ' << index2 << std::endl;
            return false;
        }
        matcher->match(camera1_->descriptors_, camera2_->descriptors_, this->matches_);
        if (!copy) {
            GetPoints();
            ComputeMatrix();
            EstimatePose();
            CleanOutliers();
        }
        return true;
    }

    int Edge::ComputeKey(int index1, int index2) {
        int a = index1 > index2 ? index1 : index2;
        int b = index1 + index2 - a;
        return a * 100 + b;
    }

    void Edge::ComputeMatrix() {
        this->e_m_ = cv::findEssentialMat(this->key_points_1_,
                                          this->key_points_2_,
                                          this->camera1_->K_,
                                          cv::RANSAC);
        this->f_m_ = cv::findFundamentalMat(this->key_points_1_,
                                            this->key_points_2_,
                                            cv::FM_RANSAC);
        this->h_m_ = cv::findHomography(this->key_points_1_,
                                        this->key_points_2_,
                                        cv::RANSAC,
                                        3);
    }

    /**
     * 从两个目标相机中复制对应的点的信息
     * @param camera1
     * @param camera2
     */
    void Edge::GetPoints(std::vector<sfm::CameraModel>::iterator camera1,
                         std::vector<sfm::CameraModel>::iterator camera2) {
        for (auto match: this->matches_) {
            this->key_points_1_.push_back(camera1->key_points_[match.queryIdx].pt);
            this->key_points_2_.push_back(camera2->key_points_[match.trainIdx].pt);
        }
    }

    void Edge::GetPoints() {
        for (auto match: this->matches_) {
            this->key_points_1_.emplace_back(this->camera1_->key_points_[match.queryIdx].pt);
            this->key_points_2_.emplace_back(this->camera2_->key_points_[match.trainIdx].pt);
            this->colors1_.emplace_back(this->camera1_->colors_[match.queryIdx]);
            this->colors2_.emplace_back(this->camera2_->colors_[match.trainIdx]);
        }
    }

    /**
     * 计算从key1 到 key2之间的位姿变化，这里是用对极几何计算的
     */
    void Edge::EstimatePose() {
        if (this->key_points_1_.size() != this->key_points_2_.size()) {
            std::cerr << "The size of the two image must be the same" << std::endl;
        }
        if (this->key_points_1_.size() * this->key_points_2_.size() == 0) {
            std::cerr << "The size of the input key points_ can not be zero" << std::endl;
        }
        cv::recoverPose(this->e_m_, this->key_points_1_, this->key_points_2_, R_, t_);
    }

    bool Edge::PassGeometryTest() {
        if (this->key_points_1_.size() != this->key_points_2_.size()) {
            std::cerr << "the size of the key point is not equal" << std::endl;
            return false;
        }
        int number = 0;
        int size = this->key_points_1_.size();
        if (key_points_1_.empty() || key_points_2_.empty()) {
            std::cerr << "The size of the image can not be zero" << std::endl;
            std::cerr << key_ << std::endl;
            std::cerr << camera1_->key_ << ' ' << camera2_->key_ << std::endl << std::endl;
            return false;
        }
        /** 现在直接把所有的外点删了，应该没啥问题 **/
        auto point1 = key_points_1_.begin();
        auto point2 = key_points_2_.begin();

        for (; point1 != key_points_1_.end() && point2 != key_points_2_.end();
               ++point1, ++point2) {
            cv::Mat temp_point_1 = (cv::Mat_<float>(3, 1) << static_cast<float>(point1->x),
                    static_cast<float>(point1->y), 1.0f);
            cv::Mat temp_point_2 = (cv::Mat_<float>(3, 1) << static_cast<float>(point2->x),
                    static_cast<float>(point2->y), 1.0f);
            cv::Mat check;

            temp_point_1.convertTo(temp_point_1, this->f_m_.type());
            temp_point_2.convertTo(temp_point_2, this->f_m_.type());
        }
        this->f_m_inliers_ = number;
        if (static_cast<double>(number) / static_cast<double>(size) >=
            FUNDAMENTAL_MATRIX_INLIERS_THRESHOLD) {
            return true;
        }
        return false;
    }

    int Edge::GetEInlierNumber() {
        int number = 0;

        cv::Mat temp_camera1_K;
        cv::Mat temp_camera2_K;
        cv::Mat temp_point1(cv::Size(3, 3), e_m_.type(), 1.0f);
        cv::Mat temp_point2(cv::Size(3, 3), e_m_.type(), 1.0f);
        cv::Mat check_result;

        camera1_->K_.copyTo(temp_camera1_K);
        camera2_->K_.copyTo(temp_camera2_K);
        cv::invert(temp_camera2_K, temp_camera2_K);
        for (int i = 0; i < key_points_1_.size(); ++i) {
            temp_point1.at<float>(0, 0) = static_cast<float>(key_points_1_[i].x);
            temp_point1.at<float>(1, 0) = static_cast<float>(key_points_1_[i].y);
            temp_point2.at<float>(0, 0) = static_cast<float>(key_points_2_[i].x);
            temp_point2.at<float>(1, 0) = static_cast<float>(key_points_2_[i].y);

            check_result = temp_point2.t() * temp_camera2_K * e_m_ * temp_camera1_K * temp_point2;
            if (check_result.at<double>(0, 0) <= ESSENTIAL_INLIER_THRESHOLD) {
                number++;
            }
        }
        return number;
    }

    int Edge::GetFInlierNumber() {
        int number = 0;
        cv::Mat temp_point1(cv::Size(3, 1), f_m_.type(), 1.0f);
        cv::Mat temp_point2(cv::Size(3, 1), f_m_.type(), 1.0f);
        cv::Mat check_result;

        for (int i = 0; i < key_points_1_.size(); ++i) {
            temp_point1.at<float>(0, 0) = static_cast<float>(key_points_1_[i].x);
            temp_point1.at<float>(1, 0) = static_cast<float>(key_points_1_[i].y);
            temp_point2.at<float>(0, 0) = static_cast<float>(key_points_2_[i].x);
            temp_point2.at<float>(1, 0) = static_cast<float>(key_points_2_[i].y);

            check_result = temp_point2.t() * f_m_ * temp_point1;
            if (check_result.at<double>(0, 0) <= FUNDAMENTAL_INLIER_THRESHOLD) {
                number++;
            }
        }
        return number;
    }

    int Edge::GetHInlierNumber() {
        return 0;
    }

    void Edge::SetInitialCameraPose(const cv::Mat &R, const cv::Mat &_t) {
        camera2_->SetCameraPose(R, _t);
    }

    /**
     * 判断一个匹配是否是外点
     */
    void Edge::CleanOutliers() {
        is_F_inliers_.resize(this->matches_.size());
        for (int i = 0; i < matches_.size(); ++i) {
            cv::Mat temp_point1 = (cv::Mat_<float>(3, 1) <<
                    camera1_->key_points_[matches_[i].queryIdx].pt.x,
                    camera1_->key_points_[matches_[i].queryIdx].pt.y,
                    1.0f);
            cv::Mat temp_point2 = (cv::Mat_<float>(3, 1) <<
                    camera2_->key_points_[matches_[i].trainIdx].pt.x,
                    camera2_->key_points_[matches_[i].trainIdx].pt.y,
                    1.0f);

            temp_point1.convertTo(temp_point1, f_m_.type());
            temp_point2.convertTo(temp_point2, f_m_.type());

            cv::Mat check_result = temp_point1.t() * f_m_ * temp_point2;
            if (check_result.at<float>(0, 0) <= FUNDAMENTAL_MATRIX_INLIERS_THRESHOLD) {
                is_F_inliers_[i] = true;
                f_m_inliers_++;
            }
        }
        std::cout << "The size of the f_m_inliers is " << f_m_inliers_ << std::endl;
    }
} // sfm