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

        if (!InitialParameters()) {
            std::cerr << "Unable the initial the parameters!" << std::endl;
        }
    }

    Edge::Edge(const Edge &input_edge) {
        camera1_ = std::move(input_edge.camera1_);
        camera2_ = std::move(input_edge.camera2_);

        if (!InitialParameters()) {
            std::cerr << "Unable the initial the parameters!" << std::endl;
        }
    }

    bool Edge::InitialParameters() {
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
        GetPoints();
//        std::cout << key_points_1_.size() << ' ' << key_points_2_.size() << std::endl;
        return true;
    }

    int Edge::ComputeKey(int index1, int index2) {
        int a = index1 > index2 ? index1 : index2;
        int b = index1 + index2 - a;
        return a * 100 + b;
    }

    void Edge::ComputeMatrix() {
        this->e_m_ = cv::findEssentialMat(this->key_points_1_, this->key_points_2_, this->camera1_->K_, cv::RANSAC);
        this->f_m_ = cv::findFundamentalMat(this->key_points_1_, this->key_points_2_, cv::FM_RANSAC);
        this->h_m_ = cv::findHomography(this->key_points_1_, this->key_points_2_, cv::RANSAC, 3);
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
            this->key_points_1_.push_back(this->camera1_->key_points_[match.queryIdx].pt);
            this->key_points_2_.push_back(this->camera2_->key_points_[match.trainIdx].pt);
        }
    }

    /**
     * 计算从key1 到 key2之间的位姿变化，这里是用对极几何计算的
     */
    void Edge::EstimatePose() {
        cv::recoverPose(this->e_m_, this->key_points_1_, this->key_points_2_, R_, t_);
//        std::cout << this->R_ << std::endl;
//        std::cout << this->t_ << std::endl;
    }

    void
    Edge::Triangulation(Points &points,
                        TrianguleType type) {
        if (camera1_ == nullptr || camera2_ == nullptr || camera1_ == camera2_) {
            std::cerr << "Fail to find the target camera" << std::endl;
            return;
        }

        if (type == kTriangulation) {
//            std::cout << "The initial R is " << std::endl << R_ << std::endl << std::endl;
//            std::cout << "The initial t is " << std::endl << t_ << std::endl;
//            std::cout << t_.cols << ' ' << t_.rows << std::endl;
            // initial the pose of the second camera
            camera2_->SetCameraPose(R_, t_);
            camera2_->T_.at<double>(0, 3) = camera2_->T_.at<double>(2, 3);
            camera2_->T_.at<double>(1, 3) = camera2_->T_.at<double>(2, 3);
            camera2_->T_.at<double>(2, 3) = 1.0f;
            std::cout << "The initial camera1 pose is " << std::endl << camera1_->T_ << std::endl << std::endl;
            std::cout << "The initial camera2 pose is " << std::endl << camera2_->T_ << std::endl << std::endl;

            std::vector<cv::Point2f> camera_point_1;
            std::vector<cv::Point2f> camera_point_2;
            std::vector<cv::Point2f> camera_inliers_1;
            std::vector<cv::Point2f> camera_inliers_2;

            std::vector<cv::Point3f> world_point;

            /** 将像素坐标转化到相机坐标系下 **/
            PixToCam(camera1_->K_, this->key_points_1_, camera_point_1);
            PixToCam(camera2_->K_, this->key_points_2_, camera_point_2);
            CleanOutliers(camera_point_1, camera_point_2, camera_inliers_1, camera_inliers_2);
            std::cout << "The size of the two inliers is " << camera_inliers_1.size() << " and "
                      << camera_inliers_2.size() << std::endl;

            cv::Mat pst_4d;

            if (camera_inliers_1.size() != camera_inliers_2.size() ||
                camera_inliers_1.size() * camera_inliers_2.size() == 0) {
                std::cerr << std::endl;
                std::cerr << "the size of the two points is not the same" << std::endl;
                std::cerr << "The size of the input key points can not be zero" << std::endl;
                std::cerr << this->key_points_1_.size() << ' ' << this->key_points_2_.size() << std::endl;
                return;
            }

            cv::triangulatePoints(camera1_->T_, camera2_->T_,
                                  camera_inliers_1, camera_inliers_2,
                                  pst_4d);
            std::cout << "Have finished the triangulation " << std::endl;
//            std::cout << pst_4d << std::endl;
//            std::cout << pst_4d.cols << ' ' << pst_4d.rows << std::endl << ' ' << pst_4d.type() << std::endl;
            for (int i = 0; i < pst_4d.cols; ++i) {
//                std::cout << pst_4d.col(i) << std::endl;
                world_point.emplace_back(cv::Point3f{pst_4d.at<float>(0, i) / pst_4d.at<float>(3, i),
                                                     pst_4d.at<float>(1, i) / pst_4d.at<float>(3, i),
                                                     pst_4d.at<float>(2, i) / pst_4d.at<float>(3, i)});
//                std::cout << world_point.back() << std::endl;
            }
            std::cout << "Have finished computing the position of the feature points" << std::endl;
            points.AddCloudPoint(world_point);
        }
    }

    void Edge::PixToCam(cv::Mat &K, std::vector<cv::Point2i> &input_points, std::vector<cv::Point2f> &output_points) {
        if (K.cols != 3 || K.rows != 3) {
            std::cerr << "the col or the row of the input K isn't equal to 3" << std::endl;
            return;
        }
        cv::Mat inverted_K;
        cv::invert(K, inverted_K);

        for (auto point: input_points) {
            cv::Mat temp_input_point = (cv::Mat_<float>(3, 1) << point.x, point.y, 1.0f);
            cv::Mat temp_output_point = inverted_K * temp_input_point;
            output_points.emplace_back(temp_output_point.at<float>(0, 0),
                                       temp_output_point.at<float>(1, 0));
        }
    }

    bool Edge::PassGeometryTest() {
        if (this->key_points_1_.size() != this->key_points_2_.size()) {
            std::cerr << "the size of the key point is not equal" << std::endl;
            return false;
        }
        int number = 0;
        if (key_points_1_.empty()) {
            std::cerr << "The size of the image can not be zero" << std::endl;
            std::cerr << key_ << std::endl;
            std::cerr << camera1_->key_ << ' ' << camera2_->key_ << std::endl << std::endl;
            return false;
        }
        for (int i = 0; i < this->key_points_1_.size(); ++i) {
            cv::Mat temp_points_1 = (cv::Mat_<float>(3, 1) << static_cast<float>(key_points_1_[i].x),
                    static_cast<float>(key_points_1_[i].y), 1.0f);
            cv::Mat temp_points_2 = (cv::Mat_<float>(3, 1) << static_cast<float>(key_points_2_[i].x),
                    static_cast<float>(key_points_2_[i].y), 1.0f);
            cv::Mat check;

            temp_points_1.convertTo(temp_points_1, this->f_m_.type());
            temp_points_2.convertTo(temp_points_2, this->f_m_.type());
            // 使用对极几何验证,同时这里也是计算对应的外点数
            check = temp_points_2.t() * this->f_m_ * temp_points_1;

//            std::cout << fabs(check.at<float>(0, 0)) << std::endl;
            if (fabs(check.at<float>(0, 0)) <= FUNDAMENTAL_INLIER_THRESHOLD) {
                number++;
            }
        }
        if (static_cast<float>(number) / static_cast<float>(this->key_points_1_.size()) >=
            FUNDAMENTAL_MATRIX_INLIERS_THRESHOLD) {
            return true;
        } else {
            return false;
        }
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
            if (check_result.at<float>(0, 0) <= 1.0f) {
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
            if (check_result.at<float>(0, 0) <= 1.0f) {
                number++;
            }
        }
        return number;
    }

    int Edge::GetHInlierNumber() {
        return 0;
    }

    void Edge::SetInitialCameraPose(const cv::Mat &_R, const cv::Mat &_t) {
        camera1_->SetCameraPose(_R, _t);
    }

    void Edge::CleanOutliers(std::vector<cv::Point2f> &outliers_point1, std::vector<cv::Point2f> &outliers_point2,
                             std::vector<cv::Point2f> &inliers_point1, std::vector<cv::Point2f> &inliers_point2) {
        for (int i = 0; i < outliers_point1.size(); ++i) {
            cv::Mat temp_point1 = (cv::Mat_<float>(3, 1) << outliers_point1[i].x, outliers_point1[i].y, 1.0f);
            cv::Mat temp_point2 = (cv::Mat_<float>(3, 1) << outliers_point2[i].x, outliers_point2[i].y, 1.0f);

            temp_point1.convertTo(temp_point1, e_m_.type());
            temp_point2.convertTo(temp_point2, e_m_.type());

            cv::Mat check_result = temp_point2.t() * e_m_ * temp_point1;
            if (check_result.at<float>(0, 0) <= ESSENTIAL_INLIER_THRESHOLD) {
                inliers_point1.emplace_back(outliers_point1[i].x, outliers_point1[i].y);
                inliers_point2.emplace_back(outliers_point2[i].x, outliers_point2[i].y);
//                std::cout << "The first point is " << outliers_point1[i].x << ' ' << outliers_point1[i].y << std::endl;
//                std::cout << "The second point is " << outliers_point2[i].x << ' ' << outliers_point2[i].y << std::endl
//                          << std::endl;
            }
        }
        std::cout << inliers_point1.size() << ' ' << inliers_point2.size() << std::endl;
    }
} // sfm