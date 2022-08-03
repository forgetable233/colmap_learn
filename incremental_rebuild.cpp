//
// Created by dcr on 22-7-15.
//

#include "incremental_rebuild.h"

namespace sfm {

    IncrementalRebuild::IncrementalRebuild(CorrespondenceGraph *_graph) {
        scene_graph_ = std::make_shared<CorrespondenceGraph>(_graph);
    }

    void IncrementalRebuild::BeginRebuild() {
        int second_max = 0;
        int begin_pair = this->GetBestBeginEdge(second_max);
        int next_image;

        std::shared_ptr<std::vector<cv::Point3f>> world_point = std::make_shared<std::vector<cv::Point3f>>();

        if (Init(begin_pair)) {
            std::cout << "Have finished the initialize" << std::endl;
        } else {
            std::cerr << "Error occurred unable to initialize" << std::endl;
        }
        while ((next_image = scene_graph_->GetBestNextImage()) != -1) {
            auto camera = scene_graph_->GetCameraModel(next_image);
            RegisterImage(next_image);
            MultiViewTriangulation(next_image, kSVD);
            joined_images_.push_back(next_image);
            scene_graph_->SetImageRegistered(next_image);
            scene_graph_->ComputeScore(next_image);
            std::cout << "Have added a new image" << std::endl;
        }
        std::cout << "Have finished the rebuild" << std::endl;
        std::cout << world_points_.size() << std::endl;
        BA();
        ViewAllPoints();
    }

    int IncrementalRebuild::GetBestBeginEdge(int &second_max) {
        return scene_graph_->GetBestBeginPair(second_max);
    }

    bool IncrementalRebuild::Init(int index) {
        this->SingalViewTriangulation(index, kInitial);
        return true;
    }

    void IncrementalRebuild::PixToCam(cv::Mat &K,
                                      std::vector<cv::Point2f> &input_points,
                                      std::vector<cv::Point2f> &output_points) {
        if (K.cols != 3 || K.rows != 3) {
            std::cerr << "the col or the row of the input K isn't equal to 3" << std::endl;
            return;
        }
        cv::Mat inverted_K;
        cv::invert(K, inverted_K);

        for (auto point: input_points) {
            cv::Mat temp_input_point = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.0f);
            cv::Mat temp_output_point = inverted_K * temp_input_point;
            output_points.emplace_back(temp_output_point.at<double>(0, 0),
                                       temp_output_point.at<double>(1, 0));
        }
    }

    void IncrementalRebuild::SingalViewTriangulation(int index, TrianguleType type) {
        auto camera1 = scene_graph_->GetCameraModel(index, CameraChoice::kCamera1);
        auto camera2 = scene_graph_->GetCameraModel(index, CameraChoice::kCamera2);
        auto edge = scene_graph_->GetEdge(index);

        if (camera1 == nullptr || camera2 == nullptr || camera2 == camera1) {
            std::cerr << "Fail to find the target camera" << std::endl;
            return;
        }
        if (type == kInitial) {
            // initial the pose of the second camera
            camera2->SetCameraPose(edge->R_, edge->t_);
            scene_graph_->SetImageRegistered(camera1->key_);
            scene_graph_->SetImageRegistered(camera2->key_);
            joined_images_.push_back(camera2->key_);
            joined_images_.push_back(camera1->key_);
            scene_graph_->ComputeScore(camera1->key_);
            scene_graph_->ComputeScore(camera2->key_);
            /** 坐标系没有发生变化 **/
            camera1->T_.at<double>(0, 0) = 1.0f;
            camera1->T_.at<double>(1, 1) = 1.0f;
            camera1->T_.at<double>(2, 2) = 1.0f;
            camera2->T_.at<double>(0, 3) = camera2->T_.at<double>(0, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(1, 3) = camera2->T_.at<double>(1, 3) / camera2->T_.at<double>(2, 3);
            camera2->T_.at<double>(2, 3) = 1.0f;
            /** 进行清外点以及向相机坐标系的转变 **/
            std::vector<cv::Point2f> clean_point_1;
            std::vector<cv::Point2f> clean_point_2;
            std::vector<cv::Point2f> camera_point_1;
            std::vector<cv::Point2f> camera_point_2;
            std::vector<Eigen::Vector3d> world_points;

            CleanOutliers(index, clean_point_1, clean_point_2);

            /** 将像素坐标转化到相机坐标系下 **/
            PixToCam(camera1->K_, clean_point_1, camera_point_1);
            PixToCam(camera2->K_, clean_point_2, camera_point_2);

            cv::Mat pst_4d;

            cv::triangulatePoints(camera1->T_, camera2->T_,
                                  camera_point_1, camera_point_2,
                                  pst_4d);
            CheckZDepthAndAddWorldPoints(camera1, camera2, pst_4d);
            std::cout << "Have finished the initial triangulation " << std::endl;
            scene_graph_->SetPairJoined(index);
            /** 针对完成三角化的点进行索引的构建 **/
        } else if (type == kNormal) {
            if (scene_graph_->ImageHasRegistered(camera1->key_)) {
                // 确定第一张图片没有加入进去
            } else {

            }
        }
    }

    void IncrementalRebuild::CleanOutliers(int index,
                                           std::vector<cv::Point2f> &clean_points_1,
                                           std::vector<cv::Point2f> &clean_points_2) {
        scene_graph_->GetInliers(index, clean_points_1, clean_points_2);
    }

    void IncrementalRebuild::CheckZDepthAndAddWorldPoints(const std::shared_ptr<CameraModel> &camera1,
                                                          const std::shared_ptr<CameraModel> &camera2,
                                                          cv::Mat &pst_4d) {
        cv::Mat P1 = camera1->K_ * camera1->T_;
        cv::Mat P2 = camera2->K_ * camera2->T_;
        cv::Mat temp_point = (cv::Mat_<double>(4, 1) << 0.0f, 0.0f, 0.0f, 1.0f);
        cv::Mat check1;
        cv::Mat check2;
        Eigen::Vector3d temp;
        int key = 0;
        int sum = 0;
        for (int i = 0; i < pst_4d.cols; ++i) {
            temp_point.at<double>(0, 0) = pst_4d.at<float>(0, i);
            temp_point.at<double>(1, 0) = pst_4d.at<float>(1, i);
            temp_point.at<double>(2, 0) = pst_4d.at<float>(2, i);
            temp_point.at<double>(3, 0) = pst_4d.at<float>(3, i);
            check1 = P1.row(2) * temp_point;
            check2 = P2.row(2) * temp_point;
            /** 测试不加上删除负数情况 **/
//            if (check1.at<double>(0, 0) >= 0 && check2.at<double>(0, 0) >= 0) {
            sum++;
            temp.x() = temp_point.at<double>(0, 0) / temp_point.at<double>(3, 0);
            temp.y() = temp_point.at<double>(1, 0) / temp_point.at<double>(3, 0);
            temp.z() = temp_point.at<double>(2, 0) / temp_point.at<double>(3, 0);
            std::shared_ptr<Point3d> world_ptr = std::make_shared<Point3d>(temp);
            // 将所有的相关点都加入到对应的相关点中
            scene_graph_->AddWorldPoints(camera1->key_, camera2->key_, i, world_ptr);
            key = ComputeWorldPointKey();
            world_points_.insert(std::pair<int, std::shared_ptr<Point3d>>(key, nullptr));
            world_points_.at(key) = world_ptr;
//            }
        }
        std::cout << "The size of the initial points is " << world_points_.size() << std::endl;
    }

    void IncrementalRebuild::CheckZDepthAndAddWorldPoints(const std::unordered_map<int, std::vector<int>> &points,
                                                          const std::vector<Eigen::Vector3d> &world_points) {
        std::vector<std::shared_ptr<CameraModel>> cameras;
        cameras.resize(IMAGE_NUMBER);
        int i = 0;
        int world_point_key;
        for (const auto &point: points) {
            cameras.clear();
            int camera_key = CorrespondenceGraph::GetCameraKeyByPoint(point.first);
            cameras.emplace_back(scene_graph_->GetCameraModel(camera_key));
            for (const auto &corrs: point.second) {
                camera_key = CorrespondenceGraph::GetCameraKeyByPoint(corrs);
                cameras.emplace_back(scene_graph_->GetCameraModel(camera_key));
            }
            std::shared_ptr<Point3d> world_ptr = std::make_shared<Point3d>(world_points[i++]);
            scene_graph_->AddWorldPoints(point.first, point.second, world_ptr);
            world_point_key = ComputeWorldPointKey();
            world_points_.insert(std::pair<int, std::shared_ptr<Point3d>>(world_point_key, nullptr));
            world_points_.at(world_point_key) = world_ptr;
        }
    }

    void IncrementalRebuild::ViewAllPoints() {
        std::vector<Eigen::Vector3d> world_points;
        Eigen::Vector3d temp_point;
        for (const auto &point: world_points_) {
            temp_point = point.second->GetPoint();
            world_points.emplace_back(temp_point.x(), temp_point.y(), temp_point.z());
        }
        if (world_points.empty()) {
            std::cerr << "The size of the world points is zero" << std::endl;
        } else {
            std::shared_ptr<PointViewer> viewer = std::make_shared<PointViewer>(world_points);
            viewer->ViewPoints();
        }
    }

    int IncrementalRebuild::ComputeWorldPointKey() {
        return this->world_points_.size() + 1;
    }

    void IncrementalRebuild::ShowMatchResult(int begin_index) {

    }

    /**
     * 使用PnP算法计算出对应图像的世界坐标位置（这里预计会用自己写的方法计算一下）
     * @param R
     * @param t
     * @param camera_key
     */
    void IncrementalRebuild::RegisterImage(int camera_key) {
        auto camera = scene_graph_->GetCameraModel(camera_key);
        if (camera != nullptr) {
            cv::Mat local_R;
            cv::Mat _t;
            cv::Mat dist_coeff = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
            std::vector<cv::Point3f> world_points;
            std::vector<cv::Point2f> image_points;
            if (scene_graph_->GetRelatedPoints(camera_key, world_points, image_points)) {
                cv::solvePnPRansac(world_points, image_points, camera->K_, dist_coeff, local_R, _t);
                cv::Mat temp_R;
                cv::Rodrigues(local_R, temp_R);
                camera->SetCameraPose(temp_R, _t);
                _t.col(0).copyTo(camera->T_.col(3));
                temp_R.col(0).copyTo(camera->T_.col(0));
                temp_R.col(1).copyTo(camera->T_.col(1));
                temp_R.col(2).copyTo(camera->T_.col(2));
            } else {
                std::cerr << "Can not get the related points" << std::endl << std::endl;
            }
        } else {
            std::cerr << "Can not find the related pair" << std::endl << std::endl;
        }
    }

    void IncrementalRebuild::ComputeScore(int camera_key) {
        scene_graph_->ComputeScore(camera_key);
    }

    void IncrementalRebuild::MultiViewTriangulation(int camera_key, MultiViewType type) {
        std::vector<Eigen::Vector3d> world_points;
        std::unordered_map<int, std::vector<int>> new_image_points;
        std::map<int, Eigen::Matrix<double, 3, 4>> P;
        for (const auto &joined_image: joined_images_) {
            GetUnregisteredPoints(joined_image, camera_key, new_image_points);
        }
        if (type == kSVD) {
            /** 利用SVD加上多视角进行一个多视角三角化的计算，算是传统三角化方法，只是增加了加入点的量 **/
            int i = 0;
            Eigen::Matrix<double, Eigen::Dynamic, 3> function;
            Eigen::Matrix<double, 3, 4> temp_P;
            for (const auto &joined_image: joined_images_) {
                scene_graph_->GetP(joined_image, temp_P);
                P.insert(std::pair<int, Eigen::Matrix<double, 3, 4>>(joined_image, temp_P));
            }
            scene_graph_->GetP(camera_key, temp_P);
            std::cout << temp_P << std::endl;
            P.insert(std::pair<int, Eigen::Matrix<double, 3, 4>>(camera_key, temp_P));
            for (const auto &image_point: new_image_points) {
                Eigen::Vector2d pixel_point;
                Eigen::Vector3d world_point;
                world_point = SVDComputeWorldPoint(image_point.first, new_image_points, P);
                world_points.push_back(world_point);
            }
            CheckZDepthAndAddWorldPoints(new_image_points, world_points);
        } else if (type == kRANSAC) {
            /** 此处是尝试复现colmap中，利用RANSAC进行多视图三角化的方法 **/
        }
    }

    /**
     * 将新加入的图片与已经加入的图片中没有完成三角化的点进行key值的获得
     * 并进行对应的三角化
     * @param old_camera_key
     * @param new_camera_key
     * @param points
     */
    void IncrementalRebuild::GetUnregisteredPoints(int old_camera_key, int new_camera_key,
                                                   std::unordered_map<int, std::vector<int>> &points) {
        int edge_key = CorrespondenceGraph::ComputeEdgeKey(old_camera_key, new_camera_key);
        const auto edge = scene_graph_->GetEdge(edge_key);
        std::vector<Eigen::Vector2d> temp_old_points;
        std::vector<Eigen::Vector2d> temp_new_points;
        if (edge->key_ / 100 == old_camera_key) {
            for (const auto &match: edge->matches_) {
                int new_point_key = CorrespondenceGraph::ComputePointKey(new_camera_key, match.trainIdx);
                int old_point_key = CorrespondenceGraph::ComputePointKey(old_camera_key, match.queryIdx);
                if (!scene_graph_->PointHasRegistered(new_point_key) &&
                    !scene_graph_->PointHasRegistered(old_point_key)) {
                    if (points.find(new_point_key) == points.end()) {
                        std::vector<int> temp;
                        points.insert(std::pair<int, std::vector<int>>(new_point_key, temp));
                    }
                    points.at(new_point_key).push_back(old_point_key);
                }
            }
        } else {
            for (const auto &match: edge->matches_) {
                int new_point_key = CorrespondenceGraph::ComputePointKey(new_camera_key, match.queryIdx);
                int old_point_key = CorrespondenceGraph::ComputePointKey(old_camera_key, match.trainIdx);
                if (!scene_graph_->PointHasRegistered(new_point_key) &&
                    !scene_graph_->PointHasRegistered(old_point_key)) {
                    if (points.find(new_point_key) == points.end()) {
                        std::vector<int> temp;
                        points.insert(std::pair<int, std::vector<int>>(new_point_key, temp));
                    }
                    points.at(new_point_key).push_back(old_point_key);
                }
            }
        }
    }

    Eigen::Vector3d IncrementalRebuild::SVDComputeWorldPoint(int point_key,
                                                             const std::unordered_map<int, std::vector<int>> &image_points,
                                                             const std::map<int, Eigen::Matrix<double, 3, 4>> &P) {
        Eigen::Matrix<double, Eigen::Dynamic, 4> A((1 + image_points.at(point_key).size()) * 2, 4);
        Eigen::Matrix3d K;
        Eigen::Vector2d pixel_point;
        Eigen::Vector3d world_point;
        int camera_key = CorrespondenceGraph::GetCameraKeyByPoint(point_key);
        int i = 0;
        auto camera = scene_graph_->GetCameraModel(camera_key);
        scene_graph_->GetPixelPoint(point_key, pixel_point);
        cv::cv2eigen(camera->K_, K);
        /** 归一化 **/
        Eigen::Vector3d temp = K.inverse() * pixel_point.homogeneous();
        pixel_point.x() = temp.x() / temp.z();
        pixel_point.y() = temp.y() / temp.z();
        A.block<1, 4>(i++, 0) = P.at(camera_key).row(2) * pixel_point.x() - P.at(camera_key).row(0);
        A.block<1, 4>(i++, 0) = P.at(camera_key).row(2) * pixel_point.y() - P.at(camera_key).row(1);
        for (const auto &corr_point: image_points.at(point_key)) {
            camera_key = CorrespondenceGraph::GetCameraKeyByPoint(corr_point);
            scene_graph_->GetPixelPoint(corr_point, pixel_point);
            A.block<1, 4>(i++, 0) = P.at(camera_key).row(2) * pixel_point.x() - P.at(camera_key).row(0);
            A.block<1, 4>(i++, 0) = P.at(camera_key).row(2) * pixel_point.y() - P.at(camera_key).row(1);
        }
        Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector4d homo_world_point = svd.matrixV().col(2);
        homo_world_point(0) /= homo_world_point(3);
        homo_world_point(1) /= homo_world_point(3);
        homo_world_point(2) /= homo_world_point(3);
        world_point.x() = homo_world_point(0);
        world_point.y() = homo_world_point(1);
        world_point.z() = homo_world_point(2);
        return world_point;
    }

    /**
     * 对图像进行BA修正
     */
    void IncrementalRebuild::BA() {
        std::cout << "Begin to solve BA problem" << std::endl;
        ceres::Problem problem;
        ceres::Solver solver;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        std::unordered_map<int, std::vector<std::shared_ptr<Point2d>>> points;
        scene_graph_->GetCameraPoints(points);
        for (int i = 0; i < IMAGE_NUMBER; ++i) {
            auto camera = scene_graph_->GetCameraModel(i);
            double camera_params[21] = {camera->K_.at<double>(0, 0), camera->K_.at<double>(0, 1),
                                        camera->K_.at<double>(0, 2), camera->K_.at<double>(1, 0),
                                        camera->K_.at<double>(1, 1), camera->K_.at<double>(1, 2),
                                        camera->K_.at<double>(2, 0), camera->K_.at<double>(2, 1),
                                        camera->K_.at<double>(2, 2), camera->T_.at<double>(0, 0),
                                        camera->T_.at<double>(0, 1), camera->T_.at<double>(0, 2),
                                        camera->T_.at<double>(0, 3), camera->T_.at<double>(1, 0),
                                        camera->T_.at<double>(1, 1), camera->T_.at<double>(1, 2),
                                        camera->T_.at<double>(1, 3), camera->T_.at<double>(2, 0),
                                        camera->T_.at<double>(2, 1), camera->T_.at<double>(2, 2),
                                        camera->T_.at<double>(2, 3)};
            std::cout << std::endl;
            double pixel_point[points.at(i).size()][2];
            double world_point[points.at(i).size()][3];
            for (int j = 0; j < points.at(i).size(); j++) {
                points.at(i)[j]->GetPixelAndWorldPoint(pixel_point[i], world_point[j]);
                ceres::CostFunction *cost_function = ReprojectionError::Create(pixel_point[j][0], pixel_point[j][1]);
                ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
//                std::cout << world_point[j][0] << ' ' << world_point[j][2] << ' ' << world_point[j][2] << std::endl;

                problem.AddResidualBlock(cost_function, loss_function, camera_params, world_point[j]);
            }
            ceres::Solve(options, &problem, &summary);
            camera->RefreshCameraParam(camera_params);
            for (int j = 0; j < points.at(i).size(); ++j) {
                points.at(i)[j]->RefreshWorldPoint(world_point[j]);
            }
            std::cout << "Have finished the BA of the image " << i << std::endl;
        }
        std::cout << "Have finished the BA" << std::endl;
    }
}
